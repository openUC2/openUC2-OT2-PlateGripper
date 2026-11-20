# Plunger homing on the OT-2: what the robot does, why ours fails, how to test

The robot is told it has a P20 Single-Channel GEN2 and drives our YPF 20K-2AB-12.7 actuator
itself. This note puts together what Opentrons' code does around the plunger (sources at the
end) with what that means for our motor and switch. It also gives a test sequence that tells
the causes apart.

## What the robot does

**Scale.** A GEN2 single has 3200 microsteps per plunger-mm at 1/32 microstepping, i.e. 100
full steps per robot-mm. The YPF is a 1.8° hybrid with 0.005 mm per full step (part code "AB",
1 mm lead), so the yoke moves **0.5 mm per robot-mm**.

**Homing (`G28.2 B`, left mount; C for the right):**
1. Moves up at **25 robot-mm/s = 2500 full steps/s**, at `plungerCurrent`, until MIN reads high
   (switch open) for 3 ms. If that never happens within 60 robot-mm, the firmware stops with
   **`ALARM: Homing fail`**.
2. Backs off 2 robot-mm (1 mm of yoke), comes back at 2 robot-mm/s, and sets **B = 20 at the
   trigger**.
3. Backs off again to **B = 18**. The API then moves the plunger to `bottom` at **31 robot-mm/s
   (3100 full steps/s)**, the firmware's cap, and drops to the idle current, 0.3 A.

**During moves** the end-stops stay armed. If MIN reads high during a move, the robot stops with
**`ALARM: Hard limit +B`** (moving up) or **`-B`** (moving down) and re-homes. Every upward move
overshoots by 0.3 robot-mm and comes back.

**What you can set per pipette (HTTP):** top, bottom, blowout, dropTip, plungerCurrent,
dropTipCurrent, dropTipSpeed. Homing speed, retract and idle current are not settable that way.
A robot-wide **max speed for B/C** (`/data/robot_settings.json`) also limits the homing speed.

## What went wrong on our side

1. **Speed.** 2500 full steps/s for homing and 3100 for free moves are tuned for Opentrons' own
   motor at 1.0 A. A size-8 hybrid at 0.5 A loses steps or stalls there (Haydon's
   comparable 1 mm-lead curve: ~28 N at 2000, ~17 N at 2400 full steps/s, at 40 V).
   - A stall going up gives `Homing fail`.
   - Steps lost going down to `bottom` shift every later position, which shows up as "timing",
     volume drift, or a later `Hard limit`.
   - **Fix: cap the axis at 10 robot-mm/s** (1000 full steps/s) robot-wide
     (`ot2/plunger_bench.py --set-max-speed 10`). Aspirate/dispense run at ~5 robot-mm/s
     anyway.
2. **Our positions were 1 robot-mm (0.5 mm) too deep.** The switch trips at B = 20, not 19 (19 is
   only the API's starting value). With the old overrides (top 17.8, bottom 6.8, blowout 0.8):
   - `bottom` pressed 0.5 mm past the first stop;
   - `blowout` pushed 0.5 mm into the blow-out stop. Driving a stepper into a hard stop loses
     steps.
   - **Fixed** in `ot2/onilab_tool.py`: top 18.8, bottom 7.8, blowout 1.8, dropTip 1.8. Re-run
     `configure_pipette.py`.
3. **Currents.** The P20 GEN2 defaults are plungerCurrent 1.0 A and dropTipCurrent 1.0 A,
   twice the YPF's 0.5 A rating. `configure_pipette.py` sets 0.5 / 0.6 A. Check that the robot
   accepted them (`GET /settings/pipettes/{id}`). A rejected update leaves the defaults
   (bottom −8.5, dropTip −27), which drive the actuator into its lower stop right after
   homing: 13 mm below the trigger on a 12.7 mm stroke.
4. **The switch must release after the 1 mm back-off.** The D2F-L's movement differential is up
   to 0.5 mm, so the margin is 2×. A switch with a longer differential, or a tab that keeps
   resting on the lever, gives `Hard limit -B` on the move to `bottom`. That looks like a
   homing error. Keep the 1 nF capacitor across the switch (on the new board), and keep the
   switch wires away from the motor wires.
5. **Logic or wiring.** At rest MIN must read 0 (`M119`) and 1 with the lever pressed (COM →
   pin 4, **NC** → GND). The other way round, homing "succeeds" at once and the next move
   down alarms.
6. **Over-aspirating.** Above ~9.6 µl commanded with bottom 6.8, the target passes the switch.
   The `OnilabPipette` wrapper never commands more than 7.44 µl, so this only bites with raw
   `aspirate()` calls.

## Test sequence (`ot2/plunger_bench.py`, on the robot)

Open Jupyter at `http://<robot-ip>:48888`, then New → Terminal:

```
systemctl stop opentrons-robot-server
python3 plunger_bench.py --axis B         # left mount; C for the right
systemctl start opentrons-robot-server
```

The script runs these steps, printing each command and the controller's reply:
1. `M119` at rest and with the lever pressed (logic).
2. One home at 10 robot-mm/s; then `M114.2` must read B 18, and `M119` must read 0 (released).
3. 2 robot-mm down: measure **1.0 mm** of yoke travel. 4.2 mm would mean a 7.5° motor; 2 mm would
   mean a 2 mm lead.
4. Homing from the low end at 10 / 15 / 20 / 25 robot-mm/s. The first failure is the motor's limit.
5. 20 cycles between B4 and B17, then creeping near the switch. A `Hard limit` here means lost
   steps.

Then cap the speed below the first failure:
`python3 plunger_bench.py --axis B --set-max-speed 10`, and restart the robot server.

The new tool board also copies MIN to the XIAO (D3). With the emulator firmware, the USB console
prints every switch edge with a millisecond time stamp and counts bounces. Leave it plugged
into a laptop during a run to see exactly when the switch opened and closed around a failure.

**Logs over HTTP** (header `Opentrons-Version: *`):
`GET http://<robot-ip>:31950/logs/serial.log?records=5000` holds every G-code and reply. Search
it for `ALARM`, `G28.2`, `Hard limit`, `Homing fail`.

## Sources

- Opentrons v8.8.2: `api/src/opentrons/drivers/smoothie_drivers/driver_3_0.py`, `constants.py`,
  `hardware_control/api.py` (`_do_plunger_home`), `hardware_control/backends/controller.py`
  (`configure_mount`), `config/defaults_ot2.py`, `shared-data/.../p20/2_2.json`,
  `model_constants.py`.
- SmoothiewareOT at 8414642 (the build v8.8.2 flashes): `src/config.default`,
  `src/modules/tools/endstops/Endstops.cpp`.
- YPF 20K-2AB-12.7: Alibaba/Amazon listings (4.5 V, 0.5 A, 9 Ω, 37.35 mm); part code as Haydon/DINGS
  ("2" = 1.8°, "AB" = 0.005 mm/step); Haydon size 8 force curves.
