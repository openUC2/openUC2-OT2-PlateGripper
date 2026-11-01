# Identity board: makes the OT-2 see the ONiLAB tool as a P20 Single-Channel GEN2

No microcontroller is needed for the pipette itself. The OT-2 pipette cable carries everything
the robot needs: I²C to an EEPROM (who is attached), the plunger home switch, and the 4 stepper
wires that the robot's own driver (Allegro A5984, 1/32 microstep, ~36 V) drives directly.

Two ways to answer the robot's EEPROM read:
- **A: a 24C02 EEPROM** (below, "Parts", "Programming the EEPROM"): the cheapest and simplest.
- **B: an ESP32-S3 that pretends to be the EEPROM** (XIAO ESP32-S3 + a BSS138 level shifter,
  section "Option B"): no EEPROM to buy, the ID is set over USB.

## OT-2 pipette connector (2×5, 2.54 mm, shrouded; 3M 30310-6002HB on Opentrons pipettes)

Checked in Opentrons' published EAGLE files (github.com/Opentrons/ot2, "PCB files"):
`pipette-main.sch` (connector JP1 on the pipette board) and the motor board `5001-stepper-drivers`
(JP4 = left mount, plunger axis B; JP7 = right mount, plunger axis C). Cable length 0.5 m.

| Pin | Net on the robot | Inside an Opentrons pipette | On our tool |
|---|---|---|---|
| 1 | SCL, 5 V I²C (10 k pull-up on the robot) | EEPROM AT24MAC402 SCL | 24C02 pin 6 |
| 2 | SDA, 5 V I²C | EEPROM SDA | 24C02 pin 5 |
| 3 | ADDRESS: tied to GND on the left mount (JP4), to 5 V on the right mount (JP7) | EEPROM A0 → 0x50 / 0x51 | 24C02 pin 1 (A0) |
| 4 | MIN (B_MIN_IN / C_MIN_IN, pulled up on the motor board) | end-stop board: D2F-L COM | switch COM |
| 5 | GND | GND | 24C02 pins 2, 3, 4, 7 (A1, A2, GND, WP); switch NC |
| 6 | +5 V | via a PNP (reverse-polarity switch) to the EEPROM | 24C02 pin 8 (VCC) + 100 nF |
| 7 | MOTOR_2A | motor JST pin 4 | coil B, one end |
| 8 | MOTOR_2B | motor JST pin 3 | coil B, other end |
| 9 | MOTOR_1A | motor JST pin 1 | coil A, one end |
| 10 | MOTOR_1B | motor JST pin 2 | coil A, other end |

```
header on the tool, looking into it (standard IDC numbering, pin 1 = triangle = red stripe)
        ┌──────┐  key  ┌──────┐
        │  1    3    5    7    9  │     SCL   ADDR  GND   M2A   M1A
        │  2    4    6    8   10  │     SDA   MIN   +5V   M2B   M1B
        └─────────────────────────┘
```

Switch: COM → MIN, NC → GND (Opentrons' `pipette-endstop.sch`: D2F-L, 1 nF across). At rest MIN
is held low; pressed (yoke at the top) it opens and the robot's pull-up reads "home".
**Verify pin 1 on your robot before connecting** (checks below).

## Where the robot's plug lands

On an Opentrons GEN2 single the 2×5 socket is on the pipette's mount-side face, near one edge
of the 44.5 mm face, with its long side vertical (Science-Jubilee assembly guide photos). The
robot's ribbon comes down the mount bar, is clipped at the bar's top block and its plug is pushed
in sideways just below the gantry cover (OT-2R manual, "attach pipette"). Scaled from the
photos, the socket centre sits about 18-21 mm above the axis of the two top mount screws and
about 9-10 mm from the face edge. V0.2 puts our header 30 mm above those screws (window in the
mount wall, `hdr_y`, `hdr_z` in `cad/pipette_tool.py`) so the board clears the pipette ledge;
the 0.5 m ribbon has slack at its clip. **Measure on the robot and move the window if needed.**

## Motor: YPF Motors 20K-2AB-12.7

Size-8 (20 mm) captive linear stepper, 1 mm lead (stroke 12.7 mm, M2.5 × 5 shaft end, M2 holes
on 15.4 mm, connector on one side near the rear). 200 steps → the robot's 3200 µsteps per robot-mm
give **0.5 mm of yoke travel per robot-mm** (6400 µsteps per real mm).

- Measure both coil resistances (pairs: the two wires with a few Ω between them) and look up or
  ask YPF for the rated current. `ot2/onilab_tool.py` sets run 0.5 A; the robot's GEN2 idle 0.3 A
  and homing 1.0 A cannot be changed (homing lasts a few seconds).
- Coil A → pins 9/10, coil B → pins 7/8. If the yoke moves down when homing, swap 7 ↔ 8.
- The connector sits on the −z side in the frame. Use a right-angle plug or solder the leads; a
  straight plug sticks ~5 mm out of the bar depth.

## Parts

| Qty | Part | Note |
|---|---|---|
| 1 | 24C02 EEPROM, SOIC-8 or DIP-8 (AT24C02, M24C02, ...) | 8-byte pages; any 2 kbit 24xx works |
| 1 | 2×5 shrouded header 2.54 mm | through the window in the mount wall, facing the mount |
| 1 | 100 nF ceramic | |
| 1 | Omron D2F-L or similar 12.8 × 5.8 × 6.5 mm lever micro switch | 2 × M2 self-tapping into the wall |
| 1 | JST-XH 4-pin + 2-pin | motor and switch |
| 1 | perfboard ~33 × 11 mm | 2 × M2.5 self-tapping into the wall (holes 27.9 mm apart, above and below the header) |

## Programming the EEPROM

Use `../firmware/ot2_eeprom_writer/ot2_eeprom_writer.ino` on a Seeed XIAO (RP2040 or SAMD21) or any
Arduino, off the robot, with the EEPROM powered from 3.3 V and A0 to GND (wiring in the sketch).
It writes the layout SmoothiewareOT expects: `opentron` signature at 0x00, ID at 0x30, model
`p20_single_v2.2` at 0x60. Give every tool its own ID: the robot stores calibration and the
plunger settings under it. (Opentrons pipettes keep WP high with 1 k and two buttons; tying WP
to GND is fine, the robot does not write the EEPROM in normal use.)

## Option B: an ESP32-S3 instead of the EEPROM

Yes, this works. The robot only ever **reads** the EEPROM, and in a way an I²C slave can follow
easily. SmoothiewareOT (`src/modules/utils/instrument/Instrument.cpp`) runs the bus at **10 kHz**.
Every access is two separate transactions: it writes the 1-byte memory pointer (STOP), waits
10 ms, then reads 8 bytes (STOP) and waits 10 ms. M369 checks the 8-byte `opentron` signature
at 0x00, then reads the ID 0x30–0x4F; M371 reads the model at 0x60. The 7-bit addresses are
0x50 (left) and 0x51 (right). The robot writes only on M370 / M372 / M374 (pointer + 8 bytes),
which normal use does not send.

On the ESP32-S3, arduino-esp32 (checked in 3.3.12, `esp32-hal-i2c-slave.c`) holds SCL low after a
read request until `onRequest()` has filled the TX FIFO, and drops unread bytes at the STOP. So
the sketch `../firmware/ot2_eeprom_emulator_esp32s3/` only has to:
- keep the pointer from the 1-byte write;
- answer 8 bytes from a 256-byte image;
- store 9-byte writes, persisted in NVS like an EEPROM.

It compiles for `esp32:esp32:XIAO_ESP32S3` with arduino-esp32 3.3.12. **It is not yet tested on
hardware.** The classic ESP32 cannot hold the clock this way: use an S2/S3/C3-class chip.

| Ribbon pin | Goes to | Note |
|---|---|---|
| 6 (+5 V) | XIAO 5V pin, shifter HV | ~30 mA at 80 MHz, radio off (estimate; measure pin 6 under load, see below) |
| 5 (GND) | XIAO GND, shifter GND, divider bottom | |
| — | XIAO 3V3 → shifter LV | |
| 1 (SCL) | shifter HV1, LV1 → XIAO D5 (GPIO6) | **the bus is 5 V with 10 k pull-ups on the robot: never wire it straight to the ESP32** |
| 2 (SDA) | shifter HV2, LV2 → XIAO D4 (GPIO5) | |
| 3 (ADDRESS) | 10 k → XIAO D0 (GPIO1), 15 k from D0 to GND | 0 V = left → 0x50, 5 V → 3.0 V = right → 0x51; read at boot |
| 4, 7–10 | switch and motor, as in option A | the XIAO is not involved |

Parts beyond option A's header, switch and JST plugs:
- 1 × XIAO ESP32-S3.
- 1 × BSS138 2-channel level shifter. Any "I²C logic level converter" board works; its 10 k
  pull-ups in parallel with the robot's give ~5 k, which is fine at 10 kHz. Two BSS138s and
  four 10 k resistors on the perfboard work too.
- Resistors: 10 k and 15 k.

The 24C02 and its 100 nF are not needed.

**Setting it up**

1. Flash the sketch over USB. Open the serial monitor (115200, LF): `show` prints the address,
   ID and model. Every board starts with its own ID derived from the chip's MAC
   (`P20SV22` + 10 digits). `id <text>` sets your own; `model p20_single_v2.2` is the default.
   The robot stores the plunger settings and calibration under the ID.
2. Bench test without the robot: run `../firmware/ot2_eeprom_writer` on a second board as the
   I²C master. It reads exactly like the robot. Wire it directly at 3.3 V (SDA, SCL, GND,
   4.7 k pull-ups to 3V3; leave the emulator's D0 low). `r` must print `signature: ok` with the
   ID and model.
3. On the robot: first measure pin 6 with the board attached; it must stay above 4.75 V. Pin 6
   was meant for an EEPROM behind a PNP, and its current budget is not documented. If it sags,
   power the XIAO from the robot's USB (as the plate gripper does) and keep only GND, I²C and
   ADDRESS on the ribbon. The XIAO boots in well under a second; the robot reads the EEPROM when
   it scans instruments (start-up, homing, protocol start, App refresh). Then continue with the
   checks below. `GET http://<robot-ip>:31950/instruments` (header `Opentrons-Version: 3`)
   shows the model and ID the robot read.

**Where it sits**: not in the CAD yet. Two places look free. Both stay within the GEN2 outline,
and neither has been checked against the parts in CAD yet:
- A 4.5 mm pocket in the mount wall's inner face, between the header board and the motor
  shelf (y ≈ 93–114, +z side, opposite the switch). The yoke disc passes ~0.9 mm away.
- Flat on top of the motor under a small printed cap: 11.6 mm headroom up to the
  estimated GEN2 top datum, so this waits for the datum measurement.

## Checks on the robot before plugging the tool in

1. Unplug the ribbon from a real pipette. Measure 5 V between pins 6 and 5 (red stripe = pin 1).
2. Pin 3 to pin 5: 0 V on the left mount, 5 V on the right mount.
3. Tool side: each motor coil pair reads its rated resistance; no short between coils.
4. Plug in, open the Opentrons App → the mount should show **P20 Single-Channel GEN2**.
5. Home: the yoke must move **up** toward the switch. If it moves down, swap one coil pair (7 ↔ 8).

## Why not the XIAO on I²C or USB for the plunger

- The I²C bus only carries the identity. The robot reads it while scanning instruments (10 kHz,
  8-byte chunks, both plunger motors switched off during each read). It never sends motion over it.
- The plunger axis is a normal stepper axis of the robot's motion controller. Driving our motor from
  those 4 wires gives native `aspirate / dispense / blow_out / home`, the App, Protocol Designer and
  calibration without any extra firmware.

## A second motor for the tip ejector (if over-travel is not used)

No board has to interpret the plunger steps. A second switch that the yoke reaches only below
blow-out (only `drop_tip()` goes there) tells a XIAO to run the ejector motor; a slow
`dropTipSpeed` over the last robot-mm gives it the time. Power: pin 6 is on whenever the robot is
on, but it feeds Opentrons' pipette board through a small PNP and its current budget is not
documented, so do not run a motor from it. USB from the robot (as the plate gripper does) gives
5 V / 0.5 A and a serial line. Options for the XIAO beyond that: plain serial from the protocol,
or imitating a Magnetic Module (VID:PID 04d8:ee90, `G28.2`, `G0 Z`, `M114.2`, `M115`).
