# OT2-Pipette V0.5 — ONiLAB 0.1–2.5 µl pipette as a native OT-2 tool

Design note: what was built, the decisions, the numbers and where they come from, the checks
and the open points. The shareable overview is [report.md](report.md).

## The ask

- **2026-10-30 (V0.1).** Include a more precise, low-cost manual pipette (ONiLAB single-channel
  0.1–2.5 µl) as a tool on the Opentrons OT-2, motorised (linear steppers; a second motor for the
  tip ejector is fine). Find out whether the OT-2's I²C pipette interface or USB should drive it,
  imitate the existing protocols, reuse the gripper's interface dimensions, look for prior art,
  build a 3D-printable first prototype.
- **2026-10-31 (V0.2).** The ONiLAB model's upper part with the hook is right, the tip tube may
  not be: estimate the pipette from product images. Confirm the connector pinout. Use the
  **YPF Motors 20K-2AB-12.7** (CAD-new BUY part, size 8, 1 mm lead) instead of the Joy-IT.
  Opentrons-style ejection or a second motor? Can Science-Jubilee tell where the mount sits on a
  GEN2 and where the ribbon plug lands? Tips: standard 10 µl tips. Convert the shared report to
  markdown in the repo.
- **2026-11-01 (V0.3).** Turn the Inventor files into truly editable, parametric parts and
  assemblies instead of imported solid bodies. Can an ESP32-S3 (plenty in stock) stand in for the
  I²C EEPROM?
- **2026-11-19 (V0.5).** The printed V0.3 was tested on the robot:
  - The screw holes and the connector do not match the robot. Is there a reference design?
  - The pipette could not be held as designed; the colleague changed the frame. Review that and
    update the assembly.
  - Design a small JLCPCB board for the XIAO, the connector and the motor and switch cables.
    Integrate it into the adapter.
  - Homing throws errors; the timing is off.
  - Does the motor hit the top limit? Is there a more compact motor with more stroke?

## What was built

| File | What |
|---|---|
| `cad/pipette_tool.py` | parametric CadQuery model + 45 checks + exports (`uv run --with cadquery python pipette_tool.py`); every number is a field of `P` |
| `cad/out/OT2-Pipette V0.5 - frame.step/.stl` | one printed part: mount wall with the 3 inserts, plug pocket + header window, top-block relief, sleeve + ledge, motor shelf with the board tabs; 32.3 × 205.3 × 37.5 mm, 59 cm³ |
| `cad/out/OT2-Pipette V0.5 - yoke.step/.stl` | rigid yoke on the actuator's M2.5 shaft end (Ruthex insert), pad face 0.5 mm above the plunger button, switch tab |
| `cad/out/OT2-Pipette V0.5 - yoke eject variant`, `- spring pad` | eject variant: skirt with a spring pad (2 M2 pins in slots) and an arm with an M3 adjust screw over the ejector button |
| `cad/out/OT2-Pipette V0.5 - tip stripper` + `openuc2_tipstripper_3_slot.json` | deck-slot stripper (slot 4.4 mm for 10 µl tips) and its OT-2 labware definition |
| `cad/out/OT2-Pipette V0.5 - assembly.step`, `- assembly eject.step` | frame, yoke(s), YPF actuator, switch, tool board, ONiLAB envelope |
| commits `c8f2271`, `aa6b085`, `9c05ed3`, `6761c79` | V0.1 (Joy-IT); V0.2 (Inventor files imported from STEP); V0.3 (native Inventor); V0.4 (colleague's changes, Inventor 2026) |
| `cad/ref/YPF-20K-2AB-12.7.stp` | the CAD-new motor, exported from Inventor |
| `cad/render_pipette_tool.py`, `cad/render_inventor.py` | the figures below (CadQuery; Inventor off-screen render of the native assemblies) |
| `cad/pipette_params.py` | the Inventor parameter table: the fields of `P` + the stack as expressions (223 parameters); checked against `stack()` |
| `cad/build_inventor.py` → `INVENTOR/OT2-Pipette V0.5 - parameters.ipt` + 7 native parts + the KiCad board + `OT2-Pipette V0.5.iam`, `- eject variant.iam` | **native, parametric Inventor model** (below) |
| `cad/check_native.py` | native parts vs CadQuery parts (STEP, volume + 20 000-point sampling) |
| `cad/cadlib/` | Inventor COM helpers (copy of openuc2-frame-opm's, plus linked parameter parts and `min(a; b)`) |
| `ot2/onilab_tool.py`, `configure_pipette.py`, `example_protocol_onilab.py` | robot numbers (1 mm lead, switch trips at B = 20), settings writer, real-µl protocol (passes `opentrons_simulate` 8.8.2) |
| `ot2/plunger_bench.py`, `docs/homing.md` | plunger test on the robot (G-code, Jupyter terminal) and the homing analysis |
| `electronics/pcb/` | **tool board**: KiCad 10 project built by `make.py` (netlist, layout, router), JLCPCB files, STEP |
| `electronics/README.md`, `firmware/ot2_eeprom_writer/` | pinout (checked against the EAGLE files), identity board, EEPROM writer (also the bench tester for the emulator) |
| `firmware/ot2_eeprom_emulator_esp32s3/` | **XIAO ESP32-S3 as the identity EEPROM** (option B in `electronics/README.md`) |

![views](pipette_tool_views.png)
![section](pipette_tool_section.png)
![head](pipette_tool_head.png)

## V0.5: what the printed prototype showed, and what changed

The V0.3 frame was printed and mounted on an OT-2 (2026-11-19). The motor and the switch work.
Four findings came out of that test:
- the screw holes and the connector did not line up with the robot;
- the pipette could not be held as designed;
- homing the plunger threw errors;
- it was unclear whether the motor hits the top when the mount is fully up.

A colleague changed the frame in Inventor (commit `6761c79`, files `OT2-Pipette V0.4 - frame.ipt`
/ `- yoke.ipt`, and edits to the V0.3 frame). The commit message lists:
- the frame widened for the end switch above the pipetting unit;
- the connector socket moved;
- a recess for the plain bearing;
- a recess for the round holder at the top of the mount;
- the switch under the motor moved.

### The robot's mount face, checked

Sources: Opentrons' own OT-2 reference model (`OT-2 Reference Model Detailed.STEP` in the
Opentrons/ot2 repository), the attach-pipette pictures in the Opentrons app and manual, the
GEN2 white paper drawing, and the Science-Jubilee OT-2 tool. The table uses the tool frame,
LEFT mount: y up from the frame origin, z toward the robot front. The top screws are at y 46.

| | Robot | V0.3 | V0.5 |
|---|---|---|---|
| Top screws (2), bottom screw | y 46, z ±13.17; y −43, z −13.17 (rear) | same (from the gripper adapter) | unchanged: **the holes were right** |
| 2×5 ribbon plug centre | **y 68.5, z +9.3** (22.5 above the screws, 9.3 toward the front; ±2) | y 76, z +13 | moved |
| Plug holder | sits in a pocket of the GEN2 face | flat face | 2.5 mm pocket around the window |
| Top block of the mount (plug holder, round black part) | a GEN2 clears it with a 5.5 mm recess 38–82 mm above the screws (y 84–128) | flat face hit it | 5.5 mm relief y 84–128 from z −5 forward; a rib keeps the wall stiff behind it |
| GEN2 top | ~126 mm above the screws (y ≈ 172; V0.2–V0.3 assumed 168) | — | motor top at y 156.4, **15.7 mm lower** |
| GEN2 depth along the bar | 44.5: front face flush with the bar front (z +18.66), rear at z −25.84 | ±18.75 used | unchanged (7 mm spare at the rear) |

**Why the V0.3 frame did not sit right.** The gripper adapter that donated the screw pattern ends
5 mm above the top screws, below the plug and the top block. The tall pipette frame did not end
there. Its flat face rested on both, so the screws could not pull it flat and the holes seemed to
be off. The colleague's "recess for the round holder" and "connector moved" are the same finding.

**Right mount:** a GEN2 is turned 180° there. The bottom screw is still at the rear (z +13.17 in
this frame) and the plug sits at the rear, z ≈ +16.5. V0.5 is the left-mount version; a
right-mount frame needs the bottom insert and the window moved (both are parameters).

**Does the motor hit the top?** Not by the numbers. A GEN2 homes with its top about 126 mm above
the screws; our motor ends 110 mm above them. When the mount goes up, what can collide is the
flat wall against the mount's own top block, which the relief now clears. Check it once anyway:
- take the gantry cover off;
- jog Z up slowly;
- watch the round black part and the plug holder.

### Tool board (electronics/pcb)

A KiCad 10 board in the spirit of Opentrons' pipette-main:
- the robot's 2×5 header, right angle, through the wall into the plug's position;
- the XIAO ESP32-S3;
- three BSS138 shifters (SCL, SDA, and MIN, so the XIAO can log the home switch);
- the ADDRESS divider, an ESD array, a Schottky to the XIAO;
- the motor and switch plugs, with a 1 nF on the switch as on Opentrons' end-stop board;
- a footprint for an optional 24C02.

It is 23.5 × 56 mm, two layers, and fully routed by script: DRC 0 errors, 0 unconnected. The
JLCPCB files (Gerbers, BOM, CPL) list only Basic or Preferred-Extended SMD parts, plus three
through-hole connectors.

**Where it sits:** upright beside the yoke on the +z side (copper z 13.75–15.35), component side
toward the pipette, screwed (2 × M2) to two tabs under the motor shelf. Its header face lands
0.55 mm inside the mount face at the plug position.

**XIAO socket:** the XIAO hangs 6 mm below the board on its own pin headers, 2 mm clear of the
yoke disc. A female socket would add 5–8.5 mm and hit the yoke inside the GEN2 outline. The pads
take sockets on a bench board.

**Eject variant:** its arm runs through the board's place, so it needs another layout.

![tool board](../electronics/pcb/out/render_iso.png)
![mount face](pipette_tool_inventor_mountface.png)

### Homing: see [homing.md](homing.md)

Three causes, all fixable without new parts:
- **Plunger positions 0.5 mm too deep.** The switch trips at B = 20, not 19, so `bottom` and
  `blowout` were pushing into the pipette's stops. Fixed in `ot2/onilab_tool.py`; re-run
  `configure_pipette.py`.
- **The robot's speeds.** Homing at 2500 and free moves at 3100 full steps/s are too fast for a
  size-8 at 0.5 A. Cap the axis at about 10 robot-mm/s; `ot2/plunger_bench.py` finds the real limit.
- **The switch.** It must release within the 1 mm back-off; keep the 1 nF across it.

`ot2/plunger_bench.py` checks each of these on the robot: logic, step size, speed ladder and cycles.

### Actuator

The YPF 20K-2AB-12.7 is confirmed as a 1.8° double-stack hybrid at 0.005 mm per step (1 mm lead):
0.5 A, 9 Ω, ~45 N. Its 12.7 mm stroke is enough for pipetting (blow-out at 8.5 of 11.2 mm usable)
but not for plunger ejection (≥ 18 mm).

| Option | Stroke | Height from the shaft shoulder to the motor top, mm | Note |
|---|---|---|---|
| YPF 20K-2AB-12.7 (today) | 12.7 | 53 | |
| Same family, 19.05 (DINGS/YPF "Kaptive") | 19.05 | ~59 | same seat and wiring; ask the seller for a drawing |
| Same family, 25.4 | 25.4 | ~70 (4.5 mm rear protrusion) | would even fit Opentrons' default positions |
| Haydon 21H4AB-2.5-907 | 19.05 | ~67 (rear screw) | real datasheet, drop-in seat |
| Nanotec LGA201S06-A-TDBA-019 | 19.05 | ~64 | buy today (~$152), 16 mm hole square: new seat |
| Non-captive size 8, 1 mm lead, coaxial | ~20 | ~56–61 | no captive sleeve; anti-rotation in the yoke |

Avoid can-stacks: under 30 N, and they stall at the drop-tip speed. With ~15 mm under the GEN2 top,
a 19.05 mm captive (+6 mm) fits. A 25.4 mm one would need the stack lowered or a frame that
grows past the GEN2 top.

### The colleague's V0.4 is in Inventor 2026

`OT2-Pipette V0.4 - frame.ipt` / `- yoke.ipt` and the edited `OT2-Pipette V0.3 - frame.ipt` were
saved by Inventor 2026 (build 300.175). Inventor 2025.5 (build 295.498, this machine) cannot open
them, nor can its Apprentice. Starting Inventor 2027, which is installed here, through automation
did not come up. So V0.5 carries the interface findings above, and the plain-bearing recess, the
widened frame and the moved switch are not yet in it. To merge them:
- **Option 1:** export STEP (AP214) of V0.4 frame and yoke from Inventor 2026, and list the
  changed parameter values.
- **Option 2:** open Inventor 2027 once interactively, so the build can read the 2026 files from
  there.

The edited V0.3 frame no longer opens in 2025, so `OT2-Pipette V0.3.iam` does not either. The V0.5
files are new and do not replace it.

## The native Inventor model (V0.3, rebuilt as V0.5)

![Inventor, eject variant](pipette_tool_inventor_eject.png)

- **One place to edit:** `INVENTOR/OT2-Pipette V0.3 - parameters.ipt` holds all 208 values as
  exported user parameters, with comments. That is every field of `P` plus the stack as
  expressions (`body_top = body_bottom + body_len`, `mot_front = retracted + shaft_out`,
  `shelf_x1 = min(...; ...)`, ...). Every part and both assemblies **link** it (Parameters →
  Link): change `datum_y` there, update the assembly, and everything moves.
- **Parts are real features.** Every sketch dimension, work-plane offset and extent is an
  expression over those names, so 0 dimensions are fixed numbers. The parts are frame, rigid
  yoke, eject yoke, spring pad, tip stripper, ONiLAB envelope, identity board and switch
  placeholders; their names say what they are, e.g. "sleeve and ledge", "body clearance",
  "ejector button slot", "ledge relief". The pipette outline is a real arc-and-line sketch. The
  sleeve and the clearance cut are offsets of it, driven by `wall_out` (= `clr + wall`) and `clr`.
- **The assemblies are constrained, not placed.** The frame is grounded. Its named work
  features (`WP motor seat`, `WP pipette x/z`, `WP mount face`, `WA insert 1-3`,
  `WA pipette axis`) carry the rest:
  - the CAD-new YPF actuator sits on the motor seat and the pipette axis;
  - the Ruthex inserts mate their insert axes, 5.7 mm into the mount face;
  - the other parts are flush with the frame's planes.
  The OT-2 reference model, gripper adapter and Dev-Core keep their offsets as assembly
  parameters. **Assembly parameter `travel`** (real mm below the robot's 'top') drives the yoke,
  and `min(travel; u_blowout)` drives the spring pad. Constraints → Drive on it shows the stroke.
- **Flex test** (every build): 15 values changed at once in the parameter part, including
  `datum_y`, `body_len`, `body_r`, `nose_r`, `knob_hole_d`, `wall`, `clr`, `mot_len`, `hdr_y`,
  `yoke_d`, `arm_r` and `st_slot_w`. All 8 parts rebuilt healthy and moved by the expected
  amounts; after reverting, every volume is back to the last 0.01 mm³.
- **Same geometry as CadQuery:** `check_native.py` compares the STEP of every native part with
  the CadQuery export. Yokes, pad, stripper, board and switch are identical. The frame differs
  by 1.4 mm³ in 65 340 mm³ and the envelope by 55 mm³, because CadQuery draws the pipette arcs
  as 24-segment polylines.
- Python stays the generator and the checker (the 40 checks run on the same values). Values
  you change in Inventor can be carried back with `cadlib.native_part.read_user_parameters`.
- The V0.2 imported bodies (`INVENTOR/OT2-Pipette V0.2*`, `REF - ONiLAB envelope.ipt`, ...)
  are superseded; they stay until the V0.2 assembly is closed in Inventor.

## What changed in V0.3

1. **Native, parametric Inventor model** (above) instead of STEP imports.
2. **Ledge relief:** the ledge between the two front screwdriver channels (15.4 × 8.5 mm, beside
   the knob hole) is cut away. Before, that cut left two loose 3 mm³ slivers that the CadQuery
   model silently dropped. Now the frame is one solid by construction, in both models.
3. **Every number in `P`:** the constants that were written into the geometry functions (wall
   fillet 4, gusset 2.5, clearances, hole sizes, stripper dimensions, ...) are fields with
   comments, so the Inventor table carries them.
4. **ESP32-S3 instead of the EEPROM (option B):** the sketch and wiring are in
   `electronics/README.md`. It compiles but is not yet tested on hardware.

## What changed from V0.1 (V0.2), and why

1. **The GEN2 top datum is ~115 mm above the gripper adapter's top, not level with it.** V0.1
   assumed level. Three independent estimates disagree with that:
   - the OT-2 reference model (`INVENTOR/00_OT-2 Reference_Pipettte.ipt`, green body) is the
     mount bar: 18 × 176.65 × 37.5 mm, top screw pair 26.33 mm apart, bottom screw 89 mm lower —
     the adapter's insert pattern. Its top is 77.65 mm above the top screws (y = 123.65);
   - OT-2R manual photo of a mounted P300 GEN2 (scale from the 32.34 mm face): the bar top sits
     just inside the gantry cover, the pipette continues ~46 mm higher, and the pipette body ends
     ~13 mm below the bar → top datum y ≈ 170;
   - Science-Jubilee guide photo of the GEN2's mount face (scale from its 44.5 mm width and the
     holder's 19.24 mm holes) and the manual photo of the plug at the bar's top block → socket
     ~18-21 mm above the top screws and ~96-103 mm below the top datum → y ≈ 166.

   V0.2 uses `datum_y = 168`. Consequences: P20 nozzle at y = −162.5 (V0.1: −277.5); the tool may
   use the GEN2 outline up to y = 168; with the V0.1 geometry the nozzle would have been ~115 mm
   too low. **One measurement on the robot settles it** (open point 1).
2. **Pipette geometry from the product photos** (ONiLAB = DLAB TopPette: Amazon part number
   `top-0.1-2.5`). Method: pixel profile along the axis of the main photo (lower part blue/black
   on white), axial scale from the Dev-Core body (bottom → top face 121 mm, nose ends at 100 mm:
   both within 1.5 % in the photo), diameters corrected for the ~22° tilt seen on the knob's end
   face; cross-checked on the upright photo (body width = Dev-Core D extent 31.6 mm).

   | | V0.1 | V0.2 (photo) |
   |---|---|---|
   | knob top above the body top face | 12 | 18.5 |
   | knob Ø × height / plunger stem Ø | 19 / – | 16.6 × 7 / 9.9 |
   | below the body: collar | – | Ø 18.1 × 12.6 |
   | taper / ejector sleeve | cone Ø 11.7 → 9.3 × 100 | 7.2 / Ø 9.4 → 8.6 × 62.6 |
   | black tip cone | – | Ø 4.7 → 2.4 × 19.1 (total below the body 101.5) |
   | ejector button | front window | D 12.3–21.5, W ±5, top 2.5 below the body top face |
   | volume window | +x side, 55–88 | **−x flat** (faces the mount), 33–56 above the body bottom |

   The window faces the mount side for an ejector on +z (photo chirality), so V0.2 puts the
   sleeve above it (from 60 mm) and cuts a window through the mount wall to read it off the robot.
3. **Motor: YPF 20K-2AB-12.7** (CAD-new). 20 × 20 × 37.35 mm, closed rear, pilot Ø 16 × 1.5,
   boss Ø 13, anti-rotation housing Ø 9 to 14.8 mm below the face, shaft Ø 3.5 with M2.5 × 5 end
   (retracted 15.8 mm below the face), M2 on 15.4 mm, connector on one side 29.7–34.7 mm from the
   face. 1 mm lead → 0.5 mm yoke travel per robot-mm, ~4× the Joy-IT's force. No rear screw: the
   whole stack is 43 mm shorter than with the Joy-IT.
4. **One frame for both yokes**: `yoke_total` = 28 mm from the shaft shoulder to the pad face
   leaves room for the spring pad; the rigid yoke is a post of the same length.
5. **Pinout confirmed** in `pipette-main.sch` and the motor board (left mount JP4 pin 3 = GND,
   right mount JP7 pin 3 = 5 V; motor JST 1A/1B/2B/2A; end-stop D2F-L COM → MIN, NC → GND).

## How it works

1. **Identity (I²C).** A 24C02 (or an ESP32-S3 emulating it, option B) answers at 0x50/0x51
   with the `opentron` signature and model `p20_single_v2.2`; the OT-2 treats the tool as a P20
   Single-Channel GEN2.
2. **Plunger.** The robot's plunger driver moves the YPF actuator coaxially above the button.
   Travel u below the robot's 'top' (real mm): pad touches 0.5, first stop 5.5, blow-out 8.5;
   usable 11.2 (12.7 stroke − 1.2 above 'top' − 0.3 at the far end).
3. **Home switch** on the wall face; the yoke's tab lifts its lever 0.6 mm above 'top'
   (0.6 mm before the retracted hard stop).
4. **Robot positions** (robot-mm, written by `configure_pipette.py`): top 17.8, bottom 6.8,
   blowout 0.8, dropTip 0.8 (= blowout: no ejection with this stroke). Run current 0.5 A.
5. **Volumes** in real µl through the wrapper: 0.25 → `aspirate(0.709)`, 1.0 → 2.852,
   2.5 → 7.442; real 1 µl/s → `flow_rate` 3.05.
6. **Tips**: standard 10 µl tips, picked up by pressing the cone into the rack; removed in the
   printed stripper (slot 4.4 mm: wider than the cone where it passes the plate, ~3.4-3.8 mm,
   narrower than a 10 µl tip rim, ~5-5.6 mm).

## Tip ejection: Opentrons over-travel or a second motor?

- **What Opentrons does (GEN2 singles):** no reverse move. In `drop_tip()` the plunger keeps going
  *down* past blow-out to its `dropTip` position (P20 GEN2: −27 vs blow-out −13) and pushes the
  ejector; it is purely mechanical. The robot applies its own `dropTipCurrent` and `dropTipSpeed`
  there; both are overridable per pipette.
- **The same on our tool (eject variant, modelled):** the yoke carries a spring pad on the
  plunger button and an arm with an M3 screw 9.0 mm above the ejector button. Down to blow-out
  the pad is rigid (spring preload > blow-out force); past blow-out the button stops, the spring
  gives way and the screw pushes the ejector 7 mm. No board, no cable, no extra power;
  `drop_tip()` works anywhere (trash, rack). Needs 16.5 mm of travel below 'top' → **an
  actuator stroke ≥ 18 mm**, and about 20–35 N at the end (spring 10-15 N + ejection 10-20 N,
  MEASURE) — fine for a 1 mm lead at low speed with `dropTipCurrent` raised for the short move.
- **With the 12.7 mm stroke** pipetting fits (8.5 of 11.2 mm), ejection does not. Same motor
  with a 19.05 or 25.4 mm stroke (ask YPF; 11.7 mm headroom is left above the motor if the body
  grows) → eject variant; until then the stripper.
- **Second motor:** no board has to interpret steps — a second switch below blow-out (only
  `drop_tip()` goes there) tells a XIAO to run the ejector motor. But it needs its own power (pin 6
  is meant for the EEPROM; use USB from the robot), a cable, firmware, and room for a second
  actuator 15 mm off-axis next to the main motor inside 32 × 37 mm. Kept as the fallback.

**Recommendation: over-travel, with a longer-stroke YPF 20K.**

## Decisions and why (kept from V0.1)

- **Native ribbon instead of USB for the plunger**: I²C only identifies; motion is the robot's
  own stepper axis. Using those wires gives the standard API, App and calibration.
- **Emulate the P20 GEN2**: lowest nozzle of the OT-2 singles, closest volume range and tips.
- **Everything above the shroud line inside the GEN2 outline** (32.34 mm from the mount face,
  37.5 mm along the bar): the neighbouring pipette is 1.7 mm away. Hence the actuator coaxially
  above the button, and the finger hook cut off (it sticks out 34 mm behind the axis; the
  GEN2 is 44.5 mm deep in total).
- **Insert pattern copied from the gripper adapter** (same mount face x = −9.5 and insert axes;
  checked against the adapter STEP).
- **Pipette inserted from below**, top face against the ledge (takes the tip pick-up force), held
  by 2 M3 grub screws through the mount wall onto the −x flat (take the plunger force; set before
  mounting). A slot in the sleeve lets the ejector button pass.
- **Nozzle 5 mm below the P20's** (`nozzle_offset`) to keep 11.7 mm between the motor and the
  estimated top datum; pipette-offset calibration absorbs it.

## Checks (`pipette_tool.py`: 43 ok, 2 info; `build_inventor.py`: flex test, constraints, interference; `check_native.py`; PCB DRC)

- Motor rear 11.7 mm below the estimated top datum; cone end 5.0 mm below the P20 nozzle.
- Pipetting stroke fits (blow-out 8.5 of 11.2 mm); ejection needs a stroke ≥ 18 mm (info).
- Frame, both yokes and the actuator with its connector housing inside the envelope.
- Frame clear of the pipette at rest and with the button at blow-out and the ejector pressed;
  clear of actuator, switch, identity board.
- Both yokes clear of the frame at −0.6 / 0 / 8.5 / 11.2 mm and of the switch; tab lifts the
  lever; yokes clear of the pipette at 'top'; eject arm clear of the button at blow-out; spring
  pad clear of the frame.
- Insert axes, mount face and adapter top identical to `OT2-Gripper V1-1 - OT2-interface adapter`.
- Frame and the pipette above the shroud line (y = −105.3) inside the GEN2 outline.
- Frame, yokes and pad are one solid each (by construction since the ledge relief).
- The Inventor parameter table reproduces `stack()` (all 29 derived heights).
- `opentrons_simulate` 8.8.2 runs the example protocol including the stripper labware.
- Inventor (`build_inventor.py`):
  - **Parts:** all 8 healthy, 0 non-parametric dimensions; the flex test passes.
  - **Constraints:** 30 (main) and 33 (eject variant), each checked to hold its part at the
    designed pose. After reopening from disk: no unhealthy constraint and no missing reference.
  - **Interference at travel 0:** 0 mm³ between the frame and the pipette envelope, actuator,
    switch, identity board and OT-2 mount bar. The yoke(s) and pad are clear of the actuator,
    envelope and frame. The three inserts overlap their 4.1 mm holes by 12.6 mm³ each (knurl,
    by design).
  - **`travel` sweep:** at −0.6 mm the tab enters the switch lever (9.6 mm³, it trips). At 8.5
    and 11.2 mm the yokes and pad stay clear of the frame, switch and actuator, and the yoke
    moved exactly `travel`.
  - **References:** the mount bar reference model, placed with its screw holes on the insert
    axes, lands on the gripper adapter's outline (x −27.5…−9.5, y −53…123.65, z ±18.7). The
    CAD-new YPF part draws its shaft fully retracted, 1.2 mm above the robot's 'top'.
- `check_native.py`: all native parts match the CadQuery parts (see above).
- V0.5: the tool board, with its parts from KiCad's STEP, stays inside the envelope. Its header
  face is 0.55 mm inside the mount face. It clears the frame (0 mm³), the pipette at rest and at
  blow-out, the switch, the actuator, and the rigid yoke from −0.6 to 11.2 mm. The eject arm
  collides (202 mm³, info).
  Inventor confirms the same pairs (0 mm³; eject arm 201.8 mm³, classified).
- PCB: DRC 0 errors, 0 unconnected (194 segments, 40 vias, 0.2 mm clearance, 0.4 mm to the edge).

## Open points (MEASURE before printing)

V0.5 first:
- **A. On the robot, left mount:**
  - centre of the ribbon plug from the top screws (22.5 up, 9.3 toward the front assumed);
  - how far the plug and its holder stand out of the mount face;
  - extent and depth of the top block (relief assumed y 84–128, from z −5 forward, 5.5 deep);
  - which way the plug's key faces. The board's header key faces the pipette axis (−z).
- **B. Colleague's V0.4:** STEP export or Inventor 2027 (see above), then merge the bearing
  recess, the widened frame and the switch position.
- **C. The plunger bench test** (`ot2/plunger_bench.py`): step size, highest clean speed, switch
  release. Then set the robot's B (or C) max speed.
- **D. PCB:** check the THT connector footprints against the JLCPCB clone parts. Solder the
  XIAO on its pins, or cut them to ≤ 1.5 mm; there is 1.8 mm of room behind the board.

Earlier open points:

1. **Robot:** with a GEN2 mounted, the height from the axis of the two top mount screws to the
   pipette's top face (V0.2 assumes 122 mm: `datum_y − 46`), and where its socket sits on the
   mount face (V0.2 header 30 mm above those screws, on the +z side). Five minutes with calipers.
2. **Pipette:** first-stop travel at 2.5 µl and blow-out travel (5 + 3 assumed), button forces at
   both stops, ejector button position/travel/force (7 mm assumed), knob Ø/height, tip cone Ø
   where a 10 µl tip seals, the 10 µl tip's rim Ø and length.
3. **Motor:** coil resistance / rated current; whether YPF has the 20K-2AB with 19.05 or 25.4 mm
   stroke and its length.
4. **ESP32-S3 option:** bench test with the writer sketch, then on the robot. Check that pin 6
   holds 5 V with the XIAO attached. Decide where the XIAO sits (wall pocket or above the
   motor; not in the CAD yet).
5. **Finger hook** must be cut off (still the case).
6. Gravimetric calibration of the real ↔ commanded volume map (10 dispenses at 0.25/0.5/1/2.5 µl).

## Next steps

- After the measurements: set the parameters, rebuild, print frame (PETG/ASA, upright, 205 mm)
  and yoke, test on the robot with the rigid yoke and the stripper.
- With a ≥ 19 mm stroke actuator: print the eject variant, choose the spring (preload above the
  blow-out force, ≥ 8 mm travel), set `EJECT_BY_PLUNGER = True` in `ot2/onilab_tool.py`.
- Measured values go into `P` (then `pipette_tool.py` and `build_inventor.py`), or straight into
  the parameter part in Inventor for a quick look. CAD-new names/numbers if the tool moves into
  CAD-new.
