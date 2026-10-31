# Identity board: makes the OT-2 see the ONiLAB tool as a P20 Single-Channel GEN2

No microcontroller is needed for the pipette itself. The OT-2 pipette cable carries everything
the robot needs: I²C to an EEPROM (who is attached), the plunger home switch, and the 4 stepper
wires that the robot's own driver (Allegro A5984, 1/32 microstep, ~36 V) drives directly.

## OT-2 pipette connector (2×5, 2.54 mm, shrouded; 3M 30310-6002HB on Opentrons pipettes)

From Opentrons' published EAGLE files (github.com/Opentrons/ot2, "PCB files/pipette-main",
motor board JP4 = left, JP7 = right). **Verify pin 1 on your robot before connecting** (see below).

| Pin | Signal | On the tool |
|---|---|---|
| 1 | SCL (5 V, 10 k pull-up on the robot) | 24C02 pin 6 |
| 2 | SDA (5 V) | 24C02 pin 5 |
| 3 | ADDRESS: GND on the left mount, 5 V on the right | 24C02 pin 1 (A0) → 0x50 left / 0x51 right |
| 4 | MIN, plunger home switch | switch COM |
| 5 | GND | 24C02 pins 2, 3, 4, 7 (A1, A2, GND, WP); switch NC |
| 6 | +5 V | 24C02 pin 8 (VCC) + 100 nF to GND |
| 7 / 8 | MOTOR_2A / 2B | stepper coil B |
| 9 / 10 | MOTOR_1A / 1B | stepper coil A |

Switch: COM → MIN, NC → GND. At rest MIN is held low; pressed (yoke at the top) it opens and the
robot's pull-up reads "home". This matches the Omron D2F-L wiring inside Opentrons pipettes.

## Parts

| Qty | Part | Note |
|---|---|---|
| 1 | 24C02 EEPROM, SOIC-8 or DIP-8 (AT24C02, M24C02, ...) | 8-byte pages; any 2 kbit 24xx works |
| 1 | 2×5 shrouded header 2.54 mm | sits in the window of the mount plate (or use a short ribbon) |
| 1 | 100 nF ceramic | |
| 1 | Omron D2F-L or similar 12.8 × 5.8 × 6.5 mm lever micro switch | 2 × M2 self-tapping into the frame |
| 1 | JST-XH 4-pin + 2-pin | motor and switch |
| 1 | perfboard ~30 × 20 mm | 4 × M2.5 self-tapping into the frame (holes 12 × 28 mm) |

## Programming the EEPROM

Use `../firmware/ot2_eeprom_writer/ot2_eeprom_writer.ino` on a Seeed XIAO (RP2040 or SAMD21) or any
Arduino, off the robot, with the EEPROM powered from 3.3 V and A0 to GND (wiring in the sketch).
It writes the layout SmoothiewareOT expects: `opentron` signature at 0x00, ID at 0x30, model
`p20_single_v2.2` at 0x60. Give every tool its own ID: the robot stores calibration and the
plunger settings under it.

## Checks on the robot before plugging the tool in

1. Unplug the ribbon from a real pipette. Measure 5 V between pins 6 and 5 (red stripe = pin 1).
2. Pin 3 to pin 5: 0 V on the left mount, 5 V on the right mount.
3. Tool side: each motor coil pair reads ~22 Ω (Joy-IT NEMA08-04LA); no short between coils.
4. Plug in, open the Opentrons App → the mount should show **P20 Single-Channel GEN2**.
5. Home: the yoke must move **up** toward the switch. If it moves down, swap one coil pair (7 ↔ 8).

## Why not the XIAO on I²C or USB for the plunger

- The I²C bus only carries the identity. The robot reads it while scanning instruments (10 kHz,
  8-byte chunks, both plunger motors switched off during each read). It never sends motion over it.
- The plunger axis is a normal stepper axis of the robot's motion controller. Driving our motor from
  those 4 wires gives native `aspirate / dispense / blow_out / home`, the App, Protocol Designer and
  calibration without any extra firmware.
- A XIAO is useful only for **extra** actuators (e.g. a tip-ejector motor): either as a USB device
  (plain serial from the protocol, or imitating a Magnetic Module: VID:PID 04d8:ee90, `G28.2`, `G0 Z`,
  `M114.2`, `M115`), or triggered by a second switch at a "drop" position of the plunger.
