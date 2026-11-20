"""Plunger bench test ON THE OT-2 (Jupyter terminal, http://<robot-ip>:48888 -> New -> Terminal).

Finds out why homing fails: endstop logic, step angle / lead, the speed at which the YPF
actuator starts to lose steps, and whether the switch releases after the firmware's back-off.
Talks G-code straight to the motor controller, so the robot server must be stopped first:

    systemctl stop opentrons-robot-server
    python3 plunger_bench.py --axis B            # B = left mount, C = right mount
    python3 plunger_bench.py --axis B --set-max-speed 10     # robot-wide cap, persists (see below)
    systemctl start opentrons-robot-server

Facts this relies on (Opentrons v8.8.2 + SmoothiewareOT 8414642, the firmware the robot flashes):
G28.2 homes up at 25 robot-mm/s (2500 full steps/s for us) until MIN reads high (switch open),
backs off 2, re-approaches at 2 mm/s, sets B = 20 at the trigger and backs off to B = 18. Limit
switches stay armed during moves ("ALARM: Hard limit +B / -B"); "ALARM: Homing fail" = no
trigger within M365.1 B60. 3200 microsteps (1/32) per robot-mm = 100 full steps per robot-mm,
0.5 mm of yoke travel per robot-mm with the YPF 20K-2AB (1.8 deg, 1 mm lead).
NOT yet run on a robot - read the output line by line; M999 clears an alarm.
"""
import argparse
import glob
import sys
import time

import serial


def open_port():
    ports = sorted(glob.glob("/dev/ttyAMA*")) or sorted(glob.glob("/dev/serial*"))
    if not ports:
        sys.exit("no /dev/ttyAMA* port - is this the OT-2?")
    return serial.Serial(ports[0], 115200, timeout=1)


def cmd(s, line, wait=30.0):
    """Send one line, return the replies up to 'ok' (or an alarm/error)."""
    s.reset_input_buffer()
    s.write((line + "\r\n").encode())
    out, t0 = [], time.time()
    while time.time() - t0 < wait:
        r = s.readline().decode(errors="replace").strip()
        if not r:
            continue
        out.append(r)
        if r.startswith("ok") or "ALARM" in r or r.startswith("error"):
            break
    print(f">> {line:<28} {' | '.join(out)}")
    return " ".join(out)


def ask(msg):
    input(f"\n{msg}  [Enter] ")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", default="B", choices=["B", "C"])
    ap.add_argument("--current", type=float, default=0.5, help="run current, A (YPF rating 0.5)")
    ap.add_argument("--set-max-speed", type=float, help="write a robot-wide max speed for the axis and exit")
    a = ap.parse_args()
    X = a.axis
    if a.set_max_speed:
        from opentrons.config import robot_configs
        rc = robot_configs.load_ot2()
        rc.default_max_speed[X] = a.set_max_speed      # also caps the G28.2 fast phase
        robot_configs.save_robot_settings(rc)
        print("saved; now", robot_configs.load_ot2().default_max_speed, "- restart the robot server")
        return
    s = open_port()
    cmd(s, "M999")
    cmd(s, "version")
    cmd(s, f"M92 {X}3200")
    cmd(s, f"M365.1 {X}60")
    cmd(s, f"M204 S10000 {X}200")
    cmd(s, f"M907 {X}{a.current}")
    cmd(s, "G90")
    # 1. endstop logic
    cmd(s, "M119")
    ask(f"Above: the {X} endstop should read 0 now. Press the switch lever by hand and keep it pressed.")
    cmd(s, "M119")
    ask("It should read 1 while pressed. Release it. If it is the other way round, the switch is wired"
        " to NO instead of NC (or COM/NC swapped) - stop here.")
    # 2. one slow home
    cmd(s, f"M203.1 {X}10")
    r = cmd(s, f"G28.2 {X}", 60)
    cmd(s, "M114.2")
    cmd(s, "M119")
    if "ALARM" in r:
        print("homing failed even at 10 robot-mm/s: check the motor wiring (coil pairs 7/8 and 9/10), the"
              " direction (yoke must move UP), and that the tab reaches the lever before the actuator's top stop")
        cmd(s, "M999")
        return
    # 3. lead / step angle
    ask("Mark the yoke position (calipers from the shelf to the yoke top).")
    cmd(s, f"G0 {X}16 F120")
    cmd(s, "M400")
    ask("Measure again: the yoke must have moved DOWN 1.0 mm (from B18 to B16). 4.2 mm would mean a"
        " 7.5 deg motor, 2 mm a 2 mm lead.")
    # 4. homing speed ladder from the low end of the stroke
    for v in (10, 15, 20, 25):
        cmd(s, f"M203.1 {X}{v}")
        cmd(s, f"G0 {X}1.8 F600")
        r = cmd(s, f"G28.2 {X}", 60)
        pos = cmd(s, "M114.2")
        print(f"   homing at {v} robot-mm/s = {v * 100} full steps/s: {'FAILED' if 'ALARM' in r else 'ok'}  {pos}")
        if "ALARM" in r:
            cmd(s, "M999")
            break
    # 5. move cycles at a moderate speed, then creep near the switch (lost steps show as Hard limit +X)
    cmd(s, f"M203.1 {X}10")
    for k in range(20):
        r1 = cmd(s, f"G0 {X}4 F600")
        r2 = cmd(s, f"G0 {X}17 F600")
        if "ALARM" in r1 + r2:
            print(f"   alarm in cycle {k}")
            cmd(s, "M999")
            break
    for k in range(10):
        r = cmd(s, f"G0 {X}19.2 F60") + cmd(s, f"G0 {X}18.6 F60")
        if "ALARM" in r:
            print("   hard limit near the switch: lost steps, or the switch trips below 20 (chatter)")
            cmd(s, "M999")
            break
    cmd(s, f"G28.2 {X}", 60)
    cmd(s, f"M18 {X}")
    print("\ndone. Fastest clean homing speed above -> use --set-max-speed with a margin (e.g. 10-15).")


if __name__ == "__main__":
    main()
