"""Write the ONiLAB tool's plunger positions and currents into the OT-2's pipette settings.

The robot stores per-pipette overrides under the EEPROM ID (PATCH /settings/pipettes/{id},
robot-server v8). Run from any PC on the robot's network after the tool shows up as
"P20 Single-Channel GEN2" in the App:

    python configure_pipette.py 192.168.1.42 left            # show what would be written
    python configure_pipette.py 192.168.1.42 left --apply    # write it
    python configure_pipette.py 192.168.1.42 left --reset    # back to P20 defaults
"""
import argparse
import json
import urllib.request

import onilab_tool as tool

HEADERS = {"Opentrons-Version": "3", "Content-Type": "application/json"}


def call(ip, method, path, body=None):
    req = urllib.request.Request(f"http://{ip}:31950{path}", method=method, headers=HEADERS,
                                 data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ip")
    ap.add_argument("mount", choices=["left", "right"])
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--reset", action="store_true")
    a = ap.parse_args()

    pip = call(a.ip, "GET", "/pipettes?refresh=true")[a.mount]
    if pip.get("model") != tool.EMULATED_MODEL:
        raise SystemExit(f"{a.mount} mount reports {pip.get('model')!r}, expected {tool.EMULATED_MODEL!r}")
    pid = pip["id"]
    fields = tool.overrides()
    body = {"fields": {k: (None if a.reset else {"value": v}) for k, v in fields.items()}}
    print(f"pipette {pid} on {a.mount}:")
    for k, v in fields.items():
        print(f"  {k:15s} {'(default)' if a.reset else v}")
    if a.apply or a.reset:
        res = call(a.ip, "PATCH", f"/settings/pipettes/{pid}", body)
        print("robot answered:", json.dumps(res.get("fields", res), indent=1)[:1500])
        print("Home the robot (or restart) so the new plunger settings are loaded.")
    else:
        print("dry run - add --apply to write")


if __name__ == "__main__":
    main()
