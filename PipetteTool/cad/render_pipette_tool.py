"""Shaded views and sections of the pipette tool -> ../docs/*.png (matplotlib, no GUI)."""
import os
import sys
from pathlib import Path

import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

import pipette_tool as t

DOCS = Path(__file__).resolve().parent.parent / "docs"
LIGHT = np.array([0.35, 0.8, 0.5]) / np.linalg.norm([0.35, 0.8, 0.5])
C = {"frame": (0.55, 0.7, 0.88), "yoke": (0.45, 0.8, 0.5), "pad": (0.92, 0.72, 0.25), "act": (0.28, 0.28, 0.3),
     "sw": (0.15, 0.15, 0.15), "pcb": (0.1, 0.5, 0.25), "pip": (0.8, 0.8, 0.84)}


def mesh(wp, tol=0.15):
    v, f = [], []
    for s in wp.solids().vals():
        vv, ff = s.tessellate(tol, 0.25)
        off = len(v)
        v += [(q.x, q.y, q.z) for q in vv]
        f += [(a + off, b + off, c + off) for a, b, c in ff]
    return np.array(v), np.array(f)


def draw(ax, items, env=None):
    allv = []
    for wp, rgb, alpha in items:
        v, f = mesh(wp)
        v = v[:, [0, 2, 1]]                      # plot with y (up) on matplotlib's vertical axis
        tri = v[f]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        sh = 0.45 + 0.55 * np.abs(n @ LIGHT)
        fc = np.clip(np.array(rgb)[None, :] * sh[:, None], 0, 1)
        fc = np.hstack([fc, np.full((len(fc), 1), alpha)])
        ax.add_collection3d(Poly3DCollection(tri, facecolors=fc, edgecolor="none"))
        allv.append(v)
    if env is not None:
        env = [[(a[0], a[2], a[1]) for a in seg] for seg in env]
        ax.add_collection3d(Line3DCollection(env, colors="#c0392b", linewidths=0.6, linestyles="--"))
    if env is not None:
        allv.append(np.array([q for seg in env for q in seg]))
    allv = np.vstack(allv)
    lo, hi = allv.min(0) - 2, allv.max(0) + 2
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(tuple(hi - lo), zoom=1.0)
    ax.set_axis_off()


def envelope_edges(p, y0, y1):
    x0, x1, z0, z1 = p.mount_x, p.env_x1, -p.env_z, p.env_z
    c = [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]
    e = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        e += [[(a[0], y0, a[1]), (b[0], y0, b[1])], [(a[0], y1, a[1]), (b[0], y1, b[1])],
              [(a[0], y0, a[1]), (a[0], y1, a[1])]]
    return e


def section(ax, objs, cols, axis, at):
    """Fill the cut of each solid with the plane axis=at ('z' -> (x, y) plot, 'x' -> (z, y) plot)."""
    cut = (cq.Workplane("XY").box(300, 700, 0.02).translate((0, 0, at)) if axis == "z"
           else cq.Workplane("XY").box(0.02, 700, 300).translate((at, 0, 0)))
    for name, wp in objs.items():
        try:
            sec = wp.intersect(cut)
            for f in sec.faces().vals():
                nrm = f.normalAt()
                if abs(nrm.z if axis == "z" else nrm.x) < 0.9:
                    continue
                for w, fill in [(f.outerWire(), cols[name])] + [(iw, "white") for iw in f.innerWires()]:
                    pts = [w.positionAt(i / 300) for i in range(301)]
                    pts = np.array([(q.x, q.y) if axis == "z" else (q.z, q.y) for q in pts])
                    ax.fill(pts[:, 0], pts[:, 1], color=fill, alpha=0.6 if name == "pipette" else 0.9, lw=0.3, ec="k")
        except Exception as e:
            print("section", name, e)


def levels(ax, p, s, x_txt):
    for y, txt, c in [(s["p20_nozzle"], "P20 GEN2 nozzle", "#c0392b"), (s["shroud_y"], "shroud bottom (Z home)", "#7f8c8d"),
                      (p.adapter_top, "gripper adapter top", "#7f8c8d"), (p.datum_y, "GEN2 top datum (estimate)", "#c0392b"),
                      (p.bar_top, "mount bar top", "#7f8c8d")]:
        ax.axhline(y, color=c, ls="--" if c == "#c0392b" else ":", lw=0.8)
        ax.text(x_txt, y + 1.5, txt, color=c, fontsize=7.5, clip_on=True)


def main():
    p = t.P()
    parts = t.build(p)
    s = t.stack(p)
    adapter = cq.importers.importStep(str(t.REF / "OT2-interface-adapter.stp"))
    act, sw, pcb, pip = t.actuator(p), t.switch(p), t.tool_board(p), t.pipette(p)
    items = [(parts["frame"], C["frame"], 1.0), (parts["yoke"], C["yoke"], 1.0), (act, C["act"], 1.0),
             (sw, C["sw"], 1.0), (pcb, C["pcb"], 1.0), (pip, C["pip"], 0.55)]
    env = envelope_edges(p, s["shroud_y"], p.datum_y)

    tiles = []
    for az, el, title in [(-35, 15, "inner side"), (215, 15, "mount face"), (55, 20, "front (ejector side)"), (0, 0, "side")]:
        fig = plt.figure(figsize=(3.2, 11))
        ax = fig.add_axes([-0.6, -0.02, 2.2, 1.0], projection="3d")
        draw(ax, items, env)
        ax.view_init(elev=el, azim=az)
        fig.text(0.5, 0.975, title, ha="center", fontsize=11)
        tmp = DOCS / f"_tile_{len(tiles)}.png"
        fig.savefig(tmp, dpi=110)
        plt.close(fig)
        tiles.append(plt.imread(tmp))
        tmp.unlink()
    plt.imsave(DOCS / "pipette_tool_views.png", np.hstack(tiles))

    # head close-up: eject variant (arm + spring pad) and the gripper adapter as a ghost
    crop = t.box(-20, 40, s["body_top"] - 25, 200, -30, 30)
    fig = plt.figure(figsize=(13, 7.5))
    for k, (az, el) in enumerate([(-30, 12), (-105, 12)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        draw(ax, [(parts["frame"].intersect(crop), C["frame"], 1.0), (parts["yoke_eject"], C["yoke"], 1.0),
                  (parts["pad"], C["pad"], 1.0), (act, C["act"], 1.0), (sw, C["sw"], 1.0), (pcb, C["pcb"], 1.0),
                  (adapter, (0.95, 0.4, 0.4), 0.15), (pip.intersect(crop), C["pip"], 0.5)])
        ax.view_init(elev=el, azim=az)
    fig.suptitle("Head, eject variant: YPF 20K actuator above the plunger button, yoke with spring pad (orange) and "
                 "ejector arm, switch, identity board; gripper adapter as red ghost", fontsize=10)
    fig.tight_layout()
    fig.savefig(DOCS / "pipette_tool_head.png", dpi=110, bbox_inches="tight")
    plt.close(fig)

    # sections: full height through the axis (plane z = axis), head detail through the arm (plane x = axis)
    cols = {"frame": "#8cb3e0", "yoke": "#73cc80", "pad": "#ebb840", "actuator": "#47474d", "switch": "#262626",
            "pcb": "#1a803f", "pipette": "#c8c8d0"}
    fig, axs = plt.subplots(1, 2, figsize=(12, 13), gridspec_kw={"width_ratios": [1, 1.5]})
    objs = {"frame": parts["frame"], "yoke": parts["yoke"], "actuator": act, "switch": sw, "pcb": pcb, "pipette": pip}
    section(axs[0], objs, cols, "z", p.pip_z)
    levels(axs[0], p, s, -24)
    axs[0].axvline(p.mount_x, color="#7f8c8d", lw=0.6)
    axs[0].axvline(p.env_x1, color="#c0392b", lw=0.6, ls="--")
    axs[0].set_xlim(-25, 40); axs[0].set_ylim(s["cone_end"] - 8, p.datum_y + 6)
    axs[0].set_xlabel("x from the mount (mm)"); axs[0].set_title("section through the axis (rigid yoke, at 'top')")
    objs2 = {"frame": parts["frame"], "yoke": parts["yoke_eject"], "pad": parts["pad"], "actuator": act, "pipette": pip}
    section(axs[1], objs2, cols, "x", p.pip_x)
    levels(axs[1], p, s, -19)
    for u, lab in [(0, "top"), (s["u_bottom"], "first stop"), (s["u_blowout"], "blow-out"), (s["u_drop"], "eject end")]:
        axs[1].annotate(f"{lab}: {u:.1f} mm", (p.env_z, s["top"] - u), fontsize=7.5, color="#2c3e50",
                        xytext=(p.env_z + 3, s["top"] - u), arrowprops=dict(arrowstyle="-", lw=0.5))
    axs[1].axvline(-p.env_z, color="#c0392b", lw=0.6, ls="--")
    axs[1].axvline(p.env_z, color="#c0392b", lw=0.6, ls="--")
    axs[1].set_xlim(-22, 40); axs[1].set_ylim(s["body_top"] - 22, p.datum_y + 6)
    axs[1].set_xlabel("z (mm, ejector side +)")
    axs[1].set_title(f"head, eject variant: arm screw {s['u_eject0']:.1f} mm above the ejector button", fontsize=10)
    for ax in axs:
        ax.set_aspect("equal"); ax.grid(alpha=0.25); ax.set_ylabel("y (mm, adapter frame)")
    fig.tight_layout()
    fig.savefig(DOCS / "pipette_tool_section.png", dpi=110)
    plt.close(fig)
    print("renders written to", DOCS)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
