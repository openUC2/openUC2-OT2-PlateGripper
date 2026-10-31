"""Shaded views and an axial section of the pipette tool -> ../docs/*.png (matplotlib, no GUI)."""
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


def main():
    p = t.P()
    parts = t.build(p)
    s = t.stack(p)
    adapter = cq.importers.importStep(str(t.REF / "OT2-interface-adapter.stp"))
    items = [(parts["frame"], (0.55, 0.7, 0.88), 1.0),
             (parts["yoke"], (0.45, 0.8, 0.5), 1.0), (t.actuator(p), (0.28, 0.28, 0.3), 1.0),
             (t.switch(p), (0.15, 0.15, 0.15), 1.0), (t.pipette(p), (0.8, 0.8, 0.84), 0.55)]
    env = envelope_edges(p, s["shroud_y"], p.top_y)

    tiles = []
    for az, el, title in [(-35, 15, "inner side"), (215, 15, "mount face"), (55, 20, "front end"), (0, 0, "side")]:
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

    # close-up of the head with the original gripper adapter as a ghost
    fig = plt.figure(figsize=(13, 7))
    for k, (az, el) in enumerate([(-40, 18), (230, 18)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        draw(ax, [(parts["frame"].intersect(t.box(-20, 40, s["shoulder_body"] - 25, 60, -30, 30)), (0.55, 0.7, 0.88), 1.0),
                  (parts["yoke"], (0.45, 0.8, 0.5), 1.0),
                  (t.actuator(p), (0.28, 0.28, 0.3), 1.0), (t.switch(p), (0.15, 0.15, 0.15), 1.0),
                  (adapter, (0.95, 0.4, 0.4), 0.18),
                  (t.pipette(p).intersect(t.box(-20, 40, s["shoulder_body"] - 25, 0, -30, 30)), (0.8, 0.8, 0.84), 0.5)])
        ax.view_init(elev=el, azim=az)
    fig.suptitle("Head: mount plate copied from the gripper adapter (red ghost), Joy-IT actuator above the plunger button, yoke + home switch",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(DOCS / "pipette_tool_head.png", dpi=110, bbox_inches="tight")
    plt.close(fig)

    # axial section (plane z = pipette axis) through everything
    fig, axs = plt.subplots(1, 2, figsize=(11, 13), gridspec_kw={"width_ratios": [1, 1.6]})
    cut = cq.Workplane("XY").box(200, 600, 0.02).translate((0, -100, p.pip_z))
    cols = {"frame": "#8cb3e0", "yoke": "#73cc80", "actuator": "#47474d",
            "switch": "#262626", "pipette": "#c8c8d0"}
    objs = {"frame": parts["frame"], "yoke": parts["yoke"], "actuator": t.actuator(p),
            "switch": t.switch(p), "pipette": t.pipette(p)}
    for ax, (ylo, yhi) in zip(axs, [(-300, 60), (-95, 58)]):
        for name, wp in objs.items():
            try:
                sec = wp.intersect(cut)
                for f in sec.faces().vals():
                    if abs(f.normalAt().z) < 0.9:
                        continue
                    for w, fill in [(f.outerWire(), cols[name])] + [(iw, "white") for iw in f.innerWires()]:
                        pts = np.array([(q.x, q.y) for q in (w.positionAt(i / 300) for i in range(301))])
                        ax.fill(pts[:, 0], pts[:, 1], color=fill, alpha=0.9 if name != "pipette" else 0.6,
                                lw=0.3, ec="k")
            except Exception as e:
                print("section", name, e)
        ax.axhline(s["p20_nozzle"], color="#c0392b", ls="--", lw=0.8)
        ax.text(-23, s["p20_nozzle"] + 2, "P20 GEN2 nozzle", color="#c0392b", fontsize=8, clip_on=True)
        ax.axhline(s["shroud_y"], color="#7f8c8d", ls=":", lw=0.8)
        ax.text(-23, s["shroud_y"] + 2, "shroud bottom (Z home)", color="#7f8c8d", fontsize=8, clip_on=True)
        ax.axvline(p.mount_x, color="#7f8c8d", lw=0.6)
        ax.axvline(p.env_x1, color="#c0392b", lw=0.6, ls="--")
        ax.set_xlim(-25, 40); ax.set_ylim(ylo, yhi)
        ax.set_aspect("equal"); ax.grid(alpha=0.25)
        ax.set_xlabel("x from mount (mm)"); ax.set_ylabel("y (mm, adapter frame)")
    axs[0].set_title("section through the pipette axis")
    axs[1].set_title("detail: actuator, yoke, button")
    fig.tight_layout()
    fig.savefig(DOCS / "pipette_tool_section.png", dpi=110)
    plt.close(fig)
    print("renders written to", DOCS)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
