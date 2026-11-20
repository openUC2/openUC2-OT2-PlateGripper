"""Small two-layer grid router for the pipette board (step 2 of build_pcb.py).

    uv run --with numpy --with scipy python route.py pads.json routes.json

Reads the pads, board outline, holes and net classes that build_pcb.py exported from KiCad,
routes every net as a minimum spanning tree of A* connections on a 0.2 mm grid (8 directions,
vias between F.Cu and B.Cu), keeps clearance with distance transforms of the copper already
placed, and writes tracks and vias for build_pcb.py to add back. Nets are tried in several
orders until all connect. No pour: GND is routed like any other net, the pours come after.
"""
import heapq
import json
import math
import random
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

G = 0.2                     # grid, mm
MAX_EXPAND = 250_000         # A* gives up after this many expansions
LAYERS = ("F.Cu", "B.Cu")
DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


class Grid:
    def __init__(self, spec):
        self.spec = spec
        xs = [p[0] for p in spec["outline"]]
        ys = [p[1] for p in spec["outline"]]
        self.x0, self.y0 = min(xs) - 1, min(ys) - 1
        self.nx = int((max(xs) + 1 - self.x0) / G) + 1
        self.ny = int((max(ys) + 1 - self.y0) / G) + 1
        self.cl = spec["rules"]["clearance"]
        self.edge = spec["rules"]["edge_clearance"]
        X, Y = np.meshgrid(self.x0 + np.arange(self.nx) * G, self.y0 + np.arange(self.ny) * G, indexing="ij")
        self.X, self.Y = X, Y
        inside = points_in_poly(X, Y, spec["outline"])
        dist_in = distance_transform_edt(inside) * G
        self.board_ok = inside & (dist_in > self.edge)          # track copper may not come closer to the edge
        self.holes = np.zeros_like(inside)
        for h in spec["holes"]:
            self.holes |= (X - h["x"]) ** 2 + (Y - h["y"]) ** 2 <= (h["d"] / 2) ** 2
        # copper owner per layer: -1 free, else net index
        self.own = {L: np.full((self.nx, self.ny), -1, np.int32) for L in LAYERS}
        self.nets = sorted({p["net"] for p in spec["pads"] if p["net"]})
        self.nid = {n: i for i, n in enumerate(self.nets)}
        for p in spec["pads"]:
            if p["net"] is None:
                for L in p["layers"]:
                    self.own[L][self.pad_mask(p)] = 10_000     # unconnected pad: blocks everyone
                continue
            for L in p["layers"]:
                self.own[L][self.pad_mask(p)] = self.nid[p["net"]]

    def cell(self, x, y):
        return int(round((x - self.x0) / G)), int(round((y - self.y0) / G))

    def xy(self, i, j):
        return self.x0 + i * G, self.y0 + j * G

    def pad_mask(self, p):
        X, Y = self.X - p["x"], self.Y - p["y"]
        a = math.radians(-p["rot"])
        u = X * math.cos(a) - Y * math.sin(a)
        v = X * math.sin(a) + Y * math.cos(a)
        hx, hy = p["sx"] / 2, p["sy"] / 2
        if p["shape"] == "circle":
            return u ** 2 + v ** 2 <= hx ** 2
        if p["shape"] == "oval":
            r = min(hx, hy)
            ex, ey = max(hx - r, 0), max(hy - r, 0)
            du, dv = np.maximum(np.abs(u) - ex, 0), np.maximum(np.abs(v) - ey, 0)
            return du ** 2 + dv ** 2 <= r ** 2
        return (np.abs(u) <= hx) & (np.abs(v) <= hy)

    def blocked(self, net, half):
        """Cells where the CENTRE of a track of half-width `half` of `net` may not go, per layer."""
        out = {}
        k = self.nid[net]
        for L in LAYERS:
            other = (self.own[L] >= 0) & (self.own[L] != k)
            d = distance_transform_edt(~other) * G
            out[L] = (d < half + self.cl + 1.0 * G) | ~self.board_ok | self.holes_blocked(half)
        return out

    def holes_blocked(self, half):
        d = distance_transform_edt(~self.holes) * G
        return d < half + self.cl + 0.3

    def paint(self, net, layer, pts, half):
        k = self.nid[net]
        for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
            n = max(2, int(math.hypot(xb - xa, yb - ya) / (G / 2)) + 1)
            for t in np.linspace(0, 1, n):
                x, y = xa + (xb - xa) * t, ya + (yb - ya) * t
                i0, j0 = self.cell(x, y)
                r = int(math.ceil(half / G)) + 1
                sl = (slice(max(i0 - r, 0), i0 + r + 1), slice(max(j0 - r, 0), j0 + r + 1))
                m = (self.X[sl] - x) ** 2 + (self.Y[sl] - y) ** 2 <= half ** 2
                self.own[layer][sl][m] = k

    def paint_via(self, net, x, y, d):
        for L in LAYERS:
            self.paint(net, L, [(x, y), (x, y)], d / 2)


def points_in_poly(X, Y, poly):
    inside = np.zeros(X.shape, bool)
    n = len(poly)
    for i in range(n):
        (xa, ya), (xb, yb) = poly[i], poly[(i + 1) % n]
        cond = ((ya > Y) != (yb > Y)) & (X < (xb - xa) * (Y - ya) / (yb - ya + 1e-12) + xa)
        inside ^= cond
    return inside


def astar(grid, net, half, via_d, starts, goals, blocked, via_cost=6.0):
    """starts/goals: sets of (layer_index, i, j). Returns [(layer, i, j), ...] or None."""
    via_ok = ~(blocked["F.Cu"] | blocked["B.Cu"])
    # a via needs room for its pad on both layers
    rv = via_d / 2 - half
    if rv > 0:
        k = grid.nid[net]
        for L in LAYERS:
            other = (grid.own[L] >= 0) & (grid.own[L] != k)
            d = distance_transform_edt(~other) * G
            via_ok &= d >= via_d / 2 + grid.cl + 1.0 * G
    gi = np.array([g[1] for g in goals])
    gj = np.array([g[2] for g in goals])
    gx, gy = gi.mean(), gj.mean()

    def h(i, j):
        return math.hypot(i - gx, j - gy) * 0.9

    openq = []
    best = {}
    prev = {}
    for s in starts:
        best[s] = 0.0
        heapq.heappush(openq, (h(s[1], s[2]), 0.0, s, None))
    goalset = set(goals)
    expanded = 0
    while openq:
        expanded += 1
        if expanded > MAX_EXPAND:
            return None
        f, g, node, d0 = heapq.heappop(openq)
        if g > best.get(node, 1e18) + 1e-9:
            continue
        if node in goalset:
            path = [node]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            return path[::-1]
        L, i, j = node
        bl = blocked[LAYERS[L]]
        for di, dj, c in DIRS:
            ni, nj = i + di, j + dj
            if not (0 <= ni < grid.nx and 0 <= nj < grid.ny) or bl[ni, nj]:
                continue
            turn = 0.2 if d0 is not None and d0 != (di, dj) else 0.0
            nn = (L, ni, nj)
            ng = g + c + turn + (0.3 if L == 1 else 0.0)      # mild preference for the top layer
            if ng < best.get(nn, 1e18):
                best[nn] = ng
                prev[nn] = node
                heapq.heappush(openq, (ng + h(ni, nj), ng, nn, (di, dj)))
        if via_ok[i, j]:
            nn = (1 - L, i, j)
            ng = g + via_cost
            if ng < best.get(nn, 1e18) and not blocked[LAYERS[1 - L]][i, j]:
                best[nn] = ng
                prev[nn] = node
                heapq.heappush(openq, (ng + h(i, j), ng, nn, None))
    return None


def simplify(cells):
    """Split a cell path into per-layer polylines (corner points only) and via positions."""
    runs, vias = [], []
    cur = [cells[0]]
    for a, b in zip(cells[:-1], cells[1:]):
        if a[0] != b[0]:
            runs.append(cur)
            vias.append((a[1], a[2]))
            cur = [b]
        else:
            cur.append(b)
    runs.append(cur)
    out = []
    for r in runs:
        pts = [r[0]]
        for k in range(1, len(r) - 1):
            d1 = (r[k][1] - r[k - 1][1], r[k][2] - r[k - 1][2])
            d2 = (r[k + 1][1] - r[k][1], r[k + 1][2] - r[k][2])
            if d1 != d2:
                pts.append(r[k])
        if len(r) > 1:
            pts.append(r[-1])
        out.append((r[0][0], pts))
    return out, vias


def pad_cells(grid, p):
    m = grid.pad_mask(p)
    ii, jj = np.nonzero(m)
    out = set()
    for L in p["layers"]:
        li = LAYERS.index(L)
        out |= {(li, int(i), int(j)) for i, j in zip(ii, jj)}
    return out


def route_all(spec, order, seed=0):
    grid = Grid(spec)
    classes = spec["classes"]
    tracks, vias, failed = [], [], []
    pads_by_net = {}
    for p in spec["pads"]:
        if p["net"]:
            pads_by_net.setdefault(p["net"], []).append(p)
    for net in order:
        pads = pads_by_net.get(net, [])
        if len(pads) < 2:
            continue
        cls = classes.get(net, classes["default"])
        half, via_d = cls["width"] / 2, cls["via"]
        # Prim over pad centres
        done = [pads[0]]
        todo = pads[1:]
        tree = pad_cells(grid, pads[0])
        while todo:
            nxt = min(todo, key=lambda q: min(math.hypot(q["x"] - d["x"], q["y"] - d["y"]) for d in done))
            blocked = grid.blocked(net, half)
            goals = pad_cells(grid, nxt)
            starts = {c for c in tree if not blocked[LAYERS[c[0]]][c[1], c[2]]} or tree
            goals_ok = {c for c in goals if not blocked[LAYERS[c[0]]][c[1], c[2]]} or goals
            for c in starts | goals_ok:                      # pads may sit inside their own clearance halo
                blocked[LAYERS[c[0]]][c[1], c[2]] = False
            path = astar(grid, net, half, via_d, starts, goals_ok, blocked)
            if path is None:
                failed.append((net, nxt["ref"], nxt["pad"]))
                todo.remove(nxt)
                continue
            runs, vs = simplify(path)
            for li, pts in runs:
                xy = [grid.xy(i, j) for _, i, j in pts]
                if len(xy) >= 2:
                    for a, b in zip(xy[:-1], xy[1:]):
                        tracks.append({"net": net, "layer": LAYERS[li], "x1": a[0], "y1": a[1], "x2": b[0], "y2": b[1],
                                       "w": cls["width"]})
                    grid.paint(net, LAYERS[li], xy, half)
                tree |= {(li, i, j) for _, i, j in pts}
            for i, j in vs:
                x, y = grid.xy(i, j)
                vias.append({"net": net, "x": x, "y": y, "d": via_d, "drill": cls["drill"]})
                grid.paint_via(net, x, y, via_d)
                tree |= {(0, i, j), (1, i, j)}
            tree |= goals
            done.append(nxt)
            todo.remove(nxt)
    return tracks, vias, failed


def main():
    spec = json.load(open(sys.argv[1]))
    nets = sorted({p["net"] for p in spec["pads"] if p["net"]})
    first = spec.get("route_first", [])
    best = None
    for attempt in range(12):
        rest = [n for n in nets if n not in first]
        random.Random(attempt).shuffle(rest)
        order = first + rest if attempt else first + sorted(rest)
        tracks, vias, failed = route_all(spec, order)
        print(f"attempt {attempt}: {len(tracks)} segments, {len(vias)} vias, failed {failed}", flush=True)
        if best is None or len(failed) < len(best[2]) or (len(failed) == len(best[2]) and len(vias) < len(best[1])):
            best = (tracks, vias, failed, order)
        if not failed:
            break
    tracks, vias, failed, order = best
    json.dump({"tracks": tracks, "vias": vias, "failed": failed, "order": order}, open(sys.argv[2], "w"), indent=1)
    print("written", sys.argv[2], "failed:", failed or "none")


if __name__ == "__main__":
    main()
