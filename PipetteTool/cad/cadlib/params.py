"""Chip parameters: one table that drives both the Python checks and the Inventor model.

A chip script declares
  - base parameters  `P(name, value, unit, comment)`  - the knobs (mm, deg or unitless "ul"), and
  - derived ones     `D(name, expr, unit, comment)`   - expressions over other parameters.

Expressions are written in the subset that Inventor and Python both understand
(`+ - * / ( )`, numbers with a unit suffix such as `0.3 mm`, `min(a; b)` / `max(a; b)` with
Inventor's `;` separator), so the very same string becomes an
Inventor user-parameter expression and is evaluated here for the checks and the initial sketch
geometry. Inventor therefore recomputes every derived value itself when a base parameter is
changed in its Parameters dialog.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

UNITS = {"mm": 1.0, "deg": 1.0, "ul": 1.0}
FUNCS = {"sqrt": math.sqrt, "abs": abs, "min": min, "max": max}
_IDENT = re.compile(r"\b[A-Za-z_][A-Za-z0-9_]*\b")
_NUM_UNIT = re.compile(r"(\d+(?:\.\d*)?|\.\d+)\s*(mm|deg|ul)\b")


@dataclass
class Param:
    name: str
    value: float | str      # number for base parameters, expression for derived ones
    unit: str = "mm"
    comment: str = ""
    derived: bool = False

    @property
    def expr(self) -> str:
        if self.derived:
            return str(self.value)
        return f"{self.value:.6g}" if self.unit == "ul" else f"{self.value:.6g} {self.unit}"


def P(name, value, unit="mm", comment=""):
    return Param(name, float(value), unit, comment, False)


def D(name, expr, unit="mm", comment=""):
    return Param(name, expr, unit, comment, True)


class ParamSet:
    """Ordered base + derived parameters with overrides, evaluation and dependency lookup."""

    def __init__(self, params: list[Param], overrides: dict | None = None):
        self.items: dict[str, Param] = {}
        for p in params:
            if p.name in self.items:
                raise ValueError(f"parameter {p.name} defined twice")
            self.items[p.name] = p
        for k, v in (overrides or {}).items():
            self.set(k, v)
        self._cache: dict[str, float] = {}

    def set(self, name: str, value) -> None:
        if name not in self.items:
            raise KeyError(f"unknown parameter {name!r}; known: {', '.join(self.items)}")
        p = self.items[name]
        if p.derived:
            p.value = str(value)               # a derived parameter may be overridden by an expression
        else:
            p.value = float(value)
        self._cache = {}

    def names_in(self, expr: str) -> list[str]:
        out = []
        for m in _IDENT.findall(_NUM_UNIT.sub("", str(expr))):
            if m in UNITS or m in FUNCS:
                continue
            if m not in self.items:
                raise KeyError(f"expression {expr!r} uses unknown name {m!r}")
            if m not in out:
                out.append(m)
        return out

    def ev(self, expr) -> float:
        """Value of an expression in mm / deg / unitless."""
        if isinstance(expr, (int, float)):
            return float(expr)
        py = _NUM_UNIT.sub(lambda m: m.group(1), str(expr)).replace(";", ",")
        ns = dict(FUNCS)
        for n in self.names_in(expr):
            ns[n] = self[n]
        return float(eval(py, {"__builtins__": {}}, ns))   # noqa: S307 - our own parameter table

    def __getitem__(self, name: str) -> float:
        if name not in self._cache:
            p = self.items[name]
            self._cache[name] = self.ev(p.value) if p.derived else float(p.value)
        return self._cache[name]

    def __getattr__(self, name: str) -> float:
        if name.startswith("_") or name in ("items",):
            raise AttributeError(name)
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def closure(self, names) -> list[str]:
        """`names` plus everything their expressions depend on, dependencies first (the order
        in which Inventor can create them)."""
        out: list[str] = []
        busy: set[str] = set()

        def visit(n: str) -> None:
            if n in out:
                return
            if n in busy:
                raise ValueError(f"parameter {n} depends on itself")
            busy.add(n)
            p = self.items[n]
            if p.derived:
                for d in self.names_in(p.value):
                    visit(d)
            busy.discard(n)
            out.append(n)

        for n in names:
            visit(n)
        return out

    def table(self) -> list[dict]:
        return [{"name": n, "value": round(self[n], 4), "unit": p.unit, "expr": p.expr if p.derived else "",
                 "comment": p.comment} for n, p in self.items.items()]

    def overrides_vs(self, defaults: "ParamSet") -> dict:
        return {n: p.value for n, p in self.items.items() if p.value != defaults.items[n].value}

    def dump(self, path: Path, extra: dict | None = None) -> None:
        data = {"parameters": self.table()}
        data.update(extra or {})
        Path(path).write_text(json.dumps(data, indent=1), encoding="utf-8")


def parse_sets(pairs: list[str] | None) -> dict:
    """['ch_w=0.8', 'well_d=15'] -> {'ch_w': '0.8', ...} (values stay strings until ParamSet.set)."""
    out = {}
    for s in pairs or []:
        if "=" not in s:
            raise SystemExit(f"--set expects name=value, got {s!r}")
        k, v = s.split("=", 1)
        out[k.strip()] = v.strip()
    return out
