#!/usr/bin/env python3
from __future__ import annotations

import argparse
import difflib
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[2]
TARGET = REPO / "FDM/Config/MooneyM20M-ground-reactions.xml"
GEAR_NAMES = ("nose", "left-main", "right-main")

DATUM_X = {"nose": -13.000, "left-main": 66.563, "right-main": 66.563}
BASELINE_Z = {"nose": -51.0, "left-main": -51.0, "right-main": -51.0}
CANDIDATE_Z = {"nose": -50.46, "left-main": -50.08, "right-main": -50.08}


def validate(text: str) -> ET.Element:
    root = ET.fromstring(text)
    found = {c.attrib.get("name") for c in root.findall("contact")}
    missing = set(GEAR_NAMES) - found
    if missing:
        raise RuntimeError(f"Missing contacts: {sorted(missing)}")
    return root


def geometry(text: str):
    root = validate(text)
    out = {}
    for name in GEAR_NAMES:
        c = root.find(f"contact[@name='{name}']")
        loc = c.find("location")
        out[name] = (
            float(loc.findtext("x")),
            float(loc.findtext("y")),
            float(loc.findtext("z")),
        )
    return out


def replace_coord(text: str, name: str, tag: str, value: float) -> str:
    pat = re.compile(
        rf'(<contact\b[^>]*\bname="{re.escape(name)}"[^>]*>.*?</contact>)',
        re.DOTALL,
    )
    m = pat.search(text)
    if not m:
        raise RuntimeError(f"Contact not found: {name}")
    block = m.group(1)

    loc_pat = re.compile(r'(<location\b[^>]*>)(.*?)(</location>)', re.DOTALL)
    lm = loc_pat.search(block)
    if not lm:
        raise RuntimeError(f"Location not found: {name}")

    body = lm.group(2)
    coord_pat = re.compile(rf'(<{tag}>\s*)([-+0-9.eE]+)(\s*</{tag}>)')
    if len(coord_pat.findall(body)) != 1:
        raise RuntimeError(f"Expected one <{tag}> in {name}")

    if tag == "x":
        value_text = f"{value:.3f}"
    elif tag == "z":
        value_text = f"{value:.2f}" if abs(value - round(value, 1)) > 1e-12 else f"{value:.1f}"
    else:
        value_text = f"{value:g}"

    new_body = coord_pat.sub(
        lambda mm: f"{mm.group(1)}{value_text}{mm.group(3)}",
        body,
        count=1,
    )
    new_loc = lm.group(1) + new_body + lm.group(3)
    new_block = block[:lm.start()] + new_loc + block[lm.end():]
    return text[:m.start()] + new_block + text[m.end():]


def apply_updates(text: str, xs=None, zs=None) -> str:
    out = text
    if xs:
        for name, value in xs.items():
            out = replace_coord(out, name, "x", value)
    if zs:
        for name, value in zs.items():
            out = replace_coord(out, name, "z", value)
    validate(out)
    return out


def show(text: str):
    print(f"Target: {TARGET.relative_to(REPO)}")
    print("gear           X(in)       Y(in)       Z(in)")
    print("------------  ----------  ----------  ----------")
    for name, (x, y, z) in geometry(text).items():
        print(f"{name:<12}  {x:>10.3f}  {y:>10.3f}  {z:>10.3f}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("preset", choices=("show", "datum-x", "baseline-z", "candidate-z", "datum-step"))
    p.add_argument("--apply", action="store_true")
    args = p.parse_args()

    if not TARGET.is_file():
        print(f"ERROR: missing {TARGET}", file=sys.stderr)
        return 2

    before = TARGET.read_text(encoding="utf-8")
    validate(before)

    if args.preset == "show":
        show(before)
        return 0

    if args.preset == "datum-x":
        after = apply_updates(before, xs=DATUM_X)
    elif args.preset == "baseline-z":
        after = apply_updates(before, zs=BASELINE_Z)
    elif args.preset == "candidate-z":
        after = apply_updates(before, zs=CANDIDATE_Z)
    else:
        after = apply_updates(before, xs=DATUM_X, zs=BASELINE_Z)

    diff = "".join(difflib.unified_diff(
        before.splitlines(keepends=True),
        after.splitlines(keepends=True),
        fromfile=str(TARGET.relative_to(REPO)),
        tofile=str(TARGET.relative_to(REPO)),
    ))
    print(diff if diff else "No changes required.")

    if not args.apply:
        print("\nDry run only. Re-run with --apply.")
        return 0

    TARGET.write_text(after, encoding="utf-8")
    validate(TARGET.read_text(encoding="utf-8"))
    print("\nApplied successfully.")
    show(after)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
