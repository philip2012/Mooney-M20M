#!/usr/bin/env python3

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
GROUND_SETTLE_TIME = 5.0
MAX_START_TIME = 8.0
POST_START_TIME = 3.0
POINT_SETTLE_TIME = 5.0

THROTTLE = 0.75

MIN_RPM = 700.0
MAX_RPM = 2575.0

ADVANCE_POINTS = (
    1.00,
    0.95,
    0.90,
    0.85,
    0.80,
)

RPM_TOLERANCE = 60.0
PITCH_LIMIT_TOLERANCE_DEG = 0.05

prop_tree = ET.parse(REPO / "Engines" / "M20M-Propeller.xml")
MAX_PITCH = float(prop_tree.getroot().findtext("maxpitch"))

RUNNING = "propulsion/engine[0]/set-running"
RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
BLADE_ANGLE = "propulsion/engine[0]/blade-angle"

THROTTLE_HANDLE = (
    "systems/powerplant-controls/engine/handles/throttle-norm"
)
MIXTURE_HANDLE = (
    "systems/powerplant-controls/engine/handles/mixture-norm"
)
PROP_HANDLE = (
    "systems/powerplant-controls/engine/handles/prop-norm"
)
MAGNETOS = (
    "systems/powerplant-controls/engine/switches/magnetos"
)
STARTER = (
    "systems/powerplant-controls/engine/switches/starter"
)
BATTERY = (
    "systems/powerplant-controls/electrical/switches/battery-master"
)


def make_fdm():
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(REPO),
        str(REPO / "Engines"),
        str(REPO / "Systems"),
        False,
    ):
        raise RuntimeError("Could not load Mooney FDM")

    return fdm


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise SystemExit(
                "FAIL: JSBSim stopped unexpectedly"
            )


def target_rpm(advance):
    return MIN_RPM + (MAX_RPM - MIN_RPM) * advance


fdm = make_fdm()

# Ground initialization.
fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
fdm.set_property_value("ic/h-agl-ft", 4.30)
fdm.set_property_value("ic/phi-deg", 0.0)
fdm.set_property_value("ic/theta-deg", 0.0)
fdm.set_property_value("ic/psi-true-deg", 0.0)
fdm.set_property_value("ic/vg-kts", 0.0)

fdm.set_property_value("fcs/left-brake-cmd-norm", 1.0)
fdm.set_property_value("fcs/right-brake-cmd-norm", 1.0)
fdm.set_property_value("fcs/center-brake-cmd-norm", 1.0)

if not fdm.run_ic():
    raise SystemExit("FAIL: run_ic() failed")

run_for(fdm, GROUND_SETTLE_TIME)

# Start engine.
fdm.set_property_value(BATTERY, 1)
fdm.set_property_value(MIXTURE_HANDLE, 1.0)
fdm.set_property_value(PROP_HANDLE, 1.0)
fdm.set_property_value(THROTTLE_HANDLE, 0.15)
fdm.set_property_value(MAGNETOS, 3)
fdm.set_property_value(STARTER, 1)

start_time = fdm.get_sim_time()

while get(fdm, RUNNING) < 0.5:
    if fdm.get_sim_time() - start_time >= MAX_START_TIME:
        raise SystemExit("FAIL: engine did not start")

    if not fdm.run():
        raise SystemExit(
            "FAIL: simulation stopped during start"
        )

fdm.set_property_value(STARTER, 0)

run_for(fdm, POST_START_TIME)

if get(fdm, RUNNING) < 0.5:
    raise SystemExit(
        "FAIL: engine died after starter release"
    )

# Bring engine to governor-working power.
fdm.set_property_value(THROTTLE_HANDLE, THROTTLE)
fdm.set_property_value(PROP_HANDLE, 1.0)

run_for(fdm, POINT_SETTLE_TIME)


print("MOONEY PROP GOVERNOR TEST")
print("=========================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"throttle:       {THROTTLE:.2f}")
print()
print(
    " advance   target RPM   actual RPM   error   "
    "blade deg   MAP     HP"
)
print(
    " -------   ----------   ----------   ------  "
    "---------  ------  -------"
)

rows = []

for advance in ADVANCE_POINTS:
    fdm.set_property_value(PROP_HANDLE, advance)

    run_for(fdm, POINT_SETTLE_TIME)

    if get(fdm, RUNNING) < 0.5:
        raise SystemExit(
            f"FAIL: engine stopped at advance {advance:.2f}"
        )

    expected = target_rpm(advance)
    actual = get(fdm, RPM)
    blade = get(fdm, BLADE_ANGLE)
    map_inhg = get(fdm, MAP)
    hp = get(fdm, POWER)

    vals = (
        expected,
        actual,
        blade,
        map_inhg,
        hp,
    )

    if not all(math.isfinite(x) for x in vals):
        raise SystemExit(
            "FAIL: non-finite propulsion state"
        )

    error = actual - expected

    rows.append(
        {
            "advance": advance,
            "target": expected,
            "rpm": actual,
            "error": error,
            "blade": blade,
        }
    )

    print(
        f"   {advance:5.2f}   "
        f"{expected:10.1f}   "
        f"{actual:10.1f}   "
        f"{error:+6.1f}   "
        f"{blade:9.3f}  "
        f"{map_inhg:6.2f}  "
        f"{hp:7.2f}"
    )


# Functional governor checks.
saturated_points = []

for row in rows:
    if abs(row["error"]) > RPM_TOLERANCE:
        at_coarse_stop = (
            abs(row["blade"] - MAX_PITCH)
            <= PITCH_LIMIT_TOLERANCE_DEG
        )

        if at_coarse_stop and row["error"] > 0.0:
            saturated_points.append(row)
            continue

        raise SystemExit(
            "FAIL: governor RPM error exceeds "
            f"{RPM_TOLERANCE:.0f} RPM at "
            f"advance {row['advance']:.2f}"
        )

for previous, current in zip(rows, rows[1:]):
    if current["rpm"] >= previous["rpm"]:
        raise SystemExit(
            "FAIL: RPM did not decrease when "
            "prop advance was reduced"
        )

    if current["blade"] < previous["blade"] - 0.05:
        raise SystemExit(
            "FAIL: blade pitch became finer while "
            "requesting lower governed RPM"
        )


print()
print("RESULT")
print("------")
print("engine remained running: PASS")
print("governor target tracking: PASS within pitch envelope")
print("RPM direction response:   PASS")
print("blade-pitch response:     PASS")

if saturated_points:
    print(
        "coarse-pitch saturation:  "
        f"OBSERVED at {MAX_PITCH:.3f} deg"
    )
else:
    print("coarse-pitch saturation:  not reached")

print()
print("PROP GOVERNOR TEST PASS")
