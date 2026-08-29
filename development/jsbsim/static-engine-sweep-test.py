#!/usr/bin/env python3

from pathlib import Path
import math

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

SETTLE_TIME = 5.0
MAX_START_TIME = 8.0
POST_START_TIME = 3.0
POINT_SETTLE_TIME = 5.0

THROTTLE_POINTS = (
    0.10,
    0.25,
    0.50,
    0.75,
    1.00,
)

RUNNING = "propulsion/engine[0]/set-running"
RPM = "propulsion/engine[0]/propeller-rpm"
POWER_HP = "propulsion/engine[0]/power-hp"
MAP_INHG = "propulsion/engine[0]/map-inhg"
THRUST_LBS = "propulsion/engine[0]/thrust-lbs"

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
            raise SystemExit("FAIL: JSBSim stopped unexpectedly")


def snapshot(fdm, throttle):
    values = {
        "throttle": throttle,
        "rpm": get(fdm, RPM),
        "map": get(fdm, MAP_INHG),
        "hp": get(fdm, POWER_HP),
        "thrust": get(fdm, THRUST_LBS),
    }

    if not all(math.isfinite(v) for v in values.values()):
        raise SystemExit("FAIL: non-finite propulsion value")

    return values


fdm = make_fdm()

# ------------------------------------------------------------
# Ground initialization.
# ------------------------------------------------------------

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

run_for(fdm, SETTLE_TIME)


# ------------------------------------------------------------
# Start engine.
# ------------------------------------------------------------

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
        raise SystemExit("FAIL: simulation stopped during start")

fdm.set_property_value(STARTER, 0)

run_for(fdm, POST_START_TIME)

if get(fdm, RUNNING) < 0.5:
    raise SystemExit("FAIL: engine died after starter release")


# ------------------------------------------------------------
# Static throttle sweep.
# ------------------------------------------------------------

print("MOONEY STATIC ENGINE SWEEP TEST")
print("===============================")
print(f"JSBSim version: {jsbsim.__version__}")
print()
print(
    " throttle       RPM      MAP      HP      thrust"
)
print(
    " --------  --------  -------  -------  ---------"
)

rows = []

for throttle in THROTTLE_POINTS:
    fdm.set_property_value(
        THROTTLE_HANDLE,
        throttle,
    )

    run_for(fdm, POINT_SETTLE_TIME)

    if get(fdm, RUNNING) < 0.5:
        raise SystemExit(
            f"FAIL: engine stopped at throttle {throttle:.2f}"
        )

    row = snapshot(fdm, throttle)
    rows.append(row)

    print(
        f"   {row['throttle']:.2f}    "
        f"{row['rpm']:8.1f}  "
        f"{row['map']:7.2f}  "
        f"{row['hp']:7.2f}  "
        f"{row['thrust']:9.2f}"
    )


# ------------------------------------------------------------
# Baseline integrity checks.
# ------------------------------------------------------------

MAP_TOLERANCE_INHG = 0.05

for previous, current in zip(rows, rows[1:]):
    if current["map"] < previous["map"] - MAP_TOLERANCE_INHG:
        raise SystemExit(
            "FAIL: manifold pressure decreased "
            "with increasing throttle"
        )

    if current["hp"] <= previous["hp"]:
        raise SystemExit(
            "FAIL: engine power did not increase "
            "with throttle"
        )

if rows[-1]["map"] <= rows[0]["map"]:
    raise SystemExit(
        "FAIL: MAP did not increase across throttle sweep"
    )

if rows[-1]["rpm"] <= rows[0]["rpm"]:
    raise SystemExit(
        "FAIL: RPM did not increase across throttle sweep"
    )

print()
print("RESULT")
print("------")
print("engine remained running: PASS")
print("MAP response:             PASS")
print("power response:           PASS")
print("overall RPM response:     PASS")
print()
print("STATIC ENGINE SWEEP TEST PASS")
