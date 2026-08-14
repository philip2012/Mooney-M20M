#!/usr/bin/env python3

from pathlib import Path

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
SETTLE_TIME = 5.0

TEST_VE = 0.765
TARGET_RPM = 2575.0
MAX_PITCH_DEG = 44.5

KTS_TO_FPS = 1.687809857

# Use the engine-facing throttle values previously found to make
# approximately 270 HP in the static density-controller diagnostic.
ALTITUDE_CASES = (
    (15000.0, 0.8925),
    (19000.0, 0.9651),
    (20000.0, 1.0000),
)

# Controlled freestream speeds. Aircraft ground speed remains zero.
INFLOW_KTS = (
    0.0,
    100.0,
    150.0,
    175.0,
    200.0,
)

THROTTLE = (
    "systems/powerplant-controls/engine/handles/throttle-norm"
)
MIXTURE = (
    "systems/powerplant-controls/engine/handles/mixture-norm"
)
PROP = (
    "systems/powerplant-controls/engine/handles/prop-norm"
)
MAGNETOS = (
    "systems/powerplant-controls/engine/switches/magnetos"
)
BATTERY = (
    "systems/powerplant-controls/electrical/switches/battery-master"
)

VE_PROP = "propulsion/engine[0]/volumetric-efficiency"
RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
PROP_POWER = "propulsion/engine[0]/propeller-power-ftlbps"
BLADE = "propulsion/engine[0]/blade-angle"
ADVANCE_RATIO = "propulsion/engine[0]/advance-ratio"
THRUST = "propulsion/engine[0]/thrust-lbs"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def run_case(altitude_ft, throttle, inflow_kts):
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
        raise RuntimeError("could not load Mooney FDM")

    # Aircraft points north and remains stationary.
    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        altitude_ft,
    )
    fdm.set_property_value("ic/h-agl-ft", 4.30)
    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)
    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    # Freeze aircraft motion. JSBSim hold-down zeros body velocity,
    # so freestream must be supplied through the runtime wind model.
    fdm.set_property_value(
        "forces/hold-down",
        1,
    )

    # Aircraft heading is north. A negative north wind vector makes
    # AeroU positive because JSBSim computes aerodynamic velocity as
    # aircraft velocity minus wind velocity.
    fdm.set_property_value(
        "atmosphere/wind-north-fps",
        -inflow_kts * KTS_TO_FPS,
    )
    fdm.set_property_value(
        "atmosphere/wind-east-fps",
        0.0,
    )
    fdm.set_property_value(
        "atmosphere/wind-down-fps",
        0.0,
    )

    fdm.set_property_value(BATTERY, 1)
    fdm.set_property_value(MIXTURE, 1.0)
    fdm.set_property_value(PROP, 1.0)
    fdm.set_property_value(THROTTLE, throttle)
    fdm.set_property_value(MAGNETOS, 3)

    run_for(fdm, 0.1)

    # Initialize engine 0.
    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    # Runtime-only VE candidate.
    fdm.set_property_value(
        VE_PROP,
        TEST_VE,
    )

    run_for(fdm, SETTLE_TIME)

    rpm = get(fdm, RPM)
    blade = get(fdm, BLADE)
    j = get(fdm, ADVANCE_RATIO)

    if inflow_kts > 1.0 and abs(j) < 0.01:
        raise RuntimeError(
            f"inflow test failed: {inflow_kts:.0f} kt requested "
            f"but advance ratio remained {j:.4f}"
        )

    if blade >= MAX_PITCH_DEG - 0.01:
        status = "MAX PITCH"
    elif abs(rpm - TARGET_RPM) <= 10.0:
        status = "GOVERNING"
    else:
        status = "OFF TARGET"

    return {
        "alt": altitude_ft,
        "inflow": inflow_kts,
        "throttle": throttle,
        "rpm": rpm,
        "map": get(fdm, MAP),
        "hp": get(fdm, POWER),
        "prop_hp": get(fdm, PROP_POWER) / 550.0,
        "blade": blade,
        "j": j,
        "thrust": get(fdm, THRUST),
        "status": status,
    }


print("MOONEY PROP INFLOW ENVELOPE DIAGNOSTIC")
print("======================================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"runtime VE:     {TEST_VE:.3f}")
print(f"target RPM:     {TARGET_RPM:.0f}")
print()

print(
    " altitude  inflow      J      RPM     blade"
    "     MAP       HP   propHP   thrust   status"
)
print(
    " --------  ------  ------  -------  -------"
    "  ------  -------  -------  -------  ----------"
)

for altitude, throttle in ALTITUDE_CASES:
    for inflow in INFLOW_KTS:
        r = run_case(
            altitude,
            throttle,
            inflow,
        )

        print(
            f"{r['alt']:8.0f}  "
            f"{r['inflow']:6.0f}  "
            f"{r['j']:6.3f}  "
            f"{r['rpm']:7.1f}  "
            f"{r['blade']:7.2f}  "
            f"{r['map']:6.2f}  "
            f"{r['hp']:7.2f}  "
            f"{r['prop_hp']:7.2f}  "
            f"{r['thrust']:7.1f}  "
            f"{r['status']}"
        )

    print()

print("PROP INFLOW ENVELOPE DIAGNOSTIC COMPLETE")
