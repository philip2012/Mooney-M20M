#!/usr/bin/env python3

from pathlib import Path
import os
import math

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

ALTITUDE_FT = float(
    os.environ.get(
        "M20M_PROP_ALT_FT",
        "5000",
    )
)

TEST_SPEEDS_KTS = tuple(
    float(v)
    for v in os.environ.get(
        "M20M_PROP_SPEEDS",
        "90,105,120,135,150,165,180",
    ).split(",")
)

ENGINE_DIR = Path(
    os.environ.get(
        "M20M_ENGINE_DIR",
        str(REPO / "Engines"),
    )
)

AIRCRAFT_ROOT = Path(
    os.environ.get(
        "M20M_AIRCRAFT_ROOT",
        str(REPO),
    )
)

KTS_TO_FPS = 1.687809857


THROTTLE = (
    "systems/powerplant-controls/engine/"
    "handles/throttle-norm"
)

MIXTURE = (
    "systems/powerplant-controls/engine/"
    "handles/mixture-norm"
)

PROP = (
    "systems/powerplant-controls/engine/"
    "handles/prop-norm"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/"
    "switches/magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/"
    "switches/battery-master"
)

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
THRUST = "propulsion/engine[0]/thrust-lbs"
J = "propulsion/engine[0]/advance-ratio"
BLADE = "propulsion/engine[0]/blade-angle"
TIP_MACH = "propulsion/engine[0]/helical-tip-Mach"
CT = "propulsion/engine[0]/thrust-coefficient"
PROP_POWER = (
    "propulsion/engine[0]/propeller-power-ftlbps"
)


def get(fdm, prop):
    return float(
        fdm.get_property_value(prop)
    )


def setv(fdm, prop, value):
    fdm.set_property_value(
        prop,
        float(value),
    )


def run_for(fdm, seconds):
    frames = int(seconds / DT)

    for _ in range(frames):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


fdm = jsbsim.FGFDMExec(None)
fdm.set_debug_level(0)
fdm.set_dt(DT)

if not fdm.load_model_with_paths(
    MODEL,
    str(AIRCRAFT_ROOT),
    str(ENGINE_DIR),
    str(REPO / "Systems"),
    False,
):
    raise SystemExit(
        "FAIL: production FDM did not load"
    )


# Artificial atmosphere point well above the ground.
setv(
    fdm,
    "ic/terrain-elevation-ft",
    ALTITUDE_FT - 1000.0,
)

setv(
    fdm,
    "ic/h-agl-ft",
    1000.0,
)

setv(
    fdm,
    "ic/vg-kts",
    0.0,
)

setv(
    fdm,
    "ic/psi-true-deg",
    0.0,
)

# Supported all-engines-running initialization.
setv(
    fdm,
    "propulsion/set-running",
    -1.0,
)

if not fdm.run_ic():
    raise SystemExit(
        "FAIL: run_ic() failed"
    )


# Keep the rigid body fixed. Only the propeller sees inflow.
setv(
    fdm,
    "forces/hold-down",
    1.0,
)

setv(
    fdm,
    "propulsion/engine[0]/ram-air-factor",
    0.0,
)

setv(fdm, BATTERY, 1.0)
setv(fdm, MIXTURE, 1.0)
setv(fdm, PROP, 1.0)
setv(fdm, MAGNETOS, 3.0)
setv(fdm, THROTTLE, 1.0)

run_for(
    fdm,
    5.0,
)


print(
    "MOONEY M20M PRODUCTION "
    "PROPELLER EFFICIENCY SWEEP"
)

print("=" * 82)

print(
    f"JSBSim version: "
    f"{jsbsim.__version__}"
)

print(
    f"Altitude: {ALTITUDE_FT:.0f} ft"
)

print(
    f"Aircraft root: {AIRCRAFT_ROOT}"
)

print(
    f"Engine directory: {ENGINE_DIR}"
)

print(
    "Rigid body held fixed; aerodynamic "
    "performance is excluded."
)

print()

print(
    " TAS      J    blade   Mtip      RPM"
    "     MAP      HP   thrust"
    "      Ct      Cp"
    "   reqHP  propHP    eta  Pmatch"
)

print(
    "----  -----  -------  -----  -------"
    "  ------  ------  -------"
    "  ------  ------"
    "  ------  ------  -----  ------"
)


for speed_kts in TEST_SPEEDS_KTS:
    target_wind = (
        -speed_kts
        * KTS_TO_FPS
    )

    # Ramp inflow to avoid a violent instantaneous
    # governor transient.
    current = get(
        fdm,
        "atmosphere/wind-north-fps",
    )

    frames = int(
        2.0 / DT
    )

    for i in range(frames):
        fraction = (
            (i + 1)
            / frames
        )

        wind = (
            current
            + (
                target_wind
                - current
            )
            * fraction
        )

        setv(
            fdm,
            "atmosphere/wind-north-fps",
            wind,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during inflow ramp"
            )

    run_for(
        fdm,
        8.0,
    )

    rpm = get(fdm, RPM)
    map_inhg = get(fdm, MAP)
    hp = get(fdm, POWER)
    thrust = get(fdm, THRUST)
    advance = get(fdm, J)
    blade = get(fdm, BLADE)
    tip_mach = get(fdm, TIP_MACH)
    ct = get(fdm, CT)

    prop_required_hp = (
        get(fdm, PROP_POWER)
        / 550.0
    )

    prop_hp = (
        thrust
        * speed_kts
        * KTS_TO_FPS
        / 550.0
    )

    eta = (
        prop_hp / prop_required_hp
        if prop_required_hp > 1.0
        else float("nan")
    )

    cp = (
        advance * ct / eta
        if abs(eta) > 1.0e-8
        else float("nan")
    )

    power_match = (
        prop_required_hp / hp
        if hp > 1.0
        else float("nan")
    )

    values = (
        rpm,
        map_inhg,
        hp,
        thrust,
        advance,
        blade,
        tip_mach,
        ct,
        cp,
        prop_required_hp,
        prop_hp,
        eta,
        power_match,
    )

    if not all(
        math.isfinite(v)
        for v in values
    ):
        raise SystemExit(
            "FAIL: non-finite propeller result"
        )

    print(
        f"{speed_kts:4.0f} "
        f"{advance:6.3f} "
        f"{blade:8.2f} "
        f"{tip_mach:6.3f} "
        f"{rpm:8.1f} "
        f"{map_inhg:7.2f} "
        f"{hp:7.2f} "
        f"{thrust:8.1f} "
        f"{ct:7.4f} "
        f"{cp:7.4f} "
        f"{prop_required_hp:7.1f} "
        f"{prop_hp:7.1f} "
        f"{eta:6.3f} "
        f"{power_match:7.3f}"
    )
