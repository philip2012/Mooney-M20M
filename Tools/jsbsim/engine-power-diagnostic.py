#!/usr/bin/env python3

from pathlib import Path
import math

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
GROUND_SETTLE_TIME = 5.0
START_TIMEOUT = 8.0
POST_START_TIME = 3.0
POINT_SETTLE_TIME = 6.0

THROTTLE_POINTS = (0.50, 0.75, 1.00)

RUNNING = "propulsion/engine[0]/set-running"
RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
THRUST = "propulsion/engine[0]/thrust-lbs"

FUEL_PPS = "propulsion/engine[0]/fuel-flow-rate-pps"
FUEL_GPH = "propulsion/engine[0]/fuel-flow-rate-gph"
ISFC = "propulsion/engine[0]/bsfc-lbs_hphr"
STATIC_FRICTION = "propulsion/engine[0]/friction-hp"
VE = "propulsion/engine[0]/volumetric-efficiency"
BOOST_LOSS = "propulsion/engine[0]/boostloss-hp"
AFR = "propulsion/engine[0]/AFR"

PROP_POWER = "propulsion/engine[0]/propeller-power-ftlbps"
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


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise SystemExit(
                "FAIL: JSBSim stopped unexpectedly"
            )


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
    raise SystemExit("FAIL: could not load Mooney FDM")


# Ground setup.
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


# Cold start.
fdm.set_property_value(BATTERY, 1)
fdm.set_property_value(MIXTURE_HANDLE, 1.0)
fdm.set_property_value(PROP_HANDLE, 1.0)
fdm.set_property_value(THROTTLE_HANDLE, 0.15)
fdm.set_property_value(MAGNETOS, 3)
fdm.set_property_value(STARTER, 1)

start = fdm.get_sim_time()

while get(fdm, RUNNING) < 0.5:
    if fdm.get_sim_time() - start > START_TIMEOUT:
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


print("MOONEY ENGINE POWER DIAGNOSTIC")
print("==============================")
print(f"JSBSim version: {jsbsim.__version__}")
print()

for throttle in THROTTLE_POINTS:
    fdm.set_property_value(THROTTLE_HANDLE, throttle)
    fdm.set_property_value(PROP_HANDLE, 1.0)

    run_for(fdm, POINT_SETTLE_TIME)

    rpm = get(fdm, RPM)
    map_inhg = get(fdm, MAP)
    hp = get(fdm, POWER)
    thrust = get(fdm, THRUST)

    fuel_pps = get(fdm, FUEL_PPS)
    fuel_pph = fuel_pps * 3600.0
    fuel_gph = get(fdm, FUEL_GPH)

    isfc = get(fdm, ISFC)
    friction = get(fdm, STATIC_FRICTION)
    ve = get(fdm, VE)
    boost_loss = get(fdm, BOOST_LOSS)
    afr = get(fdm, AFR)

    prop_power_hp = get(fdm, PROP_POWER) / 550.0
    blade = get(fdm, BLADE_ANGLE)

    fuel_hp_basis = (
        fuel_pph / isfc
        if isfc > 0.0
        else float("nan")
    )

    values = (
        rpm,
        map_inhg,
        hp,
        thrust,
        fuel_pph,
        fuel_gph,
        isfc,
        friction,
        ve,
        boost_loss,
        afr,
        prop_power_hp,
        blade,
        fuel_hp_basis,
    )

    if not all(math.isfinite(x) for x in values):
        raise SystemExit(
            f"FAIL: non-finite value at throttle {throttle}"
        )

    print(f"THROTTLE {throttle:.2f}")
    print("----------------")
    print(f"RPM:                 {rpm:10.2f}")
    print(f"MAP inHg:            {map_inhg:10.3f}")
    print(f"final engine HP:     {hp:10.3f}")
    print(f"thrust lb:           {thrust:10.3f}")
    print()
    print(f"fuel flow lb/hr:     {fuel_pph:10.3f}")
    print(f"fuel flow US gal/hr: {fuel_gph:10.3f}")
    print(f"ISFC lb/hp/hr:       {isfc:10.5f}")
    print(f"fuel/ISFC HP basis:  {fuel_hp_basis:10.3f}")
    print(f"static friction HP:  {friction:10.3f}")
    print(f"boost loss HP:       {boost_loss:10.3f}")
    print(f"volumetric eff.:     {ve:10.5f}")
    print(f"AFR:                 {afr:10.3f}")
    print()
    print(f"prop absorbed HP:    {prop_power_hp:10.3f}")
    print(f"blade angle deg:     {blade:10.3f}")
    print()


print("ENGINE POWER DIAGNOSTIC COMPLETE")
