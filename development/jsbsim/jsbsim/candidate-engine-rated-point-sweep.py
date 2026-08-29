#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
ENGINE_FILE = "Lycoming-TIO-540-AF1B.xml"

DT = 1.0 / 120.0

TARGET_MAP = 35.0
TARGET_HP = 270.0
TARGET_RPM = 2575.0

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

SETTLE_TIME = 1.5
BISECTION_STEPS = 12

# Re-sweep around the previous candidate because changing
# boost multiplier can move the required VE.
VE_POINTS = (
    0.840,
    0.845,
    0.850,
    0.855,
    0.860,
    0.865,
    0.870,
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
RAM_AIR = "propulsion/engine[0]/ram-air-factor"
AIRBOX = "propulsion/engine[0]/air-intake-impedance-factor"

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
FUEL_GPH = "propulsion/engine[0]/fuel-flow-rate-gph"
BLADE = "propulsion/engine[0]/blade-angle"
J = "propulsion/engine[0]/advance-ratio"
BOOST_LOSS = "propulsion/engine[0]/boostloss-hp"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def prepare_test_tree(root):
    shutil.copytree(
        REPO / "FDM",
        root / "FDM",
    )

    shutil.copytree(
        REPO / "Engines",
        root / "Engines",
    )

    path = root / "Engines" / ENGINE_FILE
    text = path.read_text()

    replacements = (
        (
            "    <air-intake-impedance-factor>"
            "1.0"
            "</air-intake-impedance-factor>\n",
            "",
        ),
        (
            '<ratedboost1 unit="INHG">8.1</ratedboost1>',
            '<ratedboost1 unit="INHG">6.58</ratedboost1>',
        ),
        (
            '<takeoffboost unit="INHG">8.1</takeoffboost>',
            '<takeoffboost unit="INHG">6.58</takeoffboost>',
        ),
    )

    for old, new in replacements:
        if text.count(old) != 1:
            raise RuntimeError(
                f"expected engine XML fragment not found exactly once:\n{old}"
            )

        text = text.replace(old, new)

    path.write_text(text)

    return root


def ramp_inflow(fdm):
    # Engine is already established before aerodynamic prop load
    # is introduced.
    for kts in (
        25.0,
        50.0,
        75.0,
        100.0,
        125.0,
        150.0,
        175.0,
    ):
        fdm.set_property_value(
            "atmosphere/wind-north-fps",
            -kts * KTS_TO_FPS,
        )

        run_for(fdm, 0.25)


def make_fdm(test_root, ve):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(test_root),
        str(test_root / "Engines"),
        str(REPO / "Systems"),
        False,
    ):
        raise RuntimeError("could not load test FDM")

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        0.0,
    )
    fdm.set_property_value("ic/h-agl-ft", 4.30)

    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)
    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    fdm.set_property_value(
        "forces/hold-down",
        1,
    )

    # No test wind during engine initialization.
    fdm.set_property_value(
        "atmosphere/wind-north-fps",
        0.0,
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
    fdm.set_property_value(THROTTLE, 0.65)
    fdm.set_property_value(MAGNETOS, 3)

    run_for(fdm, 0.1)

    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    fdm.set_property_value(
        VE_PROP,
        ve,
    )

    # Test wind exists solely to load the propeller.
    fdm.set_property_value(
        RAM_AIR,
        0.0,
    )

    # Let the initialized engine establish itself first.
    run_for(fdm, 1.5)

    if get(fdm, RPM) < 1000.0:
        raise RuntimeError(
            "engine failed to establish before inflow ramp"
        )

    ramp_inflow(fdm)

    run_for(fdm, 1.0)

    return fdm


def set_throttle(fdm, throttle):
    fdm.set_property_value(
        THROTTLE,
        throttle,
    )

    run_for(fdm, SETTLE_TIME)

    return get(fdm, MAP)


def find_target_map(fdm):
    low = 0.30
    high = 1.00

    low_map = set_throttle(
        fdm,
        low,
    )
    high_map = set_throttle(
        fdm,
        high,
    )

    if low_map > TARGET_MAP:
        raise RuntimeError(
            f"low throttle already gives {low_map:.2f} inHg"
        )

    if high_map < TARGET_MAP:
        raise RuntimeError(
            f"full throttle only gives {high_map:.2f} inHg"
        )

    for _ in range(BISECTION_STEPS):
        mid = (low + high) * 0.5
        map_inhg = set_throttle(
            fdm,
            mid,
        )

        if map_inhg < TARGET_MAP:
            low = mid
        else:
            high = mid

    throttle = (low + high) * 0.5

    set_throttle(
        fdm,
        throttle,
    )

    return throttle


print("MOONEY AF1B CANDIDATE RATED-POINT SWEEP")
print("========================================")
print(f"JSBSim version: {jsbsim.__version__}")
print("candidate turbo:")
print("  Z_airbox     = AUTO")
print("  rated boost  = 6.58 inHg")
print("  rated alt    = 20000 ft")
print("  takeoffboost = 6.58 inHg")
print()
print(
    f"target: {TARGET_HP:.1f} HP / "
    f"{TARGET_RPM:.0f} RPM / "
    f"{TARGET_MAP:.2f} inHg"
)
print()

print(
    "   VE    Z_airbox  throttle      J"
    "      RPM    blade     MAP       HP"
    "      GPH   boost"
)
print(
    " -----   --------  --------  ------"
    "  -------  -------  ------  -------"
    "  -------  ------"
)

best = None

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    for ve in VE_POINTS:
        fdm = make_fdm(
            test_root,
            ve,
        )

        throttle = find_target_map(
            fdm
        )

        row = {
            "ve": ve,
            "airbox": get(fdm, AIRBOX),
            "throttle": throttle,
            "j": get(fdm, J),
            "rpm": get(fdm, RPM),
            "blade": get(fdm, BLADE),
            "map": get(fdm, MAP),
            "hp": get(fdm, POWER),
            "gph": get(fdm, FUEL_GPH),
            "boost": get(fdm, BOOST_LOSS),
        }

        error = abs(
            row["hp"] - TARGET_HP
        )

        if best is None or error < best[0]:
            best = (
                error,
                row,
            )

        print(
            f" {row['ve']:5.3f}   "
            f"{row['airbox']:8.5f}  "
            f"{row['throttle']:8.4f}  "
            f"{row['j']:6.3f}  "
            f"{row['rpm']:7.1f}  "
            f"{row['blade']:7.2f}  "
            f"{row['map']:6.2f}  "
            f"{row['hp']:7.2f}  "
            f"{row['gph']:7.2f}  "
            f"{row['boost']:6.2f}"
        )

print()
print("NEAREST TO AF1B SEA-LEVEL RATED POINT")
print("--------------------------------------")

error, row = best

print(f"VE:        {row['ve']:.3f}")
print(f"Z_airbox:  {row['airbox']:.5f}")
print(f"throttle:  {row['throttle']:.4f}")
print(f"RPM:       {row['rpm']:.1f}")
print(f"MAP:       {row['map']:.2f} inHg")
print(f"power:     {row['hp']:.2f} HP")
print(f"error:     {error:.2f} HP")
print(f"fuel flow: {row['gph']:.2f} GPH")
print(f"boostloss: {row['boost']:.2f} HP")
