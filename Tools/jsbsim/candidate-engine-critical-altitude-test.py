#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
ENGINE_FILE = "Lycoming-TIO-540-AF1B.xml"

DT = 1.0 / 120.0

TEST_VE = 0.865

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

ALTITUDES = (
    0.0,
    5000.0,
    10000.0,
    15000.0,
    18000.0,
    19000.0,
    20000.0,
    21000.0,
    22000.0,
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
ADVANCE_RATIO = "propulsion/engine[0]/advance-ratio"
BOOST_LOSS = "propulsion/engine[0]/boostloss-hp"

TEMP_R = "atmosphere/T-R"


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
                f"engine XML fragment not found exactly once:\n{old}"
            )

        text = text.replace(old, new)

    path.write_text(text)

    return root


def ramp_inflow(fdm):
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


def run_case(test_root, altitude):
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
        altitude,
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
    fdm.set_property_value(THROTTLE, 1.0)
    fdm.set_property_value(MAGNETOS, 3)

    run_for(fdm, 0.1)

    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    fdm.set_property_value(
        VE_PROP,
        TEST_VE,
    )

    fdm.set_property_value(
        RAM_AIR,
        0.0,
    )

    run_for(fdm, 1.5)

    if get(fdm, RPM) < 1000.0:
        raise RuntimeError(
            f"engine failed to establish at {altitude:.0f} ft"
        )

    ramp_inflow(fdm)
    run_for(fdm, 3.0)

    return {
        "alt": altitude,
        "temp_f": get(fdm, TEMP_R) - 459.67,
        "airbox": get(fdm, AIRBOX),
        "j": get(fdm, ADVANCE_RATIO),
        "rpm": get(fdm, RPM),
        "blade": get(fdm, BLADE),
        "map": get(fdm, MAP),
        "hp": get(fdm, POWER),
        "gph": get(fdm, FUEL_GPH),
        "boost": get(fdm, BOOST_LOSS),
    }


print("MOONEY AF1B CANDIDATE CRITICAL-ALTITUDE TEST")
print("=============================================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"runtime VE:     {TEST_VE:.3f}")
print("Z_airbox:       AUTO")
print("rated boost:    6.58 inHg")
print("rated altitude: 20000 ft")
print(f"prop inflow:    {INFLOW_KTS:.0f} kt")
print()

print(
    " altitude    OAT F  Z_airbox      J"
    "      RPM    blade     MAP       HP"
    "      GPH   boost"
)
print(
    " --------  -------  --------  ------"
    "  -------  -------  ------  -------"
    "  -------  ------"
)

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    for altitude in ALTITUDES:
        r = run_case(
            test_root,
            altitude,
        )

        print(
            f"{r['alt']:8.0f}  "
            f"{r['temp_f']:7.1f}  "
            f"{r['airbox']:8.5f}  "
            f"{r['j']:6.3f}  "
            f"{r['rpm']:7.1f}  "
            f"{r['blade']:7.2f}  "
            f"{r['map']:6.2f}  "
            f"{r['hp']:7.2f}  "
            f"{r['gph']:7.2f}  "
            f"{r['boost']:6.2f}"
        )

print()
print("CANDIDATE CRITICAL-ALTITUDE TEST COMPLETE")
