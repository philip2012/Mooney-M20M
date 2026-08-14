#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

ENGINE_FILE = "Lycoming-TIO-540-AF1B.xml"

DT = 1.0 / 120.0
SETTLE_TIME = 6.0

TEST_VE = 0.865

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

ALTITUDES_FT = (
    0.0,
    5000.0,
    10000.0,
    15000.0,
    17000.0,
    18000.0,
    19000.0,
    20000.0,
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
BOOST_LOSS = "propulsion/engine[0]/boostloss-hp"
BLADE = "propulsion/engine[0]/blade-angle"
ADVANCE_RATIO = "propulsion/engine[0]/advance-ratio"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def prepare_test_tree(root, auto_impedance):
    # JSBSim searches <FullAircraftPath>/Engines before EnginePath.
    # Therefore the whole aircraft test root must be temporary;
    # otherwise the repository's real Engines/ directory wins.
    shutil.copytree(
        REPO / "FDM",
        root / "FDM",
    )

    shutil.copytree(
        REPO / "Engines",
        root / "Engines",
    )

    engine_file = root / "Engines" / ENGINE_FILE

    text = engine_file.read_text()

    impedance_element = (
        "    <air-intake-impedance-factor>"
        "1.0"
        "</air-intake-impedance-factor>\n"
    )

    ram_air_element = (
        "    <ram-air-factor>"
        "0.2"
        "</ram-air-factor>\n"
    )

    # The permanent AF1B baseline now intentionally omits the
    # impedance element so JSBSim calculates Z_airbox automatically.
    if text.count(impedance_element) != 0:
        raise RuntimeError(
            "repository baseline unexpectedly contains "
            "explicit intake impedance"
        )

    if not auto_impedance:
        # Reconstruct the old explicit-1.0 configuration only in the
        # temporary comparison tree.
        if text.count(ram_air_element) != 1:
            raise RuntimeError(
                "expected ram-air element exactly once"
            )

        text = text.replace(
            ram_air_element,
            impedance_element + ram_air_element,
            1,
        )

        engine_file.write_text(text)

    return root


def run_case(test_root, altitude_ft):
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
        raise RuntimeError("could not load Mooney FDM")

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

    fdm.set_property_value(
        "forces/hold-down",
        1,
    )

    # Controlled propeller freestream.
    fdm.set_property_value(
        "atmosphere/wind-north-fps",
        -INFLOW_KTS * KTS_TO_FPS,
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

    # Diagnostic-only overrides.
    fdm.set_property_value(
        VE_PROP,
        TEST_VE,
    )

    # Test wind loads the prop only.
    fdm.set_property_value(
        RAM_AIR,
        0.0,
    )

    run_for(fdm, SETTLE_TIME)

    return {
        "alt": altitude_ft,
        "airbox": get(fdm, AIRBOX),
        "j": get(fdm, ADVANCE_RATIO),
        "rpm": get(fdm, RPM),
        "blade": get(fdm, BLADE),
        "map": get(fdm, MAP),
        "hp": get(fdm, POWER),
        "gph": get(fdm, FUEL_GPH),
        "boost": get(fdm, BOOST_LOSS),
    }


print("MOONEY INTAKE IMPEDANCE DIAGNOSTIC")
print("==================================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"runtime VE:     {TEST_VE:.3f}")
print(f"prop inflow:    {INFLOW_KTS:.0f} kt")
print("ram-air factor: 0.0")
print()

for name, auto in (
    ("EXPLICIT Z_AIRBOX = 1.0", False),
    ("JSBSIM AUTO Z_AIRBOX", True),
):
    print(name)
    print("-" * len(name))

    with tempfile.TemporaryDirectory() as tmp:
        test_root = prepare_test_tree(
            Path(tmp),
            auto,
        )

        print(
            " altitude   Z_airbox      J      RPM"
            "    blade     MAP       HP      GPH   boost"
        )
        print(
            " --------   --------  ------  -------"
            "  -------  ------  -------  -------  ------"
        )

        for altitude in ALTITUDES_FT:
            r = run_case(
                test_root,
                altitude,
            )

            print(
                f"{r['alt']:8.0f}   "
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

print("INTAKE IMPEDANCE DIAGNOSTIC COMPLETE")
