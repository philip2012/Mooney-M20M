#!/usr/bin/env python3

"""
AF1B integration startup/state regression.

Uses the proposed one-writer integration architecture:
    AF1B controller -> local throttle limit
    powerplant-controls -> final fcs/throttle-cmd-norm[0]

FlightGear Nasal is NOT executed by standalone JSBSim.
For take-off and approach this harness therefore emulates only
the documented runtime engine-start actions from state-init.nas.

Permanent aircraft files are untouched.
"""

from pathlib import Path
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET

import jsbsim


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"
DT = 1.0 / 120.0

BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
VE_ENABLED = BASE + "/ve-enabled"
CAPABILITY_ENABLED = BASE + "/capability-enabled"

LIMIT = BASE + "/throttle-limit-norm"
TARGET = BASE + "/target-map-inhg"
CONTROLLER_TARGET = (
    BASE + "/controller-target-map-inhg"
)

VE = (
    "propulsion/engine[0]/"
    "volumetric-efficiency"
)

PILOT = (
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

STARTER = (
    "systems/powerplant-controls/engine/"
    "switches/starter"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/"
    "switches/magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/"
    "switches/battery-master"
)

ALTERNATOR = (
    "systems/powerplant-controls/electrical/"
    "switches/alternator"
)

START_ALLOWED = (
    "systems/powerplant-controls/engine/"
    "states/start-allowed"
)

STARTER_CMD = "propulsion/starter_cmd"

THROTTLE_CMD = "fcs/throttle-cmd-norm[0]"
RPM = "propulsion/engine[0]/propeller-rpm"


STATE_FILES = {
    "parked": REPO / "States/parked-overlay.xml",
    "ready-to-start": (
        REPO / "States/ready-to-start-overlay.xml"
    ),
    "take-off": (
        REPO / "States/take-off-overlay.xml"
    ),
    "approach": (
        REPO / "States/approach-overlay.xml"
    ),
}


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_frames(fdm, count):
    for _ in range(count):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


def run_for(fdm, seconds):
    run_frames(
        fdm,
        int(seconds / DT),
    )


def prepare(root):
    shutil.copytree(
        REPO / "FDM",
        root / "FDM",
    )

    shutil.copytree(
        REPO / "Engines",
        root / "Engines",
    )

    shutil.copytree(
        REPO / "Systems",
        root / "Systems",
    )

    shutil.copy2(
        REPO
        / "Tools"
        / "jsbsim"
        / "powerplant-controls-integration-candidate.xml",
        root
        / "Systems"
        / "powerplant-controls.xml",
    )

    shutil.copy2(
        REPO
        / "Tools"
        / "jsbsim"
        / "af1b-density-controller-integration-candidate.xml",
        root
        / "Systems"
        / "af1b-density-controller.xml",
    )

    fdm_path = (
        root
        / "FDM"
        / "Mooney-M20M.xml"
    )

    text = fdm_path.read_text()

    old = '''    <system file="airframe-controls" />
    <system file="powerplant-controls" />
'''

    new = '''    <system file="airframe-controls" />
    <system file="af1b-density-controller" />
    <system file="powerplant-controls" />
'''

    if text.count(old) != 1:
        raise RuntimeError(
            "Expected permanent system block once"
        )

    fdm_path.write_text(
        text.replace(
            old,
            new,
            1,
        )
    )


def make_fdm(root):
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(root),
        str(root / "Engines"),
        str(root / "Systems"),
        False,
    ):
        raise RuntimeError(
            "Temporary integration FDM failed to load"
        )

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        0.0,
    )

    fdm.set_property_value(
        "ic/h-agl-ft",
        4.30,
    )

    fdm.set_property_value(
        "ic/vg-kts",
        0.0,
    )

    if not fdm.run_ic():
        raise RuntimeError(
            "run_ic() failed"
        )

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

    fdm.set_property_value(
        "propulsion/engine[0]/ram-air-factor",
        0.0,
    )

    return fdm


def read_powerplant_overlay(path):
    root = ET.parse(path).getroot()

    node = root.find(
        "./overlay/fdm/jsbsim/"
        "systems/powerplant-controls"
    )

    if node is None:
        raise RuntimeError(
            f"No powerplant overlay in {path}"
        )

    values = {}

    def walk(element, prefix):
        children = list(element)

        if not children:
            raw = (
                element.text.strip()
                if element.text
                else ""
            )

            values[
                prefix + "/" + element.tag
            ] = float(raw)

            return

        for child in children:
            walk(
                child,
                prefix + "/" + element.tag,
            )

    for child in node:
        walk(
            child,
            "systems/powerplant-controls",
        )

    return values


def apply_state(fdm, state_name):
    values = read_powerplant_overlay(
        STATE_FILES[state_name]
    )

    for prop, value in values.items():
        fdm.set_property_value(
            prop,
            value,
        )


def assert_finite_controller(fdm, label):
    props = (
        LIMIT,
        TARGET,
        CONTROLLER_TARGET,
        VE,
        THROTTLE_CMD,
    )

    for prop in props:
        value = get(
            fdm,
            prop,
        )

        if not math.isfinite(value):
            raise RuntimeError(
                f"{label}: non-finite {prop}"
            )


def assert_default_on(fdm):
    for prop in (
        ENABLED,
        VE_ENABLED,
        CAPABILITY_ENABLED,
    ):
        value = get(
            fdm,
            prop,
        )

        if abs(value - 1.0) > 1e-9:
            raise RuntimeError(
                f"{prop} was not default-on"
            )


def check_pass_through(
    fdm,
    label,
    expected_pilot,
):
    run_frames(
        fdm,
        5,
    )

    pilot = get(
        fdm,
        PILOT,
    )

    limit = get(
        fdm,
        LIMIT,
    )

    command = get(
        fdm,
        THROTTLE_CMD,
    )

    expected = min(
        pilot,
        limit,
    )

    print(
        f"{label:<18}"
        f" pilot={pilot:.3f}"
        f" limit={limit:.4f}"
        f" cmd={command:.4f}"
        f" rpm={get(fdm, RPM):.1f}"
    )

    if abs(
        pilot - expected_pilot
    ) > 1e-6:
        raise RuntimeError(
            f"{label}: wrong pilot throttle"
        )

    if abs(
        command - expected
    ) > 1e-6:
        raise RuntimeError(
            f"{label}: throttle bridge mismatch"
        )

    # All present startup-state throttles should remain
    # below controller authority at sea-level static conditions.
    if abs(
        command - pilot
    ) > 1e-6:
        raise RuntimeError(
            f"{label}: controller unexpectedly "
            "limited startup-state pilot throttle"
        )

    assert_finite_controller(
        fdm,
        label,
    )


def start_engine(
    fdm,
    label,
    cranking_throttle,
):
    fdm.set_property_value(
        PILOT,
        cranking_throttle,
    )

    fdm.set_property_value(
        STARTER,
        1,
    )

    run_frames(
        fdm,
        5,
    )

    start_allowed = get(
        fdm,
        START_ALLOWED,
    )

    starter_cmd = get(
        fdm,
        STARTER_CMD,
    )

    command = get(
        fdm,
        THROTTLE_CMD,
    )

    print(
        f"{label:<18}"
        f" startAllowed={start_allowed:.0f}"
        f" starterCmd={starter_cmd:.0f}"
        f" throttleCmd={command:.4f}"
    )

    if start_allowed < 0.5:
        raise RuntimeError(
            f"{label}: start-allowed did not assert"
        )

    if starter_cmd < 0.5:
        raise RuntimeError(
            f"{label}: starter command did not assert"
        )

    if abs(
        command
        - cranking_throttle
    ) > 1e-5:
        raise RuntimeError(
            f"{label}: controller blocked "
            "cranking throttle"
        )

    # Mirror state-init.nas' ~8-second startup window.
    steps = int(
        8.0 / DT
    )

    peak_rpm = 0.0

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError(
                f"{label}: JSBSim stopped "
                "during start"
            )

        rpm = get(
            fdm,
            RPM,
        )

        peak_rpm = max(
            peak_rpm,
            rpm,
        )

        if rpm >= 700.0:
            break

    print(
        f"{label:<18}"
        f" startupRPM={get(fdm, RPM):.1f}"
        f" peakRPM={peak_rpm:.1f}"
    )

    if get(
        fdm,
        RPM,
    ) < 700.0:
        raise RuntimeError(
            f"{label}: engine failed to reach "
            "700 RPM within 8 seconds"
        )

    fdm.set_property_value(
        STARTER,
        0,
    )

    run_frames(
        fdm,
        5,
    )

    if get(
        fdm,
        STARTER_CMD,
    ) > 0.5:
        raise RuntimeError(
            f"{label}: starter command "
            "did not release"
        )


with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)

    prepare(
        root
    )

    print(
        "AF1B INTEGRATION STARTUP / STATE REGRESSION"
    )
    print(
        "==========================================="
    )
    print(
        f"JSBSim version: {jsbsim.__version__}"
    )
    print()

    # ------------------------------------------------------
    # PARKED
    # ------------------------------------------------------

    fdm = make_fdm(
        root
    )

    assert_default_on(
        fdm
    )

    apply_state(
        fdm,
        "parked",
    )

    check_pass_through(
        fdm,
        "parked",
        0.0,
    )

    run_for(
        fdm,
        1.0,
    )

    if get(
        fdm,
        RPM,
    ) > 100.0:
        raise RuntimeError(
            "parked: engine unexpectedly rotating"
        )

    print(
        "parked engine-off: PASS"
    )
    print()

    # ------------------------------------------------------
    # READY TO START
    # ------------------------------------------------------

    fdm = make_fdm(
        root
    )

    assert_default_on(
        fdm
    )

    apply_state(
        fdm,
        "ready-to-start",
    )

    check_pass_through(
        fdm,
        "ready-to-start",
        0.12,
    )

    run_for(
        fdm,
        1.0,
    )

    if get(
        fdm,
        RPM,
    ) > 100.0:
        raise RuntimeError(
            "ready-to-start: engine "
            "started without starter"
        )

    print(
        "ready-to-start engine-off: PASS"
    )
    print()

    # ------------------------------------------------------
    # TAKE-OFF
    #
    # Standalone JSBSim cannot execute FlightGear Nasal.
    # Emulate state-init.nas:
    #     throttle = .20
    #     starter = 1
    #     after stable running:
    #         starter = 0
    #         throttle = 0
    # ------------------------------------------------------

    fdm = make_fdm(
        root
    )

    assert_default_on(
        fdm
    )

    apply_state(
        fdm,
        "take-off",
    )

    start_engine(
        fdm,
        "take-off start",
        0.20,
    )

    fdm.set_property_value(
        PILOT,
        0.0,
    )

    check_pass_through(
        fdm,
        "take-off final",
        0.0,
    )

    print(
        "take-off startup: PASS"
    )
    print()

    # ------------------------------------------------------
    # APPROACH
    #
    # Emulate state-init.nas:
    #     throttle = .30 during start
    #     after running:
    #         starter = 0
    #         throttle = .425
    # ------------------------------------------------------

    fdm = make_fdm(
        root
    )

    assert_default_on(
        fdm
    )

    apply_state(
        fdm,
        "approach",
    )

    start_engine(
        fdm,
        "approach start",
        0.30,
    )

    fdm.set_property_value(
        PILOT,
        0.425,
    )

    fdm.set_property_value(
        MIXTURE,
        1.0,
    )

    fdm.set_property_value(
        PROP,
        1.0,
    )

    check_pass_through(
        fdm,
        "approach final",
        0.425,
    )

    print(
        "approach startup: PASS"
    )


print()
print("RESULT")
print("------")
print(
    "AF1B INTEGRATION STARTUP / STATE REGRESSION PASS"
)
print(
    "permanent aircraft files untouched"
)
