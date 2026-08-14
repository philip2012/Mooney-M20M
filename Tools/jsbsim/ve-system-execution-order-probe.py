#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

HIGH_VE = 0.90
LOW_VE = 0.65

VE_COMMAND = (
    "systems/ve-order-probe/command"
)

VE_PROP = (
    "propulsion/engine[0]/volumetric-efficiency"
)

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"

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


PROBE_SYSTEM_XML = """\
<?xml version="1.0" encoding="UTF-8"?>

<system name="ve-execution-order-probe">
    <property value="0.90">
        systems/ve-order-probe/command
    </property>

    <channel name="ve-writer">
        <pure_gain name="systems/ve-order-probe/writer">
            <input>systems/ve-order-probe/command</input>
            <gain>1.0</gain>
            <output>
                propulsion/engine[0]/volumetric-efficiency
            </output>
        </pure_gain>
    </channel>
</system>
"""


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = (
        fdm.get_sim_time()
        + seconds
    )

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


def prepare_test_tree(root):
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

    probe_path = (
        root
        / "Systems"
        / "ve-execution-order-probe.xml"
    )

    probe_path.write_text(
        PROBE_SYSTEM_XML
    )

    fdm_path = (
        root
        / "FDM"
        / "Mooney-M20M.xml"
    )

    text = fdm_path.read_text()

    old = (
        '    <system file="powerplant-controls" />\n'
    )

    new = (
        old
        + '    <system '
        'file="ve-execution-order-probe" />\n'
    )

    if text.count(old) != 1:
        raise RuntimeError(
            "Expected powerplant-controls "
            "system include exactly once"
        )

    fdm_path.write_text(
        text.replace(
            old,
            new,
            1,
        )
    )

    return root


def make_fdm(test_root):
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(test_root),
        str(test_root / "Engines"),
        str(test_root / "Systems"),
        False,
    ):
        raise RuntimeError(
            "could not load temporary Mooney FDM"
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
        "ic/phi-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/theta-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/psi-true-deg",
        0.0,
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
        BATTERY,
        1,
    )

    fdm.set_property_value(
        MIXTURE,
        1.0,
    )

    fdm.set_property_value(
        PROP,
        1.0,
    )

    fdm.set_property_value(
        THROTTLE,
        0.55,
    )

    fdm.set_property_value(
        MAGNETOS,
        3,
    )

    fdm.set_property_value(
        VE_COMMAND,
        HIGH_VE,
    )

    run_for(
        fdm,
        0.1,
    )

    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    run_for(
        fdm,
        5.0,
    )

    if get(fdm, RPM) < 500.0:
        raise RuntimeError(
            "engine did not establish"
        )

    return fdm


def snapshot(fdm, label):
    return {
        "label": label,
        "frame": int(
            get(
                fdm,
                "simulation/frame",
            )
        ),
        "time": fdm.get_sim_time(),
        "command": get(
            fdm,
            VE_COMMAND,
        ),
        "ve": get(
            fdm,
            VE_PROP,
        ),
        "map": get(
            fdm,
            MAP,
        ),
        "rpm": get(
            fdm,
            RPM,
        ),
        "hp": get(
            fdm,
            POWER,
        ),
    }


def show(r):
    print(
        f"{r['label']:<18} "
        f"frame={r['frame']:6d}  "
        f"t={r['time']:8.4f}  "
        f"cmd={r['command']:.3f}  "
        f"VE={r['ve']:.5f}  "
        f"MAP={r['map']:6.2f}  "
        f"RPM={r['rpm']:7.1f}  "
        f"HP={r['hp']:8.3f}"
    )


print(
    "MOONEY VE SYSTEM EXECUTION-ORDER PROBE"
)
print(
    "======================================"
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    f"simulation rate: {1.0 / DT:.0f} Hz"
)
print()

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    fdm = make_fdm(
        test_root
    )

    before = snapshot(
        fdm,
        "before change",
    )

    fdm.set_property_value(
        VE_COMMAND,
        LOW_VE,
    )

    if not fdm.run():
        raise RuntimeError(
            "JSBSim stopped on low-VE frame"
        )

    low_frame_1 = snapshot(
        fdm,
        "low VE frame 1",
    )

    if not fdm.run():
        raise RuntimeError(
            "JSBSim stopped on second low-VE frame"
        )

    low_frame_2 = snapshot(
        fdm,
        "low VE frame 2",
    )

    run_for(
        fdm,
        0.5,
    )

    low_settled = snapshot(
        fdm,
        "low VE +0.5s",
    )

    fdm.set_property_value(
        VE_COMMAND,
        HIGH_VE,
    )

    if not fdm.run():
        raise RuntimeError(
            "JSBSim stopped on restore frame"
        )

    restore_frame_1 = snapshot(
        fdm,
        "restore frame 1",
    )

    print(
        " label              frame       time"
        "     cmd       VE     MAP"
        "      RPM        HP"
    )
    print(
        " ------------------ ------ ----------"
        " ------- -------- -------"
        " -------- ---------"
    )

    for result in (
        before,
        low_frame_1,
        low_frame_2,
        low_settled,
        restore_frame_1,
    ):
        show(result)

    print()

    property_same_frame = (
        abs(
            low_frame_1["ve"]
            - LOW_VE
        )
        < 1e-9
    )

    restore_same_frame = (
        abs(
            restore_frame_1["ve"]
            - HIGH_VE
        )
        < 1e-9
    )

    hp_changed_first_frame = (
        abs(
            low_frame_1["hp"]
            - before["hp"]
        )
        > 0.01
    )

    print(
        "RESULT"
    )
    print(
        "------"
    )
    print(
        "VE write visible on first frame: "
        + (
            "PASS"
            if property_same_frame
            else "FAIL"
        )
    )
    print(
        "HP responds on first frame:       "
        + (
            "PASS"
            if hp_changed_first_frame
            else "FAIL"
        )
    )
    print(
        "VE restore visible first frame:   "
        + (
            "PASS"
            if restore_same_frame
            else "FAIL"
        )
    )

    if not (
        property_same_frame
        and hp_changed_first_frame
        and restore_same_frame
    ):
        raise SystemExit(
            "VE SYSTEM EXECUTION-ORDER PROBE FAIL"
        )

print()
print(
    "VE SYSTEM EXECUTION-ORDER PROBE PASS"
)
