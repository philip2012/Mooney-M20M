#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

SYSTEM_XML = (
    REPO
    / "Tools"
    / "jsbsim"
    / "af1b-density-controller-stage2d.xml"
)

DT = 1.0 / 120.0

KTS_TO_FPS = 1.687809857

BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
VE_ENABLED = BASE + "/ve-enabled"

TARGET_MAP = BASE + "/target-map-inhg"
CONTROLLER_CAP = BASE + "/controller-cap-norm"
FEEDBACK_INTEGRAL = BASE + "/feedback-integral-norm"
FEEDBACK_GOVERNING = BASE + "/feedback-governing"
FEEDBACK_READY = BASE + "/feedback-ready"
FEEDBACK_DELAY = BASE + "/feedback-delay-sec"

PILOT_THROTTLE = (
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

RAM_AIR = "propulsion/engine[0]/ram-air-factor"

MAP = "propulsion/engine[0]/map-inhg"
RPM = "propulsion/engine[0]/propeller-rpm"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"

THROTTLE_COMMAND = "fcs/throttle-cmd-norm[0]"
THROTTLE_POSITION = "fcs/throttle-pos-norm[0]"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


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

        run_for(
            fdm,
            0.25,
        )


def ramp_pilot(
    fdm,
    start,
    end,
    seconds,
    collector=None,
):
    steps = int(seconds / DT)

    for step in range(steps):
        fraction = (
            (step + 1)
            / steps
        )

        throttle = (
            start
            + (end - start)
            * fraction
        )

        fdm.set_property_value(
            PILOT_THROTTLE,
            throttle,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "pilot-throttle ramp"
            )

        if collector is not None:
            collector.append(
                snapshot(
                    fdm,
                    throttle,
                )
            )


def snapshot(
    fdm,
    pilot=None,
):
    if pilot is None:
        pilot = get(
            fdm,
            PILOT_THROTTLE,
        )

    return {
        "time": fdm.get_sim_time(),
        "pilot": pilot,
        "cap": get(
            fdm,
            CONTROLLER_CAP,
        ),
        "integral": get(
            fdm,
            FEEDBACK_INTEGRAL,
        ),
        "governing": get(
            fdm,
            FEEDBACK_GOVERNING,
        ),
        "ready": get(
            fdm,
            FEEDBACK_READY,
        ),
        "delay": get(
            fdm,
            FEEDBACK_DELAY,
        ),
        "target": get(
            fdm,
            TARGET_MAP,
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
        "blade": get(
            fdm,
            BLADE,
        ),
        "command": get(
            fdm,
            THROTTLE_COMMAND,
        ),
        "position": get(
            fdm,
            THROTTLE_POSITION,
        ),
    }


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

    shutil.copy2(
        SYSTEM_XML,
        (
            root
            / "Systems"
            / "af1b-density-controller-stage2d.xml"
        ),
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
        'file="af1b-density-controller-stage2d" />\n'
    )

    if text.count(old) != 1:
        raise RuntimeError(
            "Expected powerplant-controls "
            "include exactly once"
        )

    fdm_path.write_text(
        text.replace(
            old,
            new,
            1,
        )
    )

    return root


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
        RAM_AIR,
        0.0,
    )

    fdm.set_property_value(
        ENABLED,
        0,
    )

    fdm.set_property_value(
        VE_ENABLED,
        0,
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
        PILOT_THROTTLE,
        0.55,
    )

    fdm.set_property_value(
        MAGNETOS,
        3,
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
        1.5,
    )

    if get(fdm, RPM) < 1000.0:
        raise RuntimeError(
            "engine did not establish"
        )

    ramp_inflow(
        fdm
    )

    run_for(
        fdm,
        1.0,
    )

    # Precondition temporary full-power VE compensation.
    fdm.set_property_value(
        VE_ENABLED,
        1,
    )

    run_for(
        fdm,
        2.0,
    )

    # Enable controller, then make the same 1-second
    # pilot movement used by the dynamic qualification.
    fdm.set_property_value(
        ENABLED,
        1,
    )

    ramp_pilot(
        fdm,
        0.55,
        1.0,
        1.0,
    )

    # Allow feedback and engine/propeller state to settle.
    run_for(
        fdm,
        14.0,
    )

    return fdm


print(
    "MOONEY AF1B STAGE-2D "
    "AUTHORITY-BOUNDARY TEST"
)
print(
    "=========================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print()

with tempfile.TemporaryDirectory() as tmp:
    root = prepare_test_tree(
        Path(tmp)
    )

    fdm = make_fdm(
        root
    )

    governed = snapshot(
        fdm
    )

    print(
        "INITIAL GOVERNED CONDITION"
    )
    print(
        "--------------------------"
    )
    print(
        f"pilot={governed['pilot']:.4f}  "
        f"cap={governed['cap']:.4f}  "
        f"governing={governed['governing']:.0f}  "
        f"ready={governed['ready']:.0f}"
    )
    print(
        f"target={governed['target']:.2f}\"  "
        f"MAP={governed['map']:.2f}\"  "
        f"RPM={governed['rpm']:.1f}  "
        f"HP={governed['hp']:.2f}  "
        f"I={governed['integral']:+.6f}"
    )
    print()

    if governed["governing"] < 0.5:
        raise SystemExit(
            "FAIL: controller not governing "
            "before pilot pullback"
        )

    # ----------------------------------------------------------
    # Pilot deliberately removes authority from density control.
    # ----------------------------------------------------------

    pullback_samples = []

    ramp_pilot(
        fdm,
        1.0,
        0.55,
        1.0,
        pullback_samples,
    )

    # Begin the freeze measurement only once the pilot has
    # completed the pullback and is definitely below the cap.
    hold_start = snapshot(
        fdm
    )

    hold_integral_start = (
        hold_start["integral"]
    )

    hold_cap_start = (
        hold_start["cap"]
    )

    hold_samples = []

    steps = int(
        5.0 / DT
    )

    for _ in range(steps):
        fdm.set_property_value(
            PILOT_THROTTLE,
            0.55,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "manual-throttle hold"
            )

        hold_samples.append(
            snapshot(
                fdm,
                0.55,
            )
        )

    hold_end = hold_samples[-1]

    integral_drift = (
        hold_end["integral"]
        - hold_integral_start
    )

    cap_drift = (
        hold_end["cap"]
        - hold_cap_start
    )

    minimum_map = min(
        s["map"]
        for s in hold_samples
    )

    governing_max = max(
        s["governing"]
        for s in hold_samples
    )

    print(
        "PILOT BELOW CONTROLLER CAP"
    )
    print(
        "--------------------------"
    )
    print(
        f"pilot={hold_end['pilot']:.4f}  "
        f"cap={hold_end['cap']:.4f}  "
        f"governing={hold_end['governing']:.0f}"
    )
    print(
        f"MAP={hold_end['map']:.2f}\"  "
        f"minimumMAP={minimum_map:.2f}\""
    )
    print(
        f"I start={hold_integral_start:+.8f}"
    )
    print(
        f"I end=  {hold_end['integral']:+.8f}"
    )
    print(
        f"I drift={integral_drift:+.10f}"
    )
    print(
        f"cap drift={cap_drift:+.10f}"
    )

    authority_ok = (
        governing_max < 0.5
        and hold_end["ready"] < 0.5
        and hold_end["delay"] < 0.01
        and abs(
            integral_drift
        ) < 1e-8
        and abs(
            cap_drift
        ) < 1e-6
        and minimum_map
        < governed["target"] - 3.0
    )

    print(
        "authority freeze: "
        + (
            "PASS"
            if authority_ok
            else "FAIL"
        )
    )
    print()

    # ----------------------------------------------------------
    # Return pilot authority to the controller.
    #
    # Do NOT impose transient pass/fail limits yet. We want to
    # observe whether immediate feedback re-entry creates a kick.
    # ----------------------------------------------------------

    recovery_samples = []

    governing_reentry_time = None
    feedback_reentry_time = None

    recovery_start_time = (
        fdm.get_sim_time()
    )

    steps = int(
        1.0 / DT
    )

    for step in range(steps):
        fraction = (
            (step + 1)
            / steps
        )

        pilot = (
            0.55
            + 0.45
            * fraction
        )

        fdm.set_property_value(
            PILOT_THROTTLE,
            pilot,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "authority reacquisition"
            )

        sample = snapshot(
            fdm,
            pilot,
        )

        recovery_samples.append(
            sample
        )

        if (
            governing_reentry_time is None
            and sample["governing"] > 0.5
        ):
            governing_reentry_time = (
                fdm.get_sim_time()
                - recovery_start_time
            )

        if (
            feedback_reentry_time is None
            and sample["ready"] > 0.5
        ):
            feedback_reentry_time = (
                fdm.get_sim_time()
                - recovery_start_time
            )

    # Continue after pilot has reached full throttle.
    steps = int(
        5.0 / DT
    )

    for _ in range(steps):
        fdm.set_property_value(
            PILOT_THROTTLE,
            1.0,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "recovery observation"
            )

        sample = snapshot(
            fdm,
            1.0,
        )

        recovery_samples.append(
            sample
        )

        if (
            feedback_reentry_time is None
            and sample["ready"] > 0.5
        ):
            feedback_reentry_time = (
                fdm.get_sim_time()
                - recovery_start_time
            )

    recovered = recovery_samples[-1]

    max_map = max(
        s["map"]
        for s in recovery_samples
    )

    max_rpm = max(
        s["rpm"]
        for s in recovery_samples
    )

    max_hp = max(
        s["hp"]
        for s in recovery_samples
    )

    max_integral = max(
        s["integral"]
        for s in recovery_samples
    )

    min_integral = min(
        s["integral"]
        for s in recovery_samples
    )

    print(
        "AUTHORITY REACQUISITION"
    )
    print(
        "-----------------------"
    )
    print(
        "governing re-entry: "
        + (
            f"{governing_reentry_time:.3f}s"
            if governing_reentry_time
            is not None
            else "NEVER"
        )
    )
    print(
        "feedback re-entry: "
        + (
            f"{feedback_reentry_time:.3f}s"
            if feedback_reentry_time
            is not None
            else "NEVER"
        )
    )

    print(
        "authority-to-feedback delay: "
        + (
            f"{feedback_reentry_time - governing_reentry_time:.3f}s"
            if (
                feedback_reentry_time is not None
                and governing_reentry_time is not None
            )
            else "N/A"
        )
    )
    print(
        f"peak MAP={max_map:.2f}\""
    )
    print(
        f"peak RPM={max_rpm:.1f}"
    )
    print(
        f"peak HP={max_hp:.1f}"
    )
    print(
        f"integral range="
        f"{min_integral:+.6f} "
        f"to {max_integral:+.6f}"
    )
    print(
        f"recovered MAP="
        f"{recovered['map']:.2f}\"  "
        f"target="
        f"{recovered['target']:.2f}\""
    )
    print(
        f"recovered RPM="
        f"{recovered['rpm']:.1f}  "
        f"HP={recovered['hp']:.2f}  "
        f"cap={recovered['cap']:.4f}"
    )
    print()

    reentry_delay_ok = (
        governing_reentry_time is not None
        and feedback_reentry_time is not None
        and abs(
            (
                feedback_reentry_time
                - governing_reentry_time
            )
            - 1.5
        ) < 0.03
    )

    recovery_ok = (
        reentry_delay_ok
        and abs(
            recovered["map"]
            - recovered["target"]
        ) < 0.10
        and abs(
            recovered["rpm"]
            - 2575.0
        ) < 10.0
        and abs(
            recovered["hp"]
            - 270.0
        ) < 2.0
    )

    print(
        "RESULT"
    )
    print(
        "------"
    )
    print(
        "pilot-below-cap freeze: "
        + (
            "PASS"
            if authority_ok
            else "FAIL"
        )
    )
    print(
        "eventual recovery:       "
        + (
            "PASS"
            if recovery_ok
            else "FAIL"
        )
    )

    if not (
        authority_ok
        and recovery_ok
    ):
        raise SystemExit(
            "AF1B STAGE-2D "
            "AUTHORITY TEST FAIL"
        )

print()
print(
    "AF1B STAGE-2D "
    "AUTHORITY TEST PASS"
)
