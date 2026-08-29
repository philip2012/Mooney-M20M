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
    / "af1b-density-controller-stage2e.xml"
)

DT = 1.0 / 120.0

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

PILOT_RAMP_SEC = 1.0
DURATION = 15.0

TEST_CASES = (
    ("SL ISA", 0.0, 0.0),
    ("10K ISA-15", 10000.0, -15.0),
    ("10K ISA+15", 10000.0, 15.0),
    ("20K ISA-15", 20000.0, -15.0),
    ("20K ISA", 20000.0, 0.0),
    ("20K ISA+15", 20000.0, 15.0),
)

BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
VE_ENABLED = BASE + "/ve-enabled"

TARGET_MAP = BASE + "/target-map-inhg"
FEEDFORWARD = BASE + "/feedforward-cap-norm"
RAW_CAP = BASE + "/raw-controller-cap-norm"
CONTROLLER_CAP = BASE + "/controller-cap-norm"

FEEDBACK_DELAY = BASE + "/feedback-delay-sec"
FEEDBACK_READY = BASE + "/feedback-ready"
FEEDBACK_GOVERNING = BASE + "/feedback-governing"
FEEDBACK_INTEGRAL = BASE + "/feedback-integral-norm"

VE_PROP = (
    "propulsion/engine[0]/volumetric-efficiency"
)

RAM_AIR = (
    "propulsion/engine[0]/ram-air-factor"
)

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"
ADVANCE_RATIO = (
    "propulsion/engine[0]/advance-ratio"
)

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

THROTTLE_COMMAND = (
    "fcs/throttle-cmd-norm[0]"
)

THROTTLE_POSITION = (
    "fcs/throttle-pos-norm[0]"
)


def get(fdm, prop):
    return fdm.get_property_value(prop)


def clamp(value, low, high):
    return max(
        low,
        min(
            high,
            value,
        ),
    )


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
            / "af1b-density-controller-stage2e.xml"
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
        'file="af1b-density-controller-stage2e" />\n'
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


def make_fdm(
    test_root,
    altitude_ft,
    temp_bias_f,
):
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
        "atmosphere/delta-T",
        temp_bias_f,
    )

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        altitude_ft,
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

    # Diagnostic isolation:
    #
    # Precondition the full-power FGPiston compensation while
    # throttle control itself remains disabled. Pilot throttle
    # therefore remains authoritative at 0.55.
    fdm.set_property_value(
        VE_ENABLED,
        1,
    )

    run_for(
        fdm,
        2.0,
    )

    return fdm


def run_case(
    test_root,
    name,
    altitude_ft,
    temp_bias_f,
):
    fdm = make_fdm(
        test_root,
        altitude_ft,
        temp_bias_f,
    )

    pre_feedforward = get(
        fdm,
        FEEDFORWARD,
    )

    pre_cap = get(
        fdm,
        CONTROLLER_CAP,
    )

    preposition_error = abs(
        pre_cap
        - pre_feedforward
    )

    # This moment corresponds to t=0 in the original
    # Python dynamic-controller prototype.
    fdm.set_property_value(
        ENABLED,
        1,
    )

    start_time = (
        fdm.get_sim_time()
    )

    samples = []

    governing_time = None
    feedback_ready_time = None

    previous_cap = pre_cap
    max_cap_rate = 0.0

    steps = int(
        DURATION / DT
    )

    for step in range(steps):
        time_s = (
            step * DT
        )

        ramp_fraction = clamp(
            time_s
            / PILOT_RAMP_SEC,
            0.0,
            1.0,
        )

        pilot_throttle = (
            0.55
            + 0.45
            * ramp_fraction
        )

        fdm.set_property_value(
            PILOT_THROTTLE,
            pilot_throttle,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "Stage-2E transient"
            )

        elapsed = (
            fdm.get_sim_time()
            - start_time
        )

        cap = get(
            fdm,
            CONTROLLER_CAP,
        )

        cap_rate = abs(
            (
                cap
                - previous_cap
            )
            / DT
        )

        max_cap_rate = max(
            max_cap_rate,
            cap_rate,
        )

        previous_cap = cap

        governing = get(
            fdm,
            FEEDBACK_GOVERNING,
        )

        ready = get(
            fdm,
            FEEDBACK_READY,
        )

        if (
            governing_time is None
            and governing > 0.5
        ):
            governing_time = (
                elapsed
            )

        if (
            feedback_ready_time is None
            and ready > 0.5
        ):
            feedback_ready_time = (
                elapsed
            )

        samples.append(
            {
                "time": elapsed,
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
                "j": get(
                    fdm,
                    ADVANCE_RATIO,
                ),
                "pilot": pilot_throttle,
                "command": get(
                    fdm,
                    THROTTLE_COMMAND,
                ),
                "position": get(
                    fdm,
                    THROTTLE_POSITION,
                ),
                "feedforward": get(
                    fdm,
                    FEEDFORWARD,
                ),
                "raw_cap": get(
                    fdm,
                    RAW_CAP,
                ),
                "cap": cap,
                "integral": get(
                    fdm,
                    FEEDBACK_INTEGRAL,
                ),
                "delay": get(
                    fdm,
                    FEEDBACK_DELAY,
                ),
                "ve": get(
                    fdm,
                    VE_PROP,
                ),
            }
        )

    settled = [
        r
        for r in samples
        if r["time"]
        >= DURATION - 2.0
    ]

    def average(key):
        return (
            sum(
                r[key]
                for r in settled
            )
            / len(settled)
        )

    avg_target = average(
        "target"
    )

    avg_map = average(
        "map"
    )

    avg_rpm = average(
        "rpm"
    )

    avg_hp = average(
        "hp"
    )

    avg_blade = average(
        "blade"
    )

    avg_j = average(
        "j"
    )

    avg_cap = average(
        "cap"
    )

    avg_integral = average(
        "integral"
    )

    max_map = max(
        r["map"]
        for r in samples
    )

    peak_rpm = max(
        samples,
        key=lambda r: r["rpm"],
    )

    max_hp = max(
        r["hp"]
        for r in samples
    )

    reasons = []

    if preposition_error > 0.002:
        reasons.append(
            "PREPOSITION"
        )

    authority_feedback_delay = None

    if (
        governing_time is not None
        and feedback_ready_time is not None
    ):
        authority_feedback_delay = (
            feedback_ready_time
            - governing_time
        )

    if (
        authority_feedback_delay is None
        or abs(
            authority_feedback_delay
            - 1.5
        ) > 0.03
    ):
        reasons.append(
            "DELAY"
        )

    if max_cap_rate > 0.351:
        reasons.append(
            "CAP RATE"
        )

    if abs(
        avg_map
        - avg_target
    ) > 0.10:
        reasons.append(
            "MAP"
        )

    if abs(
        avg_rpm
        - 2575.0
    ) > 10.0:
        reasons.append(
            "RPM"
        )

    if abs(
        avg_hp
        - 270.0
    ) > 2.0:
        reasons.append(
            "POWER"
        )

    if avg_blade >= 44.45:
        reasons.append(
            "PROP LIMIT"
        )

    return {
        "name": name,
        "alt": altitude_ft,
        "bias": temp_bias_f,
        "target": avg_target,
        "map": avg_map,
        "rpm": avg_rpm,
        "hp": avg_hp,
        "blade": avg_blade,
        "j": avg_j,
        "feedforward": pre_feedforward,
        "pre_cap": pre_cap,
        "cap": avg_cap,
        "integral": avg_integral,
        "governing_time": governing_time,
        "ready_time": feedback_ready_time,
        "authority_feedback_delay": (
            authority_feedback_delay
        ),
        "max_cap_rate": max_cap_rate,
        "max_map": max_map,
        "max_rpm": peak_rpm["rpm"],
        "peak_rpm_time": peak_rpm["time"],
        "max_hp": max_hp,
        "status": (
            "PASS"
            if not reasons
            else ",".join(reasons)
        ),
    }


print(
    "MOONEY AF1B DENSITY CONTROLLER "
    "STAGE-2E DYNAMIC XML TEST"
)
print(
    "=============================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "pilot ramp:      "
    f"{PILOT_RAMP_SEC:.2f} s"
)
print(
    "feedback delay:  1.50 s"
)
print(
    "integral gain:   0.020"
)
print(
    "cap rate limit:  0.35/s"
)
print()

all_pass = True

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    for case in TEST_CASES:
        result = run_case(
            test_root,
            *case,
        )

        if result["status"] != "PASS":
            all_pass = False

        print(
            f"{result['name']:<11} "
            f"target={result['target']:6.2f}\"  "
            f"MAP={result['map']:6.2f}\"  "
            f"err="
            f"{result['map'] - result['target']:+.3f}\"  "
            f"RPM={result['rpm']:7.1f}  "
            f"HP={result['hp']:6.2f}"
        )

        print(
            f"    FF={result['feedforward']:.4f}  "
            f"preCap={result['pre_cap']:.4f}  "
            f"finalCap={result['cap']:.4f}  "
            f"I={result['integral']:+.5f}  "
            f"blade={result['blade']:.2f}  "
            f"J={result['j']:.3f}"
        )

        print(
            f"    governingAt="
            f"{result['governing_time']:.3f}s  "
            f"feedbackReady="
            f"{result['ready_time']:.3f}s  "
            f"authorityDelay="
            f"{result['authority_feedback_delay']:.3f}s  "
            f"maxCapRate="
            f"{result['max_cap_rate']:.3f}/s  "
            f"maxMAP={result['max_map']:.2f}\"  "
            f"maxRPM={result['max_rpm']:.1f}"
            f"@{result['peak_rpm_time']:.3f}s  "
            f"maxHP={result['max_hp']:.1f}"
        )

        print(
            f"    status: "
            f"{result['status']}"
        )

        print()


# ==============================================================
# Stage-2E edge qualification
# ==============================================================

ANTI_UPPER = (
    BASE
    + "/antiwindup-upper"
)

INTEGRATOR_TRIGGER = (
    BASE
    + "/feedback-integrator-trigger"
)

PRE_REQUEST = (
    BASE
    + "/pre-integrator-controller-request-norm"
)

MAP_ERROR = (
    BASE
    + "/map-error-inhg"
)

INTEGRAL_INITIALIZER = (
    FEEDBACK_INTEGRAL
    + "/initial-integrator-value"
)


def wait_for_true(
    fdm,
    prop,
    timeout,
):
    end = (
        fdm.get_sim_time()
        + timeout
    )

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "edge-test wait"
            )

        if get(
            fdm,
            prop,
        ) > 0.5:
            return True

    return False


def upper_saturation_freeze_probe(
    root,
):
    fdm = make_fdm(
        root,
        30000.0,
        0.0,
    )

    fdm.set_property_value(
        PILOT_THROTTLE,
        1.0,
    )

    fdm.set_property_value(
        ENABLED,
        1,
    )

    ready = wait_for_true(
        fdm,
        FEEDBACK_READY,
        5.0,
    )

    upper = wait_for_true(
        fdm,
        ANTI_UPPER,
        2.0,
    )

    if not (
        ready
        and upper
    ):
        print(
            "UPPER SATURATION FREEZE: "
            "FAIL - condition not reached"
        )

        return False

    integral_start = get(
        fdm,
        FEEDBACK_INTEGRAL,
    )

    min_error = float("inf")
    min_pre_request = float("inf")
    min_upper = float("inf")
    min_trigger = float("inf")

    steps = int(
        2.0 / DT
    )

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "upper saturation hold"
            )

        min_error = min(
            min_error,
            get(
                fdm,
                MAP_ERROR,
            ),
        )

        min_pre_request = min(
            min_pre_request,
            get(
                fdm,
                PRE_REQUEST,
            ),
        )

        min_upper = min(
            min_upper,
            get(
                fdm,
                ANTI_UPPER,
            ),
        )

        min_trigger = min(
            min_trigger,
            get(
                fdm,
                INTEGRATOR_TRIGGER,
            ),
        )

    integral_end = get(
        fdm,
        FEEDBACK_INTEGRAL,
    )

    integral_drift = (
        integral_end
        - integral_start
    )

    passed = (
        min_error > 0.05
        and min_pre_request >= 0.999
        and min_upper > 0.5
        and min_trigger > 0.5
        and abs(
            integral_drift
        ) < 1e-8
    )

    print()
    print(
        "UPPER SATURATION FREEZE"
    )
    print(
        "-----------------------"
    )
    print(
        f"target="
        f"{get(fdm, TARGET_MAP):.2f}\"  "
        f"MAP="
        f"{get(fdm, MAP):.2f}\""
    )
    print(
        f"minimum error="
        f"{min_error:+.4f}\""
    )
    print(
        f"minimum request="
        f"{min_pre_request:.6f}"
    )
    print(
        f"antiwindup-upper="
        f"{min_upper:.0f}  "
        f"trigger="
        f"{min_trigger:.0f}"
    )
    print(
        f"integral start="
        f"{integral_start:+.10f}"
    )
    print(
        f"integral end=  "
        f"{integral_end:+.10f}"
    )
    print(
        f"integral drift="
        f"{integral_drift:+.12f}"
    )
    print(
        "upper freeze: "
        + (
            "PASS"
            if passed
            else "FAIL"
        )
    )

    return passed


def upper_error_reversal_probe(
    root,
):
    fdm = make_fdm(
        root,
        20000.0,
        0.0,
    )

    fdm.set_property_value(
        PILOT_THROTTLE,
        1.0,
    )

    fdm.set_property_value(
        ENABLED,
        1,
    )

    if not wait_for_true(
        fdm,
        FEEDBACK_READY,
        5.0,
    ):
        print(
            "UPPER ERROR REVERSAL: "
            "FAIL - feedback never ready"
        )

        return False

    run_for(
        fdm,
        4.0,
    )

    baseline_cap = get(
        fdm,
        CONTROLLER_CAP,
    )

    # Source-supported FGPID/integrator setter.
    # This is a diagnostic state injection only.
    fdm.set_property_value(
        INTEGRAL_INITIALIZER,
        0.25,
    )

    # SetInitialOutput() changes the integrator's internal state,
    # but the bound component-output property is not propagated
    # until the integrator executes on the next FCS frame.
    stale_integral_output = get(
        fdm,
        FEEDBACK_INTEGRAL,
    )

    previous_cap = baseline_cap

    if not fdm.run():
        raise RuntimeError(
            "JSBSim stopped while publishing "
            "forced integrator state"
        )

    forced_integral = get(
        fdm,
        FEEDBACK_INTEGRAL,
    )

    cap = get(
        fdm,
        CONTROLLER_CAP,
    )

    max_cap_rate = abs(
        (
            cap
            - previous_cap
        )
        / DT
    )

    previous_cap = cap

    saw_inward_unwind = False

    steps = int(
        2.0 / DT
    )

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "upper reversal probe"
            )

        cap = get(
            fdm,
            CONTROLLER_CAP,
        )

        cap_rate = abs(
            (
                cap
                - previous_cap
            )
            / DT
        )

        max_cap_rate = max(
            max_cap_rate,
            cap_rate,
        )

        previous_cap = cap

        pre_request = get(
            fdm,
            PRE_REQUEST,
        )

        error = get(
            fdm,
            MAP_ERROR,
        )

        anti_upper = get(
            fdm,
            ANTI_UPPER,
        )

        trigger = get(
            fdm,
            INTEGRATOR_TRIGGER,
        )

        if (
            pre_request > 0.999
            and error < -0.02
            and anti_upper < 0.5
            and abs(
                trigger
            ) < 0.5
        ):
            saw_inward_unwind = True

    integral_end = get(
        fdm,
        FEEDBACK_INTEGRAL,
    )

    passed = (
        abs(
            forced_integral
            - 0.25
        ) < 0.001
        and saw_inward_unwind
        and max_cap_rate > 0.30
        and max_cap_rate <= 0.351
        and integral_end
        < forced_integral - 0.005
    )

    print()
    print(
        "UPPER SATURATION ERROR REVERSAL"
    )
    print(
        "-------------------------------"
    )
    print(
        f"stale output immediately after setter="
        f"{stale_integral_output:+.8f}"
    )
    print(
        f"baseline cap="
        f"{baseline_cap:.6f}"
    )
    print(
        f"forced integral="
        f"{forced_integral:+.8f}"
    )
    print(
        f"final integral="
        f"{integral_end:+.8f}"
    )
    print(
        f"max cap rate="
        f"{max_cap_rate:.6f}/s"
    )
    print(
        "inward unwind observed: "
        + (
            "YES"
            if saw_inward_unwind
            else "NO"
        )
    )
    print(
        "upper reversal + rate limit: "
        + (
            "PASS"
            if passed
            else "FAIL"
        )
    )

    return passed


edge_pass = True

with tempfile.TemporaryDirectory() as tmp:
    edge_root = prepare_test_tree(
        Path(tmp)
    )

    if not upper_saturation_freeze_probe(
        edge_root
    ):
        edge_pass = False


with tempfile.TemporaryDirectory() as tmp:
    edge_root = prepare_test_tree(
        Path(tmp)
    )

    if not upper_error_reversal_probe(
        edge_root
    ):
        edge_pass = False


if not edge_pass:
    all_pass = False


print(
    "RESULT"
)
print(
    "------"
)

if all_pass:
    print(
        "AF1B DENSITY CONTROLLER "
        "STAGE-2E DYNAMIC XML TEST PASS"
    )
else:
    raise SystemExit(
        "AF1B DENSITY CONTROLLER "
        "STAGE-2E DYNAMIC XML TEST FAIL"
    )
