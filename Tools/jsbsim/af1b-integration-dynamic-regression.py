#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim

from af1b_lycoming_reference import (
    figure_3_34_post_critical_map,
)


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"

SYSTEM_XML = (
    REPO
    / "Tools"
    / "jsbsim"
    / "af1b-density-controller-integration-candidate.xml"
)

POWERPLANT_XML = (
    REPO
    / "Tools"
    / "jsbsim"
    / "powerplant-controls-integration-candidate.xml"
)

DT = 1.0 / 120.0

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

PSF_PER_INHG = 70.72620474785911

PILOT_RAMP_SEC = 1.0
DURATION = 15.0

TEST_CASES = (
    ("20K ISA-15", 20000.0, -15.0),
    ("20K ISA",    20000.0,   0.0),
    ("20K ISA+15", 20000.0,  15.0),
    ("22K ISA-15", 22000.0, -15.0),
    ("22K ISA",    22000.0,   0.0),
    ("22K ISA+15", 22000.0,  15.0),
    ("24K ISA",    24000.0,   0.0),
)

BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
VE_ENABLED = BASE + "/ve-enabled"

CAPABILITY_ENABLED = (
    BASE + "/capability-enabled"
)

OPERATIONAL_ACTIVE = (
    BASE + "/operational-active"
)

THROTTLE_LIMIT = (
    BASE + "/throttle-limit-norm"
)

MAXIMUM_MAP = (
    BASE + "/maximum-map-inhg"
)

CONTROLLER_TARGET = (
    BASE + "/controller-target-map-inhg"
)

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
            / "af1b-density-controller.xml"
        ),
    )

    shutil.copy2(
        POWERPLANT_XML,
        (
            root
            / "Systems"
            / "powerplant-controls.xml"
        ),
    )

    fdm_path = (
        root
        / "FDM"
        / "Mooney-M20M.xml"
    )

    text = fdm_path.read_text()

    old = (
        '    <system file="airframe-controls" />\n'
        '    <system file="powerplant-controls" />\n'
    )

    new = (
        '    <system file="airframe-controls" />\n'
        '    <system file="af1b-density-controller" />\n'
        '    <system file="powerplant-controls" />\n'
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


def ambient_pressure_inhg(
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
            "could not load atmosphere probe"
        )

    fdm.set_property_value(
        "atmosphere/delta-T",
        temp_bias_f,
    )

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        altitude_ft - 4.30,
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
            "atmosphere probe run_ic() failed"
        )

    return (
        get(
            fdm,
            "atmosphere/P-psf",
        )
        / PSF_PER_INHG
    )


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
        altitude_ft - 4.30,
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
        CAPABILITY_ENABLED,
        1,
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
    isa_pressure = ambient_pressure_inhg(
        test_root,
        altitude_ft,
        0.0,
    )

    current_pressure = ambient_pressure_inhg(
        test_root,
        altitude_ft,
        temp_bias_f,
    )

    isa_capability = (
        figure_3_34_post_critical_map(
            altitude_ft
        )
    )

    external_capability = clamp(
        isa_capability
        * current_pressure
        / isa_pressure,
        0.0,
        38.0,
    )

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
                "integration-bridge transient"
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
                    CONTROLLER_TARGET,
                ),
                "density_target": get(
                    fdm,
                    TARGET_MAP,
                ),
                "capability": get(
                    fdm,
                    MAXIMUM_MAP,
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
                "limit": get(
                    fdm,
                    THROTTLE_LIMIT,
                ),
                "operational": get(
                    fdm,
                    OPERATIONAL_ACTIVE,
                ),
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


    avg_density_target = average(
        "density_target"
    )

    avg_capability = average(
        "capability"
    )


    capability_parity_error = (
        avg_capability
        - external_capability
    )

    owner = (
        "DENSITY"
        if avg_density_target
        <= avg_capability
        else "CAPABILITY"
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

    max_bridge_error = max(
        abs(
            r["command"]
            - min(
                r["pilot"],
                r["limit"],
            )
        )
        for r in samples
    )

    min_operational = min(
        r["operational"]
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

    if avg_blade >= 44.45:
        reasons.append(
            "PROP LIMIT"
        )

    if abs(
        capability_parity_error
    ) > 0.01:
        reasons.append(
            "CAPABILITY PARITY"
        )

    if max_bridge_error > 1e-6:
        reasons.append(
            "THROTTLE BRIDGE"
        )

    if min_operational < 0.5:
        reasons.append(
            "OPERATIONAL GATE"
        )

    return {
        "name": name,
        "alt": altitude_ft,
        "bias": temp_bias_f,
        "target": avg_target,
        "density_target": avg_density_target,
        "capability": avg_capability,
        "external_capability": external_capability,
        "capability_parity_error": capability_parity_error,
        "owner": owner,
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
        "max_bridge_error": max_bridge_error,
        "min_operational": min_operational,
        "status": (
            "PASS"
            if not reasons
            else ",".join(reasons)
        ),
    }


print(
    "MOONEY AF1B DENSITY CONTROLLER "
    "INTEGRATION-BRIDGE DYNAMIC REGRESSION"
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
            f"HP={result['hp']:6.2f}  "
            f"owner={result['owner']}"
        )

        print(
            f"    density={result['density_target']:.2f}\"  "
            f"capInt={result['capability']:.3f}\"  "
            f"capExt={result['external_capability']:.3f}\"  "
            f"dCap={result['capability_parity_error']:+.4f}\""
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
            f"    operationalMin="
            f"{result['min_operational']:.0f}  "
            f"maxBridgeErr="
            f"{result['max_bridge_error']:.9f}"
        )

        print(
            f"    status: "
            f"{result['status']}"
        )

        print()


print(
    "RESULT"
)
print(
    "------"
)

if all_pass:
    print(
        "AF1B DENSITY CONTROLLER "
        "INTEGRATION-BRIDGE DYNAMIC REGRESSION PASS"
    )
else:
    print(
        "AF1B DENSITY CONTROLLER "
        "INTEGRATION-BRIDGE DYNAMIC REGRESSION FAIL"
    )
    raise SystemExit(1)
