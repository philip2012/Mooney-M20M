#!/usr/bin/env python3

from pathlib import Path

from af1b_lycoming_reference import (
    figure_3_34_post_critical_map,
)

import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

SYSTEM_XML = (
    REPO
    / "Tools"
    / "jsbsim"
    / "af1b-density-controller-stage3a.xml"
)

DT = 1.0 / 120.0
KTS_TO_FPS = 1.687809857

ALTITUDES_FT = (
    19000.0,
    20000.0,
    21000.0,
    22000.0,
    23000.0,
    24000.0,
    25000.0,
)

BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
VE_ENABLED = BASE + "/ve-enabled"

AMBIENT_MAP = BASE + "/ambient-pressure-inhg"
TARGET_MAP = BASE + "/target-map-inhg"

FEEDFORWARD = BASE + "/feedforward-cap-norm"
PRE_REQUEST = (
    BASE
    + "/pre-integrator-controller-request-norm"
)
RAW_CAP = BASE + "/raw-controller-cap-norm"
CONTROLLER_CAP = BASE + "/controller-cap-norm"

FEEDBACK_READY = BASE + "/feedback-ready"
FEEDBACK_GOVERNING = BASE + "/feedback-governing"
FEEDBACK_INTEGRAL = BASE + "/feedback-integral-norm"

ANTI_UPPER = BASE + "/antiwindup-upper"
INTEGRATOR_TRIGGER = (
    BASE + "/feedback-integrator-trigger"
)

BOOST_FRACTION = BASE + "/ve-boost-fraction"

VE_PROP = (
    "propulsion/engine[0]/volumetric-efficiency"
)

MAP = "propulsion/engine[0]/map-inhg"
RPM = "propulsion/engine[0]/propeller-rpm"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"

RAM_AIR = "propulsion/engine[0]/ram-air-factor"

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


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


def ramp_inflow(
    fdm,
    final_kts,
):
    # Ramp progressively so the propeller/governor is not hit
    # with an instantaneous inflow step.
    steps = (
        25.0,
        50.0,
        75.0,
        100.0,
        125.0,
        150.0,
        175.0,
    )

    for kts in steps:
        if kts >= final_kts:
            break

        fdm.set_property_value(
            "atmosphere/wind-north-fps",
            -kts * KTS_TO_FPS,
        )

        run_for(
            fdm,
            0.25,
        )

    fdm.set_property_value(
        "atmosphere/wind-north-fps",
        -final_kts * KTS_TO_FPS,
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
):
    steps = int(
        seconds / DT
    )

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
                "pilot ramp"
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
            / "af1b-density-controller-stage3a.xml"
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
        'file="af1b-density-controller-stage3a" />\n'
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


def make_fdm(
    root,
    altitude_ft,
    inflow_kts,
):
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

    # First critical-altitude qualification is ISA.
    fdm.set_property_value(
        "atmosphere/delta-T",
        0.0,
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
        fdm,
        inflow_kts,
    )

    run_for(
        fdm,
        1.0,
    )

    # Same staged activation used by the qualified controller
    # regression: establish VE scheduling first, then controller.
    fdm.set_property_value(
        VE_ENABLED,
        1,
    )

    run_for(
        fdm,
        2.0,
    )

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

    # Long enough for feedback and propulsion to converge or
    # for upper saturation to become unambiguous.
    run_for(
        fdm,
        18.0,
    )

    return fdm


def snapshot(fdm):
    return {
        "ambient": get(
            fdm,
            AMBIENT_MAP,
        ),
        "target": get(
            fdm,
            TARGET_MAP,
        ),
        "map": get(
            fdm,
            MAP,
        ),
        "error": (
            get(fdm, TARGET_MAP)
            - get(fdm, MAP)
        ),
        "ff": get(
            fdm,
            FEEDFORWARD,
        ),
        "pre": get(
            fdm,
            PRE_REQUEST,
        ),
        "raw": get(
            fdm,
            RAW_CAP,
        ),
        "cap": get(
            fdm,
            CONTROLLER_CAP,
        ),
        "ready": get(
            fdm,
            FEEDBACK_READY,
        ),
        "governing": get(
            fdm,
            FEEDBACK_GOVERNING,
        ),
        "integral": get(
            fdm,
            FEEDBACK_INTEGRAL,
        ),
        "anti": get(
            fdm,
            ANTI_UPPER,
        ),
        "trigger": get(
            fdm,
            INTEGRATOR_TRIGGER,
        ),
        "boost": get(
            fdm,
            BOOST_FRACTION,
        ),
        "ve": get(
            fdm,
            VE_PROP,
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
    }


print(
    "MOONEY AF1B STAGE-3A "
    "RATED-ALTITUDE A/B VALIDATION"
)
print(
    "============================================"
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "pilot throttle: 1.00"
)
print(
    "prop command: full RPM"
)
print(
    "engine ram-air-factor: 0.0"
)
print(
    "temperature: ISA"
)
print()

# A/B values are applied only to a temporary copy of the engine XML.
RATED_ALTITUDES_FT = (
    20000.0,
    19000.0,
)

# Artificial propeller inflow exists only to provide repeatable
# dynamometer loading. Engine ram recovery remains disabled.
TEST_INFLOW_KTS = 125.0

CP_FACTORS = (
    1.25,
    1.30,
    1.35,
    1.40,
    1.45,
    1.50,
    1.55,
    1.60,
)

TARGET_RPM = 2575.0

all_rows = {}

for rated_altitude_ft in RATED_ALTITUDES_FT:
    rows = []

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        prepare_test_tree(
            root
        )

        engine_path = (
            root
            / "Engines"
            / "Lycoming-TIO-540-AF1B.xml"
        )

        engine_text = engine_path.read_text()

        old_rated_altitude = (
            '<ratedaltitude1 unit="FT">'
            '20000'
            '</ratedaltitude1>'
        )

        new_rated_altitude = (
            '<ratedaltitude1 unit="FT">'
            f'{rated_altitude_ft:.0f}'
            '</ratedaltitude1>'
        )

        if engine_text.count(old_rated_altitude) != 1:
            raise RuntimeError(
                "Expected permanent ratedaltitude1=20000 "
                "exactly once in temporary engine copy"
            )

        engine_path.write_text(
            engine_text.replace(
                old_rated_altitude,
                new_rated_altitude,
                1,
            )
        )

        prop_path = (
            root
            / "Engines"
            / "M20M-Propeller.xml"
        )

        base_prop_text = prop_path.read_text()

        original_cp = (
            "<cp_factor>1.25</cp_factor>"
        )

        if base_prop_text.count(original_cp) != 1:
            raise RuntimeError(
                "Expected permanent starter cp_factor "
                "1.25 exactly once in temporary prop copy"
            )

        for altitude_ft in ALTITUDES_FT:
            candidates = []

            for cp_factor in CP_FACTORS:
                prop_text = base_prop_text.replace(
                    original_cp,
                    (
                        "<cp_factor>"
                        f"{cp_factor:.6f}"
                        "</cp_factor>"
                    ),
                    1,
                )

                prop_path.write_text(
                    prop_text
                )

                # Fresh FDM reloads both the temporary engine
                # and temporary propeller for every candidate.
                fdm = make_fdm(
                    root,
                    altitude_ft,
                    TEST_INFLOW_KTS,
                )

                s = snapshot(
                    fdm
                )

                candidates.append(
                    (
                        cp_factor,
                        s,
                    )
                )

            best_factor, best = min(
                candidates,
                key=lambda row: (
                    abs(
                        row[1]["rpm"]
                        - TARGET_RPM
                    ),
                    abs(
                        row[0]
                        - 1.25
                    ),
                ),
            )

            reference = (
                figure_3_34_post_critical_map(
                    altitude_ft
                )
            )

            delta = (
                best["map"]
                - reference
            )

            rows.append(
                (
                    altitude_ft,
                    best_factor,
                    best,
                    reference,
                    delta,
                )
            )

    all_rows[
        rated_altitude_ft
    ] = rows


for rated_altitude_ft in RATED_ALTITUDES_FT:
    print()
    print(
        f"RATEDALTITUDE1 = "
        f"{rated_altitude_ft:.0f} FT"
    )
    print(
        "-----------------------------------------------"
    )
    print(
        " Alt  CPfac    RPM   Blade    MAP    Ref"
        "   Delta      HP    Cap"
    )
    print(
        "---------------------------------------------------------------"
    )

    for (
        altitude_ft,
        cp_factor,
        s,
        reference,
        delta,
    ) in all_rows[rated_altitude_ft]:
        print(
            f"{altitude_ft / 1000:4.0f}K  "
            f"{cp_factor:5.2f}  "
            f"{s['rpm']:7.1f}  "
            f"{s['blade']:6.2f}  "
            f"{s['map']:6.2f}  "
            f"{reference:5.2f}  "
            f"{delta:+6.2f}  "
            f"{s['hp']:7.1f}  "
            f"{s['cap']:5.3f}"
        )


print()
print(
    "DIRECT A/B"
)
print(
    "----------"
)
print(
    " Alt   MAP@20kRA  MAP@19kRA   Change"
    "   Ref    Delta19kRA   HP19kRA"
)
print(
    "--------------------------------------------------------------"
)

baseline = {
    altitude_ft: (
        s,
        reference,
        delta,
    )
    for (
        altitude_ft,
        cp_factor,
        s,
        reference,
        delta,
    ) in all_rows[20000.0]
}

candidate = {
    altitude_ft: (
        s,
        reference,
        delta,
    )
    for (
        altitude_ft,
        cp_factor,
        s,
        reference,
        delta,
    ) in all_rows[19000.0]
}

for altitude_ft in ALTITUDES_FT:
    base_s, reference, base_delta = baseline[
        altitude_ft
    ]

    cand_s, _, cand_delta = candidate[
        altitude_ft
    ]

    change = (
        cand_s["map"]
        - base_s["map"]
    )

    print(
        f"{altitude_ft / 1000:4.0f}K  "
        f"{base_s['map']:9.2f}  "
        f"{cand_s['map']:9.2f}  "
        f"{change:+7.2f}  "
        f"{reference:5.2f}  "
        f"{cand_delta:+10.2f}  "
        f"{cand_s['hp']:8.1f}"
    )


print()
print(
    "SUMMARY"
)
print(
    "-------"
)

for rated_altitude_ft in RATED_ALTITUDES_FT:
    qualified_deltas = []

    for (
        altitude_ft,
        cp_factor,
        s,
        reference,
        delta,
    ) in all_rows[rated_altitude_ft]:
        if altitude_ft < 20000.0:
            continue

        rpm_ok = (
            abs(
                s["rpm"]
                - TARGET_RPM
            )
            <= 2.0
        )

        blade_ok = (
            s["blade"]
            < 44.45
        )

        if rpm_ok and blade_ok:
            qualified_deltas.append(
                delta
            )

    mean_abs_delta = (
        sum(
            abs(delta)
            for delta in qualified_deltas
        )
        / len(qualified_deltas)
    )

    max_abs_delta = max(
        abs(delta)
        for delta in qualified_deltas
    )

    print(
        f"ratedaltitude1={rated_altitude_ft:.0f} ft:  "
        f"20-25k mean |MAP error|="
        f"{mean_abs_delta:.3f}\"  "
        f"max |MAP error|="
        f"{max_abs_delta:.3f}\""
    )


print()
print(
    "INTERPRETATION"
)
print(
    "--------------"
)
print(
    "This test changes ratedaltitude1 only in a temporary copied "
    "engine XML. The permanent AF1B engine file is untouched."
)
print(
    "The temporary CP factor is a dynamometer load, not a "
    "propeller-tuning recommendation."
)
print(
    "The 19000-ft candidate is acceptable only if it improves "
    "the Lycoming maximum-MAP envelope while preserving the "
    "20,000-ft rated operating point and sane Stage-3A behavior."
)
