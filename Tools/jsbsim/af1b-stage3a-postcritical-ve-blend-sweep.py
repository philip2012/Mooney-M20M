#!/usr/bin/env python3

from pathlib import Path

from af1b_lycoming_reference import (
    AF1B_2575_POWER_REFERENCE,
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
    20000.0,
    22000.0,
    24000.0,
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

VE_CALIBRATED = (
    BASE + "/ve-calibrated"
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
        "ve_calibrated": get(
            fdm,
            VE_CALIBRATED,
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
    "POST-CRITICAL VE BLEND SWEEP"
)
print(
    "============================================"
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "temporary ratedaltitude1: 19000 ft"
)
print(
    "RPM target: 2575"
)
print(
    "engine ram-air-factor: 0.0"
)
print(
    "temperature: ISA"
)
print()

TEST_INFLOW_KTS = 125.0
TARGET_RPM = 2575.0

ALTITUDES_FT = (
    20000.0,
    22000.0,
    24000.0,
)

# 0.00 = current Stage-3A schedule
# 1.00 = full VEcal
#
# Intermediate values continuously blend the existing Stage-3A
# scheduled VE toward VEcal:
#
#   gain =
#       boost_fraction
#       + blend * (1 - boost_fraction)
#
#   VEtest =
#       0.90
#       + gain * (VEcal - 0.90)
#
BLENDS = (
    0.00,
    0.50,
    0.60,
    0.65,
    0.70,
    0.75,
    0.80,
    1.00,
)

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

power_reference = {
    point.altitude_ft: point
    for point in AF1B_2575_POWER_REFERENCE
}

rows = []

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)

    prepare_test_tree(
        root
    )

    # Apply the MAP-qualified rated-altitude candidate only to the
    # temporary copied engine.
    engine_path = (
        root
        / "Engines"
        / "Lycoming-TIO-540-AF1B.xml"
    )

    engine_text = engine_path.read_text()

    old_ra = (
        '<ratedaltitude1 unit="FT">'
        '20000'
        '</ratedaltitude1>'
    )

    new_ra = (
        '<ratedaltitude1 unit="FT">'
        '19000'
        '</ratedaltitude1>'
    )

    if engine_text.count(old_ra) != 1:
        raise RuntimeError(
            "Expected ratedaltitude1=20000 exactly once "
            "in temporary engine copy"
        )

    engine_path.write_text(
        engine_text.replace(
            old_ra,
            new_ra,
            1,
        )
    )

    system_path = (
        root
        / "Systems"
        / "af1b-density-controller-stage3a.xml"
    )

    base_system_text = system_path.read_text()

    old_schedule = '''        <fcs_function name="systems/af1b-density-controller/ve-scheduled">
            <function>
                <sum>
                    <value>0.90</value>

                    <product>
                        <property>
                            systems/af1b-density-controller/ve-boost-fraction
                        </property>

                        <difference>
                            <property>
                                systems/af1b-density-controller/ve-calibrated
                            </property>

                            <value>0.90</value>
                        </difference>
                    </product>
                </sum>
            </function>

            <clipto>
                <min>0.55</min>
                <max>0.92</max>
            </clipto>
        </fcs_function>
'''

    if base_system_text.count(old_schedule) != 1:
        raise RuntimeError(
            "Expected Stage-3A VE schedule exactly once"
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
            "Expected cp_factor=1.25 exactly once "
            "in temporary prop copy"
        )

    for blend in BLENDS:
        new_schedule = f'''        <fcs_function name="systems/af1b-density-controller/ve-scheduled">
            <function>
                <sum>
                    <value>0.90</value>

                    <product>
                        <sum>
                            <property>
                                systems/af1b-density-controller/ve-boost-fraction
                            </property>

                            <product>
                                <value>{blend:.6f}</value>

                                <difference>
                                    <value>1.0</value>

                                    <property>
                                        systems/af1b-density-controller/ve-boost-fraction
                                    </property>
                                </difference>
                            </product>
                        </sum>

                        <difference>
                            <property>
                                systems/af1b-density-controller/ve-calibrated
                            </property>

                            <value>0.90</value>
                        </difference>
                    </product>
                </sum>
            </function>

            <clipto>
                <min>0.55</min>
                <max>0.92</max>
            </clipto>
        </fcs_function>
'''

        # Rewrite only the TEMPORARY controller copy.
        system_path.write_text(
            base_system_text.replace(
                old_schedule,
                new_schedule,
                1,
            )
        )

        for altitude_ft in ALTITUDES_FT:
            candidates = []

            for cp_factor in CP_FACTORS:
                # Rewrite only the TEMPORARY propeller copy.
                prop_path.write_text(
                    base_prop_text.replace(
                        original_cp,
                        (
                            "<cp_factor>"
                            f"{cp_factor:.6f}"
                            "</cp_factor>"
                        ),
                        1,
                    )
                )

                # Fresh FDM reloads the temporary controller,
                # engine, and propeller for every candidate.
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

            rows.append(
                (
                    blend,
                    altitude_ft,
                    best_factor,
                    best,
                    power_reference[
                        altitude_ft
                    ],
                )
            )


print(
    " Blend  Alt    RPM     MAP      HP      VE"
    "    VEcal  Boost   Power error"
)
print(
    "--------------------------------------------------------------------"
)

for (
    blend,
    altitude_ft,
    cp_factor,
    s,
    ref,
) in rows:
    if ref.hp_nominal is not None:
        error = (
            s["hp"]
            - ref.hp_nominal
        )

        error_text = (
            f"{error:+6.1f} HP"
        )
    else:
        if s["hp"] < ref.hp_min:
            distance = (
                s["hp"]
                - ref.hp_min
            )
        elif s["hp"] > ref.hp_max:
            distance = (
                s["hp"]
                - ref.hp_max
            )
        else:
            distance = 0.0

        error_text = (
            f"{distance:+6.1f} HP"
        )

    print(
        f"{blend:5.2f}  "
        f"{altitude_ft / 1000:3.0f}K  "
        f"{s['rpm']:7.1f}  "
        f"{s['map']:6.2f}  "
        f"{s['hp']:7.1f}  "
        f"{s['ve']:6.3f}  "
        f"{s['ve_calibrated']:6.3f}  "
        f"{s['boost']:5.3f}   "
        f"{error_text}"
    )


print()
print(
    "BLEND SUMMARY"
)
print(
    "-------------"
)

for blend in BLENDS:
    selected = [
        row
        for row in rows
        if row[0] == blend
    ]

    errors = []

    for (
        _,
        altitude_ft,
        cp_factor,
        s,
        ref,
    ) in selected:
        if ref.hp_nominal is not None:
            errors.append(
                abs(
                    s["hp"]
                    - ref.hp_nominal
                )
            )
        else:
            if s["hp"] < ref.hp_min:
                errors.append(
                    ref.hp_min
                    - s["hp"]
                )
            elif s["hp"] > ref.hp_max:
                errors.append(
                    s["hp"]
                    - ref.hp_max
                )
            else:
                errors.append(
                    0.0
                )

    mean_error = (
        sum(errors)
        / len(errors)
    )

    print(
        f"blend={blend:4.2f}  "
        f"mean power distance="
        f"{mean_error:5.2f} HP"
    )


print()
print(
    "INTERPRETATION"
)
print(
    "--------------"
)
print(
    "blend=0 reproduces the current Stage-3A VE schedule."
)
print(
    "blend=1 applies VEcal at full strength."
)
print(
    "Intermediate blends remain continuous and therefore do not "
    "reintroduce the rejected Stage-3B binary discontinuity."
)
print(
    "The winning blend is diagnostic evidence only. Do not apply "
    "it to the permanent aircraft until continuity and reduced-power "
    "regressions have been repeated."
)
