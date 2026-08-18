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
    / "af1b-density-controller-stage5c.xml"
)

DT = 1.0 / 120.0
KTS_TO_FPS = 1.687809857

PSF_PER_INHG = 70.72620474785911

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

CAPABILITY_ENABLED = (
    BASE + "/capability-enabled"
)

MAXIMUM_MAP = (
    BASE + "/maximum-map-inhg"
)

CONTROLLER_TARGET = (
    BASE + "/controller-target-map-inhg"
)

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


def clamp(
    value,
    low,
    high,
):
    return max(
        low,
        min(
            high,
            value,
        ),
    )


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
            / "af1b-density-controller-stage5c.xml"
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
        'file="af1b-density-controller-stage5c" />\n'
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


def ambient_pressure_inhg(
    root,
    altitude_ft,
    temp_bias_f,
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
            "could not load temporary Mooney FDM "
            "for atmosphere probe"
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
    root,
    altitude_ft,
    temp_bias_f,
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
        "controller_target": get(
            fdm,
            CONTROLLER_TARGET,
        ),
        "maximum_map": get(
            fdm,
            MAXIMUM_MAP,
        ),
        "map": get(
            fdm,
            MAP,
        ),
        "error": (
            get(fdm, CONTROLLER_TARGET)
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
    "MOONEY AF1B STAGE-5C "
    "INTERNAL CAPABILITY ENGINE VALIDATION"
)
print(
    "=============================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "capability source: Stage-5C XML"
)
print(
    "external Stage-5B equation: comparison only"
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
print()

TEST_INFLOW_KTS = 125.0

TEMP_BIASES_F = (
    -15.0,
    0.0,
    15.0,
)

CP_FACTORS = (
    1.25,
    1.40,
    1.55,
    1.70,
    1.85,
    2.00,
    2.15,
    2.30,
    2.45,
    2.60,
)

TARGET_RPM = 2575.0

# Harness criteria, not Lycoming tolerances.
RPM_SELECTION_TOL = 5.0
RPM_QUALIFICATION_TOL = 2.0
MAP_TRACKING_TOL = 0.05
TARGET_LOGIC_TOL = 0.01
CAPABILITY_PARITY_TOL = 0.01
BLADE_LIMIT = 44.45

rows = []

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)

    prepare_test_tree(
        root
    )

    prop_path = (
        root
        / "Engines"
        / "M20M-Propeller.xml"
    )

    base_prop_text = (
        prop_path.read_text()
    )

    original = (
        "<cp_factor>1.25</cp_factor>"
    )

    if base_prop_text.count(original) != 1:
        raise RuntimeError(
            "Expected permanent starter cp_factor=1.25 "
            "exactly once in temporary prop copy"
        )

    # External Stage-5B oracle pressures.
    #
    # Comparison only. Nothing from this dictionary is written
    # into Stage-5C.
    isa_pressure = {
        altitude_ft: ambient_pressure_inhg(
            root,
            altitude_ft,
            0.0,
        )
        for altitude_ft in ALTITUDES_FT
    }

    for temp_bias_f in TEMP_BIASES_F:
        for altitude_ft in ALTITUDES_FT:

            current_pressure = (
                ambient_pressure_inhg(
                    root,
                    altitude_ft,
                    temp_bias_f,
                )
            )

            external_isa_capability = (
                figure_3_34_post_critical_map(
                    altitude_ft
                )
            )

            external_capability = clamp(
                external_isa_capability
                * current_pressure
                / isa_pressure[altitude_ft],
                0.0,
                38.0,
            )

            candidates = []

            for cp_factor in CP_FACTORS:
                prop_text = (
                    base_prop_text.replace(
                        original,
                        (
                            "<cp_factor>"
                            f"{cp_factor:.6f}"
                            "</cp_factor>"
                        ),
                        1,
                    )
                )

                prop_path.write_text(
                    prop_text
                )

                # IMPORTANT:
                # No capability value is supplied here.
                fdm = make_fdm(
                    root,
                    altitude_ft,
                    temp_bias_f,
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

            qualified_candidates = [
                row
                for row in candidates
                if (
                    abs(
                        row[1]["rpm"]
                        - TARGET_RPM
                    ) <= RPM_SELECTION_TOL
                    and row[1]["blade"]
                    < BLADE_LIMIT
                )
            ]

            if not qualified_candidates:
                detail = ", ".join(
                    (
                        f"CP={cp_factor:.2f}: "
                        f"RPM={sample['rpm']:.1f}, "
                        f"blade={sample['blade']:.2f}"
                    )
                    for cp_factor, sample
                    in candidates
                )

                raise RuntimeError(
                    "No uncontaminated dyno point at "
                    f"dT={temp_bias_f:+.0f}, "
                    f"alt={altitude_ft:.0f}: "
                    + detail
                )

            best_factor, best = min(
                qualified_candidates,
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

            internal_capability = (
                best["maximum_map"]
            )

            expected_controller_target = min(
                best["target"],
                internal_capability,
            )

            owner = (
                "DENSITY"
                if (
                    best["target"]
                    <= internal_capability
                )
                else "CAPABILITY"
            )

            rpm_error = (
                best["rpm"]
                - TARGET_RPM
            )

            map_error = (
                best["map"]
                - best["controller_target"]
            )

            target_logic_error = (
                best["controller_target"]
                - expected_controller_target
            )

            capability_parity_error = (
                internal_capability
                - external_capability
            )

            rpm_ok = (
                abs(rpm_error)
                <= RPM_QUALIFICATION_TOL
            )

            blade_ok = (
                best["blade"]
                < BLADE_LIMIT
            )

            map_ok = (
                abs(map_error)
                <= MAP_TRACKING_TOL
            )

            target_ok = (
                abs(target_logic_error)
                <= TARGET_LOGIC_TOL
            )

            capability_ok = (
                abs(
                    capability_parity_error
                )
                <= CAPABILITY_PARITY_TOL
            )

            passed = (
                rpm_ok
                and blade_ok
                and map_ok
                and target_ok
                and capability_ok
            )

            rows.append(
                {
                    "temp_bias": temp_bias_f,
                    "altitude": altitude_ft,
                    "cp": best_factor,
                    "s": best,
                    "internal_cap": (
                        internal_capability
                    ),
                    "external_cap": (
                        external_capability
                    ),
                    "cap_error": (
                        capability_parity_error
                    ),
                    "owner": owner,
                    "rpm_error": rpm_error,
                    "map_error": map_error,
                    "target_logic_error": (
                        target_logic_error
                    ),
                    "passed": passed,
                }
            )


print(
    " dT   Alt   CapInt  CapExt    dCap"
    "   Density   CtrlTgt    MAP"
    "     HP     RPM   Blade"
    "      VE      Boost    CtrlCap"
    "    Owner       Q"
)
print(
    "--------------------------------------------------------"
    "--------------------------------------------------------"
    "----------------"
)

for row in rows:
    s = row["s"]

    print(
        f"{row['temp_bias']:+3.0f}  "
        f"{row['altitude']/1000:4.0f}K  "
        f"{row['internal_cap']:6.3f}  "
        f"{row['external_cap']:6.3f}  "
        f"{row['cap_error']:+7.4f}  "
        f"{s['target']:7.2f}  "
        f"{s['controller_target']:7.2f}  "
        f"{s['map']:6.2f}  "
        f"{s['hp']:6.1f}  "
        f"{s['rpm']:6.1f}  "
        f"{s['blade']:6.2f}  "
        f"{s['ve']:7.5f}  "
        f"{s['boost']:7.5f}  "
        f"{s['cap']:7.4f}  "
        f"{row['owner']:10s}  "
        f"{'PASS' if row['passed'] else 'FAIL'}"
    )


max_capability_error = max(
    abs(
        row["cap_error"]
    )
    for row in rows
)

max_map_tracking_error = max(
    abs(
        row["map_error"]
    )
    for row in rows
)

max_target_logic_error = max(
    abs(
        row["target_logic_error"]
    )
    for row in rows
)


print()
print(
    "MAXIMUM ERRORS"
)
print(
    "--------------"
)
print(
    f"Stage-5C vs external Stage-5B capability: "
    f"{max_capability_error:.6f}\""
)
print(
    f"MAP vs controller target: "
    f"{max_map_tracking_error:.6f}\""
)
print(
    f"controller-target logic: "
    f"{max_target_logic_error:.6f}\""
)


print()
print(
    "20K OWNERSHIP"
)
print(
    "-------------"
)

for row in rows:
    if row["altitude"] != 20000.0:
        continue

    s = row["s"]

    print(
        f"dT={row['temp_bias']:+.0f}: "
        f"density={s['target']:.2f}\"  "
        f"cap={row['internal_cap']:.2f}\"  "
        f"MAP={s['map']:.2f}\"  "
        f"HP={s['hp']:.1f}  "
        f"owner={row['owner']}"
    )


print()
print(
    "22K / 24K POWER"
)
print(
    "---------------"
)

for row in rows:
    if row["altitude"] not in (
        22000.0,
        24000.0,
    ):
        continue

    s = row["s"]

    print(
        f"{row['altitude']/1000:.0f}K "
        f"dT={row['temp_bias']:+.0f}: "
        f"MAP={s['map']:.2f}\"  "
        f"HP={s['hp']:.1f}  "
        f"VE={s['ve']:.5f}"
    )


all_passed = all(
    row["passed"]
    for row in rows
)


print()
print(
    "RESULT"
)
print(
    "------"
)

if all_passed:
    print(
        "AF1B STAGE-5C INTERNAL "
        "CAPABILITY ENGINE MATRIX PASS"
    )
else:
    print(
        "AF1B STAGE-5C INTERNAL "
        "CAPABILITY ENGINE MATRIX FAIL"
    )

    raise SystemExit(1)
