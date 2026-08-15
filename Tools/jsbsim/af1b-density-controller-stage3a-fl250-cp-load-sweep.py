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
    / "af1b-density-controller-stage3a.xml"
)

DT = 1.0 / 120.0
KTS_TO_FPS = 1.687809857

ALTITUDES_FT = (
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
        0.9066666666666666,
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
    "FL250 2400-RPM CP-LOAD SWEEP"
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
    "temperature: ISA"
)
print()

TEST_INFLOW_KTS = 125.0

# Temporary dynamometer-load multipliers.
# Permanent M20M-Propeller.xml remains untouched.
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

    base_prop_text = prop_path.read_text()

    original = "<cp_factor>1.25</cp_factor>"

    if base_prop_text.count(original) != 1:
        raise RuntimeError(
            "Expected permanent starter cp_factor "
            "1.25 exactly once in temporary prop copy"
        )

    for cp_factor in CP_FACTORS:
        # Rewrite only the TEMPORARY copied propeller.
        prop_text = base_prop_text.replace(
            original,
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

        # Fresh FDM means the modified temporary prop file is
        # reloaded cleanly for every CP-factor point.
        fdm = make_fdm(
            root,
            25000.0,
            TEST_INFLOW_KTS,
        )

        s = snapshot(
            fdm
        )

        rows.append(
            (
                cp_factor,
                s,
            )
        )


print(
    f"artificial inflow: {TEST_INFLOW_KTS:.1f} kt"
)
print()
print(
    " CPfac    MAP     RPM      HP    Blade    Cap    Boost     VE"
)
print(
    "---------------------------------------------------------------"
)

for cp_factor, s in rows:
    print(
        f"{cp_factor:6.3f}  "
        f"{s['map']:6.2f}  "
        f"{s['rpm']:7.1f}  "
        f"{s['hp']:6.1f}  "
        f"{s['blade']:6.2f}  "
        f"{s['cap']:5.3f}  "
        f"{s['boost']:6.3f}  "
        f"{s['ve']:6.3f}"
    )


print()
print(
    "CLOSEST TO 2400 RPM"
)
print(
    "-------------------"
)

best_factor, best = min(
    rows,
    key=lambda row: abs(
        row[1]["rpm"] - 2400.0
    ),
)

rpm_error = (
    best["rpm"]
    - 2400.0
)

print(
    f"cp_factor={best_factor:.3f}  "
    f"RPM={best['rpm']:.1f}  "
    f"MAP={best['map']:.3f}\"  "
    f"HP={best['hp']:.2f}  "
    f"blade={best['blade']:.2f}  "
    f"cap={best['cap']:.6f}  "
    f"VE={best['ve']:.6f}"
)

print(
    f"RPM error={rpm_error:+.1f}"
)

if abs(rpm_error) <= 25.0:
    if best["map"] >= 34.0:
        print(
            "FL250 34-INHG / 2400-RPM CAPABILITY: PASS"
        )
    else:
        print(
            "FL250 34-INHG / 2400-RPM CAPABILITY: FAIL"
        )
else:
    print(
        "FL250 CAPABILITY: INCONCLUSIVE "
        "(no CP-load point held approximately 2400 RPM)"
    )


print()
print(
    "INTERPRETATION"
)
print(
    "--------------"
)
print(
    "This is a temporary dynamometer-load experiment."
)
print(
    "cp_factor is varied only in the temporary copied propeller; "
    "no permanent propeller tuning is implied."
)
print(
    "The useful result is the MAP available when actual RPM is "
    "approximately 2400."
)
print(
    "Only that point should be compared with the documented "
    "FL250 34-inHg / 2400-RPM capability constraint."
)
