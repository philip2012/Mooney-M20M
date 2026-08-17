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
    / "af1b-density-controller-stage5a.xml"
)

DT = 1.0 / 120.0
KTS_TO_FPS = 1.687809857

ALTITUDES_FT = tuple(
    19850.0 + 25.0 * index
    for index in range(17)
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
            / "af1b-density-controller-stage5a.xml"
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
        'file="af1b-density-controller-stage5a" />\n'
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
    "MOONEY AF1B STAGE-5A "
    "CAPABILITY-HANDOFF CONTINUITY"
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

# Propeller airflow exists only to provide a repeatable dynamometer
# load. Engine ram recovery remains disabled.
TEST_INFLOW_KTS = 125.0

# Temporary aerodynamic-load sweep.
# These values modify only the copied propeller inside the temp tree.
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

    for altitude_ft in ALTITUDES_FT:
        candidates = []

        for cp_factor in CP_FACTORS:
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

            # Fresh FDM for every point so the temporary propeller
            # coefficient is reloaded cleanly.
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

        # First minimize RPM error. If several points govern at
        # essentially the same RPM, prefer the smallest departure
        # from the permanent cp_factor=1.25.
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


print(
    f"artificial prop inflow: {TEST_INFLOW_KTS:.1f} kt"
)
print()
print(
    " Alt(ft)   Density    MaxMAP    CtrlTgt"
    "     MAP    HP      VE    Boost    Cap      Owner"
)
print(
    "------------------------------------------------"
    "-------------------------------------------"
)

for (
    altitude_ft,
    cp_factor,
    s,
    reference,
    delta,
) in rows:
    density_gap = abs(
        s["controller_target"]
        - s["target"]
    )

    capability_gap = abs(
        s["controller_target"]
        - s["maximum_map"]
    )

    owner = (
        "DENSITY"
        if density_gap <= capability_gap
        else "CAPABILITY"
    )

    print(
        f"{altitude_ft:7.0f}  "
        f"{s['target']:8.3f}  "
        f"{s['maximum_map']:8.3f}  "
        f"{s['controller_target']:8.3f}  "
        f"{s['map']:6.3f}  "
        f"{s['hp']:6.2f}  "
        f"{s['ve']:7.5f}  "
        f"{s['boost']:7.5f}  "
        f"{s['cap']:7.4f}  "
        f"{owner}"
    )


print()
print(
    "QUALIFICATION"
)
print(
    "-------------"
)

for (
    altitude_ft,
    cp_factor,
    s,
    reference,
    delta,
) in rows:
    rpm_error = (
        s["rpm"]
        - TARGET_RPM
    )

    rpm_ok = (
        abs(rpm_error)
        <= 2.0
    )

    # The current temporary starter prop has a 44.50-deg
    # coarse stop. We require some margin so the point is not
    # being defined by that stop.
    blade_ok = (
        s["blade"]
        < 44.45
    )

    if rpm_ok and blade_ok:
        qualification = "VALID"
    elif rpm_ok:
        qualification = "RPM-OK / PITCH-STOP"
    else:
        qualification = "RPM-MISMATCH"

    print(
        f"{altitude_ft:7.0f} ft  "
        f"{qualification:19s}  "
        f"RPMerr={rpm_error:+5.1f}  "
        f"CP={cp_factor:.2f}  "
        f"MAPdelta={delta:+.3f}\""
    )


print()
print()
print(
    "STEP CONTINUITY"
)
print(
    "---------------"
)
print(
    " From -> To       dMAP      dHP       dVE"
    "      dBoost     dCap"
)
print(
    "------------------------------------------------------------"
)

max_map_step = 0.0
max_hp_step = 0.0
max_ve_step = 0.0

for previous, current in zip(
    rows,
    rows[1:],
):
    (
        previous_altitude,
        previous_cp,
        previous_s,
        previous_ref,
        previous_delta,
    ) = previous

    (
        current_altitude,
        current_cp,
        current_s,
        current_ref,
        current_delta,
    ) = current

    dmap = (
        current_s["map"]
        - previous_s["map"]
    )

    dhp = (
        current_s["hp"]
        - previous_s["hp"]
    )

    dve = (
        current_s["ve"]
        - previous_s["ve"]
    )

    dboost = (
        current_s["boost"]
        - previous_s["boost"]
    )

    dcap = (
        current_s["cap"]
        - previous_s["cap"]
    )

    max_map_step = max(
        max_map_step,
        abs(dmap),
    )

    max_hp_step = max(
        max_hp_step,
        abs(dhp),
    )

    max_ve_step = max(
        max_ve_step,
        abs(dve),
    )

    print(
        f"{previous_altitude/1000:5.2f}K"
        f" -> "
        f"{current_altitude/1000:5.2f}K  "
        f"{dmap:+7.3f}\"  "
        f"{dhp:+7.2f}  "
        f"{dve:+8.5f}  "
        f"{dboost:+8.5f}  "
        f"{dcap:+7.4f}"
    )


print()
print(
    "MAXIMUM 25-FT STEP"
)
print(
    "------------------"
)

print(
    f"|dMAP|max = {max_map_step:.4f}\""
)
print(
    f"|dHP|max  = {max_hp_step:.3f} HP"
)
print(
    f"|dVE|max  = {max_ve_step:.6f}"
)

print(
    "INTERPRETATION"
)
print(
    "--------------"
)
print(
    "CP factor and artificial inflow are temporary dynamometer "
    "loads only. They are not propeller tuning recommendations."
)
print(
    "Engine ram-air-factor remains zero so MAP can be compared "
    "with Lycoming zero-ram altitude-performance data."
)
print(
    "Below the turbo capability boundary, Stage-5A may command "
    "less MAP than the Figure 3-34 maximum envelope; a negative "
    "delta there is not automatically an error."
)
print(
    "Above the capability boundary, positive MAP delta indicates "
    "the current JSBSim turbo can sustain more MAP than the "
    "Lycoming Figure 3-34 envelope."
)
