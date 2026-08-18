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
    / "af1b-density-controller-stage5b.xml"
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

TEST_MAXIMUM_MAP = (
    BASE + "/test-maximum-map-inhg"
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
            / "af1b-density-controller-stage5b.xml"
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
        'file="af1b-density-controller-stage5b" />\n'
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
    capability_inhg,
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
        TEST_MAXIMUM_MAP,
        capability_inhg,
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
    "MOONEY AF1B STAGE-5B "
    "NON-ISA CAPABILITY-HANDOFF CONTINUITY"
)
print(
    "=============================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "candidate law:"
)
print(
    "  Figure-3-34 ISA capability"
)
print(
    "  * Pamb(non-ISA) / Pamb(ISA)"
)
print()

TEST_INFLOW_KTS = 125.0

# Narrow windows around the three observed/predicted
# ownership crossovers.
SWEEPS = {
    -15.0: tuple(
        19650.0 + 25.0 * i
        for i in range(13)
    ),
    0.0: tuple(
        19875.0 + 25.0 * i
        for i in range(13)
    ),
    15.0: tuple(
        20075.0 + 25.0 * i
        for i in range(13)
    ),
}

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
RPM_SELECTION_TOL = 5.0
BLADE_LIMIT = 44.45

all_rows = {}

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
            "Expected cp_factor=1.25 exactly once"
        )

    for temp_bias_f, altitudes in SWEEPS.items():

        isa_pressure = {
            altitude_ft: ambient_pressure_inhg(
                root,
                altitude_ft,
                0.0,
            )
            for altitude_ft in altitudes
        }

        rows = []

        for altitude_ft in altitudes:

            current_pressure = (
                ambient_pressure_inhg(
                    root,
                    altitude_ft,
                    temp_bias_f,
                )
            )

            isa_capability = (
                figure_3_34_post_critical_map(
                    altitude_ft
                )
            )

            capability = clamp(
                isa_capability
                * current_pressure
                / isa_pressure[altitude_ft],
                0.0,
                38.0,
            )

            candidates = []

            for cp_factor in CP_FACTORS:
                prop_path.write_text(
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

                fdm = make_fdm(
                    root,
                    altitude_ft,
                    temp_bias_f,
                    capability,
                    TEST_INFLOW_KTS,
                )

                candidates.append(
                    (
                        cp_factor,
                        snapshot(fdm),
                    )
                )

            qualified = [
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

            if not qualified:
                raise RuntimeError(
                    "No clean prop-load point at "
                    f"dT={temp_bias_f:+.0f}, "
                    f"alt={altitude_ft:.0f}"
                )

            cp_factor, s = min(
                qualified,
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

            owner = (
                "DENSITY"
                if s["target"] <= capability
                else "CAPABILITY"
            )

            rows.append(
                {
                    "alt": altitude_ft,
                    "cp": cp_factor,
                    "capability": capability,
                    "owner": owner,
                    "s": s,
                }
            )

        all_rows[temp_bias_f] = rows


overall_pass = True

for temp_bias_f, rows in all_rows.items():

    print()
    print(
        f"dT = {temp_bias_f:+.0f} F"
    )
    print(
        "----------------------------------------------"
    )
    print(
        " Alt     Density   CapB    Ctrl"
        "     MAP      HP       VE"
        "      Boost     Cap     Owner"
    )

    for row in rows:
        s = row["s"]

        print(
            f"{row['alt']:7.0f}  "
            f"{s['target']:7.3f}  "
            f"{row['capability']:6.3f}  "
            f"{s['controller_target']:6.3f}  "
            f"{s['map']:6.3f}  "
            f"{s['hp']:7.2f}  "
            f"{s['ve']:8.5f}  "
            f"{s['boost']:8.5f}  "
            f"{s['cap']:7.4f}  "
            f"{row['owner']}"
        )

    transitions = []

    max_dmap = 0.0
    max_dhp = 0.0
    max_dve = 0.0
    max_dboost = 0.0
    max_dcap = 0.0

    print()
    print(
        "25-FT STEP CONTINUITY"
    )

    for previous, current in zip(
        rows,
        rows[1:],
    ):
        ps = previous["s"]
        cs = current["s"]

        dmap = cs["map"] - ps["map"]
        dhp = cs["hp"] - ps["hp"]
        dve = cs["ve"] - ps["ve"]
        dboost = cs["boost"] - ps["boost"]
        dcap = cs["cap"] - ps["cap"]

        max_dmap = max(
            max_dmap,
            abs(dmap),
        )

        max_dhp = max(
            max_dhp,
            abs(dhp),
        )

        max_dve = max(
            max_dve,
            abs(dve),
        )

        max_dboost = max(
            max_dboost,
            abs(dboost),
        )

        max_dcap = max(
            max_dcap,
            abs(dcap),
        )

        if (
            previous["owner"]
            != current["owner"]
        ):
            transitions.append(
                (
                    previous["alt"],
                    current["alt"],
                    previous["owner"],
                    current["owner"],
                )
            )

        print(
            f"{previous['alt']:7.0f}"
            f" -> {current['alt']:7.0f}  "
            f"dMAP={dmap:+.4f}\"  "
            f"dHP={dhp:+.3f}  "
            f"dVE={dve:+.6f}  "
            f"dBoost={dboost:+.6f}  "
            f"dCap={dcap:+.5f}"
        )

    transition_ok = (
        len(transitions) == 1
        and transitions[0][2] == "DENSITY"
        and transitions[0][3] == "CAPABILITY"
    )

    tracking_ok = all(
        abs(
            row["s"]["map"]
            - row["s"]["controller_target"]
        ) <= 0.05
        for row in rows
    )

    print()
    print(
        "SUMMARY"
    )
    print(
        f"ownership transitions: {transitions}"
    )
    print(
        f"max |dMAP|   = {max_dmap:.4f}\""
    )
    print(
        f"max |dHP|    = {max_dhp:.3f} HP"
    )
    print(
        f"max |dVE|    = {max_dve:.6f}"
    )
    print(
        f"max |dBoost| = {max_dboost:.6f}"
    )
    print(
        f"max |dCap|   = {max_dcap:.5f}"
    )
    print(
        "single clean DENSITY->CAPABILITY handoff: "
        f"{'PASS' if transition_ok else 'FAIL'}"
    )
    print(
        "MAP tracking: "
        f"{'PASS' if tracking_ok else 'FAIL'}"
    )

    if not (
        transition_ok
        and tracking_ok
    ):
        overall_pass = False


print()
print(
    "RESULT"
)
print(
    "------"
)

if overall_pass:
    print(
        "AF1B STAGE-5B NON-ISA "
        "HANDOFF CONTINUITY PASS"
    )
else:
    print(
        "AF1B STAGE-5B NON-ISA "
        "HANDOFF CONTINUITY FAIL"
    )
    raise SystemExit(1)
