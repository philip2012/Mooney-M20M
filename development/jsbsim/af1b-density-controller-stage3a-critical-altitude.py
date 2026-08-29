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
    18000.0,
    20000.0,
    22000.0,
    24000.0,
    26000.0,
    28000.0,
    30000.0,
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
        fdm
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
    "CRITICAL-ALTITUDE SWEEP"
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

rows = []

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)

    prepare_test_tree(
        root
    )

    for altitude_ft in ALTITUDES_FT:
        fdm = make_fdm(
            root,
            altitude_ft,
        )

        start = snapshot(
            fdm
        )

        # Measure whether a saturated controller continues winding
        # its integral after the system has already settled.
        integral_start = start["integral"]

        minimum_anti = start["anti"]
        minimum_trigger = start["trigger"]
        minimum_error = start["error"]
        minimum_pre = start["pre"]

        steps = int(
            5.0 / DT
        )

        for _ in range(steps):
            if not fdm.run():
                raise RuntimeError(
                    "JSBSim stopped during "
                    "anti-windup observation"
                )

            sample = snapshot(
                fdm
            )

            minimum_anti = min(
                minimum_anti,
                sample["anti"],
            )

            minimum_trigger = min(
                minimum_trigger,
                sample["trigger"],
            )

            minimum_error = min(
                minimum_error,
                sample["error"],
            )

            minimum_pre = min(
                minimum_pre,
                sample["pre"],
            )

        end = snapshot(
            fdm
        )

        integral_drift = (
            end["integral"]
            - integral_start
        )

        saturated = (
            end["cap"] > 0.999
            and end["error"] > 0.05
        )

        regulated = (
            abs(end["error"]) <= 0.10
        )

        if saturated:
            aw_ok = (
                minimum_anti > 0.5
                and minimum_trigger > 0.5
                and abs(integral_drift) < 1e-8
            )
        else:
            aw_ok = True

        if regulated:
            regime = "REGULATED"
        elif saturated and aw_ok:
            regime = "AUTH-LIMIT"
        elif saturated:
            regime = "AW-FAIL"
        else:
            regime = "TRANSITION"

        rows.append(
            (
                altitude_ft,
                end,
                integral_drift,
                regime,
                aw_ok,
            )
        )


print(
    " Alt    Pamb   Target   MAP    Err   "
    "Cap    Boost    VE     RPM     HP    Blade   Regime"
)
print(
    "--------------------------------------------------------------------------"
)

for (
    altitude_ft,
    s,
    drift,
    regime,
    aw_ok,
) in rows:
    print(
        f"{altitude_ft/1000:4.0f}K  "
        f"{s['ambient']:6.2f}  "
        f"{s['target']:6.2f}  "
        f"{s['map']:6.2f}  "
        f"{s['error']:+5.2f}  "
        f"{s['cap']:5.3f}  "
        f"{s['boost']:6.3f}  "
        f"{s['ve']:6.3f}  "
        f"{s['rpm']:7.1f}  "
        f"{s['hp']:6.1f}  "
        f"{s['blade']:6.2f}  "
        f"{regime}"
    )


print()
print(
    "DETAIL"
)
print(
    "------"
)

for (
    altitude_ft,
    s,
    drift,
    regime,
    aw_ok,
) in rows:
    print(
        f"{altitude_ft:7.0f} ft  "
        f"{regime}"
    )

    print(
        f"    Pamb={s['ambient']:.3f}\"  "
        f"target={s['target']:.3f}\"  "
        f"MAP={s['map']:.3f}\"  "
        f"error={s['error']:+.3f}\""
    )

    print(
        f"    FF={s['ff']:.6f}  "
        f"pre={s['pre']:.6f}  "
        f"raw={s['raw']:.6f}  "
        f"cap={s['cap']:.6f}"
    )

    print(
        f"    ready={s['ready']:.0f}  "
        f"governing={s['governing']:.0f}  "
        f"anti-upper={s['anti']:.0f}  "
        f"trigger={s['trigger']:.0f}"
    )

    print(
        f"    boost={s['boost']:.6f}  "
        f"VE={s['ve']:.6f}  "
        f"RPM={s['rpm']:.1f}  "
        f"HP={s['hp']:.2f}  "
        f"blade={s['blade']:.2f}"
    )

    print(
        f"    integral={s['integral']:+.8f}  "
        f"5s drift={drift:+.12f}"
    )


print()
print(
    "INTERPRETATION"
)
print(
    "--------------"
)
print(
    "REGULATED: controller still has enough turbo authority "
    "to hold the Figure-17 MAP target."
)
print(
    "AUTH-LIMIT: full controller authority is reached, "
    "MAP falls below target naturally, and upper anti-windup "
    "freezes impossible positive demand."
)
print(
    "TRANSITION: near the critical-altitude boundary; review "
    "cap and MAP error rather than forcing a binary conclusion."
)
print(
    "No altitude in this test is required to retain 270 HP "
    "after turbo authority is exhausted."
)
