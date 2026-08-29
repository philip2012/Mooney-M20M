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
    / "af1b-density-controller-stage4b.xml"
)

DT = 1.0 / 120.0

KTS_TO_FPS = 1.687809857

PSF_TO_INHG = 0.01413903097960689

ALTITUDES_FT = (
    0.0,
    10000.0,
    20000.0,
    22000.0,
)

# Stage-4B low-boost/reduced-power characterization.
PILOT_THROTTLES = (
    0.30,
    0.40,
    0.50,
    0.60,
    0.70,
    0.80,
    0.90,
    1.00,
)

THROTTLE_RAMP_SEC = 1.0
SETTLE_SEC = 6.0
AVERAGE_SEC = 2.0

BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
VE_ENABLED = BASE + "/ve-enabled"

TARGET_MAP = BASE + "/target-map-inhg"
FEEDFORWARD = BASE + "/feedforward-cap-norm"
CONTROLLER_CAP = BASE + "/controller-cap-norm"

VE_BOOST_FRACTION = (
    BASE + "/ve-boost-fraction"
)

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

PAMB_PSF = "atmosphere/P-psf"

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

THROTTLE_COMMAND = "fcs/throttle-cmd-norm[0]"
THROTTLE_POSITION = "fcs/throttle-pos-norm[0]"


def get(fdm, prop):
    return fdm.get_property_value(prop)


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

        value = (
            start
            + (end - start)
            * fraction
        )

        fdm.set_property_value(
            PILOT_THROTTLE,
            value,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "pilot-throttle ramp"
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
            / "af1b-density-controller-stage4b.xml"
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
        'file="af1b-density-controller-stage4b" />\n'
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

    # MAP-qualified candidate.
    # TEMPORARY engine copy only.
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

    return root


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

    # ISA only for this first characterization sweep.
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

    # Critical isolation:
    # density-controller throttle authority never turns on.
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

    return fdm


def average_state(
    fdm,
    seconds,
):
    totals = {
        "pilot": 0.0,
        "command": 0.0,
        "position": 0.0,
        "target": 0.0,
        "feedforward": 0.0,
        "cap": 0.0,
        "pamb": 0.0,
        "ve": 0.0,
        "boost_fraction": 0.0,
        "map": 0.0,
        "rpm": 0.0,
        "hp": 0.0,
        "blade": 0.0,
    }

    steps = int(
        seconds / DT
    )

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "averaging interval"
            )

        totals["pilot"] += get(
            fdm,
            PILOT_THROTTLE,
        )

        totals["command"] += get(
            fdm,
            THROTTLE_COMMAND,
        )

        totals["position"] += get(
            fdm,
            THROTTLE_POSITION,
        )

        totals["target"] += get(
            fdm,
            TARGET_MAP,
        )

        totals["feedforward"] += get(
            fdm,
            FEEDFORWARD,
        )

        totals["cap"] += get(
            fdm,
            CONTROLLER_CAP,
        )

        totals["pamb"] += (
            get(
                fdm,
                PAMB_PSF,
            )
            * PSF_TO_INHG
        )

        totals["ve"] += get(
            fdm,
            VE_PROP,
        )

        totals["boost_fraction"] += get(
            fdm,
            VE_BOOST_FRACTION,
        )

        totals["map"] += get(
            fdm,
            MAP,
        )

        totals["rpm"] += get(
            fdm,
            RPM,
        )

        totals["hp"] += get(
            fdm,
            POWER,
        )

        totals["blade"] += get(
            fdm,
            BLADE,
        )

    return {
        key: value / steps
        for key, value in totals.items()
    }


def run_point(
    root,
    altitude_ft,
    pilot,
    ve_enabled,
):
    fdm = make_fdm(
        root,
        altitude_ft,
    )

    fdm.set_property_value(
        VE_ENABLED,
        1 if ve_enabled else 0,
    )

    # Allow VE routing to become established before moving
    # the pilot control away from the startup condition.
    run_for(
        fdm,
        0.5,
    )

    ramp_pilot(
        fdm,
        0.55,
        pilot,
        THROTTLE_RAMP_SEC,
    )

    run_for(
        fdm,
        SETTLE_SEC,
    )

    return average_state(
        fdm,
        AVERAGE_SEC,
    )


print(
    "MOONEY AF1B STAGE-4B LOW-BOOST VE REGRESSION"
)
print(
    "============================================"
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "controller throttle authority: DISABLED"
)
print(
    "comparison: native VE vs temporary "
    "full-power VE compensation"
)
print()

all_rows = []

with tempfile.TemporaryDirectory() as tmp:
    root = prepare_test_tree(
        Path(tmp)
    )

    for altitude_ft in ALTITUDES_FT:
        print()
        print(
            f"{altitude_ft:,.0f} FT ISA"
        )
        print(
            "------------------------------------------------------------"
        )
        print(
            "Pilot  Boost  MAPnat  MAPve   dMAP   "
            "HPnat   HPve    dHP    VEve"
        )

        for pilot in PILOT_THROTTLES:
            native = run_point(
                root,
                altitude_ft,
                pilot,
                False,
            )

            compensated = run_point(
                root,
                altitude_ft,
                pilot,
                True,
            )

            delta_map = (
                compensated["map"]
                - native["map"]
            )

            delta_hp = (
                compensated["hp"]
                - native["hp"]
            )

            row = {
                "altitude": altitude_ft,
                "pilot": pilot,
                "native": native,
                "compensated": compensated,
                "delta_map": delta_map,
                "delta_hp": delta_hp,
            }

            all_rows.append(
                row
            )

            print(
                f"{pilot:5.2f}  "
                f"{compensated['boost_fraction']:5.3f}  "
                f"{native['map']:6.2f}  "
                f"{compensated['map']:6.2f}  "
                f"{delta_map:+6.2f}  "
                f"{native['hp']:6.1f}  "
                f"{compensated['hp']:6.1f}  "
                f"{delta_hp:+6.1f}  "
                f"{compensated['ve']:6.3f}"
            )

        print()

print()
print(
    "DETAIL"
)
print(
    "------"
)

for row in all_rows:
    native = row["native"]
    compensated = row["compensated"]

    print(
        f"{row['altitude']:7.0f} ft  "
        f"pilot={row['pilot']:.2f}  "
        f"FF={native['feedforward']:.4f}  "
        f"pilot-FF="
        f"{row['pilot'] - native['feedforward']:+.4f}  "
        f"Pamb={native['pamb']:.2f}\"  "
        f"target={native['target']:.2f}\""
    )

    print(
        f"    native: "
        f"cmd={native['command']:.4f}  "
        f"pos={native['position']:.4f}  "
        f"MAP={native['map']:.2f}\"  "
        f"RPM={native['rpm']:.1f}  "
        f"HP={native['hp']:.1f}  "
        f"VE={native['ve']:.5f}  "
        f"blade={native['blade']:.2f}"
    )

    print(
        f"    VE-on:  "
        f"cmd={compensated['command']:.4f}  "
        f"pos={compensated['position']:.4f}  "
        f"MAP={compensated['map']:.2f}\"  "
        f"RPM={compensated['rpm']:.1f}  "
        f"HP={compensated['hp']:.1f}  "
        f"VE={compensated['ve']:.5f}  "
        f"blade={compensated['blade']:.2f}"
    )

    print(
        f"    delta:  "
        f"MAP={row['delta_map']:+.3f}\"  "
        f"HP={row['delta_hp']:+.3f}"
    )

print()
print(
    "NOTE"
)
print(
    "----"
)
print(
    "This is a characterization sweep, not a pass/fail test."
)
print(
    "The Stage-3A throttle controller remains disabled throughout."
)
print(
    "Do not infer a VE regime threshold until the paired results "
    "have been reviewed."
)
