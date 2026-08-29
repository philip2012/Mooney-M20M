#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
ENGINE_FILE = "Lycoming-TIO-540-AF1B.xml"

DT = 1.0 / 120.0

ALTITUDE_FT = 20000.0
TARGET_MAP = 36.50
TARGET_HP = 270.0
TARGET_RPM = 2575.0

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

SETTLE_TIME = 2.0
BISECTION_STEPS = 14

THROTTLE = (
    "systems/powerplant-controls/engine/handles/throttle-norm"
)
MIXTURE = (
    "systems/powerplant-controls/engine/handles/mixture-norm"
)
PROP = (
    "systems/powerplant-controls/engine/handles/prop-norm"
)
MAGNETOS = (
    "systems/powerplant-controls/engine/switches/magnetos"
)
BATTERY = (
    "systems/powerplant-controls/electrical/switches/battery-master"
)

VE_PROP = "propulsion/engine[0]/volumetric-efficiency"
RAM_AIR = "propulsion/engine[0]/ram-air-factor"
AIRBOX = "propulsion/engine[0]/air-intake-impedance-factor"

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
FUEL_GPH = "propulsion/engine[0]/fuel-flow-rate-gph"
BLADE = "propulsion/engine[0]/blade-angle"
ADVANCE_RATIO = "propulsion/engine[0]/advance-ratio"
BOOST_LOSS = "propulsion/engine[0]/boostloss-hp"
AFR = "propulsion/engine[0]/AFR"

TEMP_R = "atmosphere/T-R"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def prepare_test_tree(root):
    shutil.copytree(
        REPO / "FDM",
        root / "FDM",
    )

    shutil.copytree(
        REPO / "Engines",
        root / "Engines",
    )

    path = root / "Engines" / ENGINE_FILE
    text = path.read_text()

    replacements = (
        (
            "    <air-intake-impedance-factor>"
            "1.0"
            "</air-intake-impedance-factor>\n",
            "",
        ),
        (
            '<ratedboost1 unit="INHG">8.1</ratedboost1>',
            '<ratedboost1 unit="INHG">6.58</ratedboost1>',
        ),
        (
            '<takeoffboost unit="INHG">8.1</takeoffboost>',
            '<takeoffboost unit="INHG">6.58</takeoffboost>',
        ),
    )

    for old, new in replacements:
        if text.count(old) != 1:
            raise RuntimeError(
                f"engine XML fragment not found exactly once:\n{old}"
            )

        text = text.replace(
            old,
            new,
        )

    path.write_text(text)

    return root


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

        run_for(fdm, 0.3)


def make_fdm(test_root):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(test_root),
        str(test_root / "Engines"),
        str(REPO / "Systems"),
        False,
    ):
        raise RuntimeError("could not load candidate FDM")

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        ALTITUDE_FT,
    )
    fdm.set_property_value("ic/h-agl-ft", 4.30)
    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)
    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

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

    fdm.set_property_value(BATTERY, 1)
    fdm.set_property_value(MIXTURE, 1.0)
    fdm.set_property_value(PROP, 1.0)
    fdm.set_property_value(THROTTLE, 1.0)
    fdm.set_property_value(MAGNETOS, 3)

    run_for(fdm, 0.1)

    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    # Start deliberately low so we do not hit the propeller
    # coarse-pitch stop while introducing test inflow.
    fdm.set_property_value(
        VE_PROP,
        0.60,
    )

    fdm.set_property_value(
        RAM_AIR,
        0.0,
    )

    run_for(fdm, 2.0)

    if get(fdm, RPM) < 1000.0:
        raise RuntimeError(
            "engine failed to establish before inflow ramp"
        )

    ramp_inflow(fdm)
    run_for(fdm, 2.0)

    return fdm


def set_ve(fdm, ve):
    fdm.set_property_value(
        VE_PROP,
        ve,
    )

    run_for(fdm, SETTLE_TIME)

    return get(fdm, POWER)


def snapshot(fdm, ve):
    return {
        "ve": ve,
        "temp_r": get(fdm, TEMP_R),
        "airbox": get(fdm, AIRBOX),
        "map": get(fdm, MAP),
        "rpm": get(fdm, RPM),
        "hp": get(fdm, POWER),
        "gph": get(fdm, FUEL_GPH),
        "blade": get(fdm, BLADE),
        "j": get(fdm, ADVANCE_RATIO),
        "boost": get(fdm, BOOST_LOSS),
        "afr": get(fdm, AFR),
    }


print("MOONEY AF1B 20K VE COMPENSATION DIAGNOSTIC")
print("===========================================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"altitude:       {ALTITUDE_FT:.0f} ft")
print(f"target MAP:     {TARGET_MAP:.2f} inHg")
print(f"target power:   {TARGET_HP:.1f} HP")
print(f"target RPM:     {TARGET_RPM:.0f}")
print(f"prop inflow:    {INFLOW_KTS:.0f} kt")
print()

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    fdm = make_fdm(
        test_root
    )

    print(
        "Initial candidate:"
    )
    r = snapshot(
        fdm,
        0.60,
    )

    print(
        f"VE={r['ve']:.4f} "
        f"MAP={r['map']:.2f} "
        f"RPM={r['rpm']:.1f} "
        f"HP={r['hp']:.2f} "
        f"blade={r['blade']:.2f}"
    )
    print()

    low = 0.50
    high = 0.865

    low_hp = set_ve(
        fdm,
        low,
    )

    high_hp = set_ve(
        fdm,
        high,
    )

    if low_hp > TARGET_HP:
        raise RuntimeError(
            f"VE lower bound already produces "
            f"{low_hp:.2f} HP"
        )

    if high_hp < TARGET_HP:
        raise RuntimeError(
            f"VE upper bound only produces "
            f"{high_hp:.2f} HP"
        )

    for _ in range(BISECTION_STEPS):
        mid = (
            low + high
        ) * 0.5

        hp = set_ve(
            fdm,
            mid,
        )

        if hp < TARGET_HP:
            low = mid
        else:
            high = mid

    ve = (
        low + high
    ) * 0.5

    set_ve(
        fdm,
        ve,
    )

    r = snapshot(
        fdm,
        ve,
    )

print("RESULT")
print("------")
print(f"effective VE: {r['ve']:.5f}")
print(f"Z_airbox:     {r['airbox']:.5f}")
print(f"MAP:          {r['map']:.2f} inHg")
print(f"RPM:          {r['rpm']:.1f}")
print(f"power:        {r['hp']:.2f} HP")
print(f"fuel flow:    {r['gph']:.2f} GPH")
print(f"AFR:          {r['afr']:.3f}")
print(f"blade:        {r['blade']:.2f} deg")
print(f"J:            {r['j']:.3f}")
print(f"boost loss:   {r['boost']:.2f} HP")
print()

base_ve = 0.865

correction = (
    r["ve"] / base_ve
)

equivalent_manifold_temp_r = (
    r["temp_r"] / correction
)

equivalent_manifold_temp_f = (
    equivalent_manifold_temp_r - 459.67
)

print("THERMODYNAMIC SURROGATE")
print("-----------------------")
print(f"base VE:              {base_ve:.5f}")
print(f"VE correction factor: {correction:.5f}")
print(
    "ambient temperature: "
    f"{r['temp_r'] - 459.67:.1f} F"
)
print(
    "equivalent manifold temperature: "
    f"{equivalent_manifold_temp_f:.1f} F"
)
