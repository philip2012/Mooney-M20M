#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
ENGINE_FILE = "Lycoming-TIO-540-AF1B.xml"

DT = 1.0 / 120.0

BASE_VE = 0.865
TARGET_HP = 270.0
TARGET_RPM = 2575.0

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

# Figure 17 digitization.
FIG17_SLOPE = 0.030
FIG17_NORMAL_INTERCEPT = 32.15

TEMP_EXPONENT = 2.0 / 7.0
PSF_PER_INHG = 70.72620474785911

SEA_LEVEL_MAP_ANCHOR = 35.00

ALTITUDES_FT = (
    0.0,
    5000.0,
    10000.0,
    15000.0,
    18000.0,
    20000.0,
)

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

TEMP_R = "atmosphere/T-R"
PRESSURE_PSF = "atmosphere/P-psf"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def fig17_normal_map(temp_f):
    return (
        FIG17_NORMAL_INTERCEPT
        + FIG17_SLOPE * temp_f
    )


def fig17_temperature_from_map(map_inhg):
    return (
        (map_inhg - FIG17_NORMAL_INTERCEPT)
        / FIG17_SLOPE
    )


def controller_temperature_f(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
    heat_offset_r,
):
    pressure_ratio = (
        map_inhg / ambient_pressure_inhg
    )

    temp_r = (
        ambient_temp_r
        * pressure_ratio ** TEMP_EXPONENT
        + heat_offset_r
    )

    return temp_r - 459.67


def map_residual(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
    heat_offset_r,
):
    temp_f = controller_temperature_f(
        ambient_temp_r,
        ambient_pressure_inhg,
        map_inhg,
        heat_offset_r,
    )

    return (
        map_inhg
        - fig17_normal_map(temp_f)
    )


def solve_controller_map(
    ambient_temp_r,
    ambient_pressure_inhg,
    heat_offset_r,
):
    low = 30.0
    high = 38.0

    f_low = map_residual(
        ambient_temp_r,
        ambient_pressure_inhg,
        low,
        heat_offset_r,
    )

    for _ in range(50):
        mid = (low + high) * 0.5

        f_mid = map_residual(
            ambient_temp_r,
            ambient_pressure_inhg,
            mid,
            heat_offset_r,
        )

        if f_low * f_mid <= 0.0:
            high = mid
        else:
            low = mid
            f_low = f_mid

    return (low + high) * 0.5


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

        text = text.replace(old, new)

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

        run_for(fdm, 0.25)


def make_fdm(test_root, altitude_ft):
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
        altitude_ft,
    )
    fdm.set_property_value("ic/h-agl-ft", 4.30)
    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)
    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    fdm.set_property_value("forces/hold-down", 1)

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
    fdm.set_property_value(THROTTLE, 0.60)
    fdm.set_property_value(MAGNETOS, 3)

    run_for(fdm, 0.1)

    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    fdm.set_property_value(
        VE_PROP,
        0.65,
    )

    fdm.set_property_value(
        RAM_AIR,
        0.0,
    )

    run_for(fdm, 1.5)

    if get(fdm, RPM) < 1000.0:
        raise RuntimeError(
            f"engine failed to establish at "
            f"{altitude_ft:.0f} ft"
        )

    ramp_inflow(fdm)
    run_for(fdm, 1.0)

    return fdm


def set_target_map(
    fdm,
    target_map,
):
    low = 0.25
    high = 1.0

    last_throttle = 1.0

    def settle_at(
        throttle,
        base_time=1.5,
    ):
        nonlocal last_throttle

        # Recovering RPM/boost after an increase in throttle can
        # take considerably longer than MAP's own pressure lag,
        # especially close to critical altitude.
        if throttle > last_throttle + 1.0e-6:
            settle_time = max(
                base_time,
                4.0,
            )
        else:
            settle_time = base_time

        fdm.set_property_value(
            THROTTLE,
            throttle,
        )

        run_for(
            fdm,
            settle_time,
        )

        last_throttle = throttle

        return get(fdm, MAP)

    # Establish actual full-throttle capability first.
    high_map = settle_at(
        high,
        4.0,
    )

    # Give RPM/boost another opportunity to stabilize near
    # critical altitude.
    for _ in range(2):
        previous = high_map

        high_map = settle_at(
            high,
            2.0,
        )

        if abs(
            high_map - previous
        ) < 0.005:
            break

    # This is not an error. At critical altitude a low-VE test
    # point may be unable to produce enough RPM/airflow for the
    # turbo to achieve the requested controller MAP.
    if high_map < target_map - 0.02:
        return None

    low_map = settle_at(
        low,
        2.0,
    )

    if low_map > target_map:
        raise RuntimeError(
            f"low throttle already gives "
            f"{low_map:.2f} inHg"
        )

    for _ in range(12):
        mid = (
            low + high
        ) * 0.5

        map_inhg = settle_at(
            mid,
            1.5,
        )

        if map_inhg < target_map:
            low = mid
        else:
            high = mid

    throttle = (
        low + high
    ) * 0.5

    settle_at(
        throttle,
        2.0,
    )

    return throttle


def evaluate_ve(
    test_root,
    altitude_ft,
    ve,
    target_map,
):
    fdm = make_fdm(
        test_root,
        altitude_ft,
    )

    fdm.set_property_value(
        VE_PROP,
        ve,
    )

    run_for(
        fdm,
        1.0,
    )

    throttle = set_target_map(
        fdm,
        target_map,
    )

    if throttle is None:
        # Target MAP is unavailable at this VE.
        # Leave the engine at stabilized full throttle and report
        # the capability-limited result. For VE root finding this
        # is unambiguously a lower-bound condition.
        fdm.set_property_value(
            THROTTLE,
            1.0,
        )

        run_for(
            fdm,
            3.0,
        )

        return {
            "reachable": False,
            "hp": get(fdm, POWER),
            "throttle": 1.0,
            "rpm": get(fdm, RPM),
            "map": get(fdm, MAP),
            "blade": get(fdm, BLADE),
            "gph": get(fdm, FUEL_GPH),
            "j": get(fdm, ADVANCE_RATIO),
            "boost": get(fdm, BOOST_LOSS),
            "airbox": get(fdm, AIRBOX),
        }

    run_for(
        fdm,
        2.0,
    )

    return {
        "reachable": True,
        "hp": get(fdm, POWER),
        "throttle": throttle,
        "rpm": get(fdm, RPM),
        "map": get(fdm, MAP),
        "blade": get(fdm, BLADE),
        "gph": get(fdm, FUEL_GPH),
        "j": get(fdm, ADVANCE_RATIO),
        "boost": get(fdm, BOOST_LOSS),
        "airbox": get(fdm, AIRBOX),
    }


def solve_ve(
    test_root,
    altitude_ft,
    target_map,
):
    low = 0.45
    high = 0.90

    low_result = evaluate_ve(
        test_root,
        altitude_ft,
        low,
        target_map,
    )

    high_result = evaluate_ve(
        test_root,
        altitude_ft,
        high,
        target_map,
    )

    def describe(
        ve,
        result,
    ):
        reach = (
            "OK"
            if result["reachable"]
            else "LIMIT"
        )

        return (
            f"VE {ve:.3f} -> "
            f"MAP {result['map']:.2f} / "
            f"{result['hp']:.2f} HP / "
            f"{result['rpm']:.1f} RPM / "
            f"thr {result['throttle']:.4f} / "
            f"{reach}"
        )

    print(
        f"  bounds @ {altitude_ft:.0f} ft: "
        + describe(
            low,
            low_result,
        )
        + "; "
        + describe(
            high,
            high_result,
        )
    )

    if (
        low_result["reachable"]
        and low_result["hp"] > TARGET_HP
    ):
        raise RuntimeError(
            f"VE lower bound already gives "
            f"{low_result['hp']:.2f} HP"
        )

    if not high_result["reachable"]:
        raise RuntimeError(
            "VE upper bound cannot reach "
            "controller MAP"
        )

    if high_result["hp"] < TARGET_HP:
        raise RuntimeError(
            f"VE upper bound only gives "
            f"{high_result['hp']:.2f} HP"
        )

    # Invariant:
    #
    # low  = either MAP capability limited or < target HP
    # high = MAP reachable and >= target HP
    for _ in range(14):
        mid = (
            low + high
        ) * 0.5

        result = evaluate_ve(
            test_root,
            altitude_ft,
            mid,
            target_map,
        )

        if not result["reachable"]:
            low = mid
            continue

        if result["hp"] < TARGET_HP:
            low = mid
        else:
            high = mid

    # Use the reachable side of the bracket.
    ve = high

    result = evaluate_ve(
        test_root,
        altitude_ft,
        ve,
        target_map,
    )

    if not result["reachable"]:
        raise RuntimeError(
            "final VE solution unexpectedly "
            "cannot reach target MAP"
        )

    return ve, result


# Establish the single Figure-17 thermal surrogate calibration
# from the standard sea-level 35.0 inHg anchor.
cal_fdm = jsbsim.FGFDMExec(None)
cal_fdm.set_debug_level(0)

if not cal_fdm.load_model_with_paths(
    MODEL,
    str(REPO),
    str(REPO / "Engines"),
    str(REPO / "Systems"),
    False,
):
    raise RuntimeError("could not load calibration FDM")

cal_fdm.set_property_value(
    "ic/terrain-elevation-ft",
    0.0,
)
cal_fdm.set_property_value(
    "ic/h-agl-ft",
    4.30,
)

if not cal_fdm.run_ic():
    raise RuntimeError("calibration run_ic failed")

sl_temp_r = get(
    cal_fdm,
    TEMP_R,
)

sl_pressure_inhg = (
    get(cal_fdm, PRESSURE_PSF)
    / PSF_PER_INHG
)

sl_reference_temp_f = (
    fig17_temperature_from_map(
        SEA_LEVEL_MAP_ANCHOR
    )
)

sl_reference_temp_r = (
    sl_reference_temp_f + 459.67
)

sl_compressed_temp_r = (
    sl_temp_r
    * (
        SEA_LEVEL_MAP_ANCHOR
        / sl_pressure_inhg
    ) ** TEMP_EXPONENT
)

heat_offset_r = (
    sl_reference_temp_r
    - sl_compressed_temp_r
)


print("MOONEY AF1B DENSITY/POWER COMPENSATION SWEEP")
print("============================================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"base VE:        {BASE_VE:.5f}")
print(f"target power:   {TARGET_HP:.1f} HP")
print(f"target RPM:     {TARGET_RPM:.0f}")
print(f"prop inflow:    {INFLOW_KTS:.0f} kt")
print(
    f"thermal offset: {heat_offset_r:.2f} F "
    "(calibrated surrogate)"
)
print()

print(
    " altitude   TrefF   MAPcmd   throttle"
    "     VEeff    VE/base      RPM"
    "    blade       HP      GPH"
)
print(
    " --------  ------  -------  --------"
    "  --------  ---------  -------"
    "  -------  -------  -------"
)

rows = []

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    for altitude in ALTITUDES_FT:
        fdm = make_fdm(
            test_root,
            altitude,
        )

        ambient_temp_r = get(
            fdm,
            TEMP_R,
        )

        ambient_pressure_inhg = (
            get(fdm, PRESSURE_PSF)
            / PSF_PER_INHG
        )

        target_map = solve_controller_map(
            ambient_temp_r,
            ambient_pressure_inhg,
            heat_offset_r,
        )

        tref_f = controller_temperature_f(
            ambient_temp_r,
            ambient_pressure_inhg,
            target_map,
            heat_offset_r,
        )

        ve, result = solve_ve(
            test_root,
            altitude,
            target_map,
        )

        row = {
            "alt": altitude,
            "tref": tref_f,
            "map": result["map"],
            "throttle": result["throttle"],
            "ve": ve,
            "ratio": ve / BASE_VE,
            "rpm": result["rpm"],
            "blade": result["blade"],
            "hp": result["hp"],
            "gph": result["gph"],
            "j": result["j"],
            "boost": result["boost"],
            "airbox": result["airbox"],
        }

        rows.append(row)

        print(
            f"{row['alt']:8.0f}  "
            f"{row['tref']:6.1f}  "
            f"{row['map']:7.2f}  "
            f"{row['throttle']:8.4f}  "
            f"{row['ve']:8.5f}  "
            f"{row['ratio']:9.5f}  "
            f"{row['rpm']:7.1f}  "
            f"{row['blade']:7.2f}  "
            f"{row['hp']:7.2f}  "
            f"{row['gph']:7.2f}"
        )


# Least-squares diagnostic only.
n = len(rows)

mean_t = sum(
    r["tref"] for r in rows
) / n

mean_ve = sum(
    r["ve"] for r in rows
) / n

denom = sum(
    (r["tref"] - mean_t) ** 2
    for r in rows
)

slope = sum(
    (r["tref"] - mean_t)
    * (r["ve"] - mean_ve)
    for r in rows
) / denom

intercept = (
    mean_ve - slope * mean_t
)

max_error = max(
    abs(
        r["ve"]
        - (
            intercept
            + slope * r["tref"]
        )
    )
    for r in rows
)

print()
print("LINEAR-FIT DIAGNOSTIC")
print("---------------------")
print(
    "VEeff ~= "
    f"{intercept:.6f} "
    f"{slope:+.8f} * Tref_F"
)
print(
    f"maximum VE fit residual: "
    f"{max_error:.6f}"
)
print()

print("SANITY CHECKS")
print("-------------")

for r in rows:
    status = "PASS"

    if abs(r["rpm"] - TARGET_RPM) > 10.0:
        status = "RPM"

    if abs(r["hp"] - TARGET_HP) > 1.0:
        status = "POWER"

    if r["blade"] >= 44.45:
        status = "PROP LIMIT"

    print(
        f"{r['alt']:8.0f} ft  "
        f"J={r['j']:.3f}  "
        f"Zair={r['airbox']:.5f}  "
        f"{status}"
    )

print()
print("DENSITY/POWER COMPENSATION SWEEP COMPLETE")
