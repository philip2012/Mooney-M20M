#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"

PROTOTYPE_XML = (
    REPO
    / "Tools"
    / "jsbsim"
    / "af1b-density-controller-stage1.xml"
)

DT = 1.0 / 120.0

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

PSF_PER_INHG = 70.72620474785911

FIG17_SLOPE = 0.030
FIG17_INTERCEPT = 32.15

TEMP_EXPONENT = 2.0 / 7.0
HEAT_OFFSET_R = 12.23

VE_A0 = 0.56412103
VE_M1 = 0.37457390
VE_M2 = -0.05280825
VE_X1 = 0.08601217
VE_XM = 0.01277322
VE_X2 = 0.00986585


TEST_CASES = (
    ("SL ISA",       0.0,      0.0),
    ("10K ISA-15",   10000.0, -15.0),
    ("10K ISA+15",   10000.0,  15.0),
    ("20K ISA-15",   20000.0, -15.0),
    ("20K ISA",      20000.0,   0.0),
    ("20K ISA+15",   20000.0,  15.0),
)


BASE = "systems/af1b-density-controller"

ENABLED = BASE + "/enabled"
PAMB_XML = BASE + "/ambient-pressure-inhg"
TREF_R_XML = BASE + "/figure17-reference-temperature-r"
TARGET_MAP_XML = BASE + "/target-map-inhg"

VE_CAL_XML = BASE + "/ve-calibrated"
VE_COMMAND_XML = BASE + "/ve-command"

CAP_XML = BASE + "/feedforward-cap-norm"
LIMITED_THROTTLE_XML = BASE + "/limited-throttle-norm"
ENGINE_COMMAND_XML = BASE + "/engine-throttle-command-norm"

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

VE_PROP = "propulsion/engine[0]/volumetric-efficiency"

RAM_AIR = "propulsion/engine[0]/ram-air-factor"

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"
ADVANCE_RATIO = "propulsion/engine[0]/advance-ratio"

TEMP_R = "atmosphere/T-R"
PRESSURE_PSF = "atmosphere/P-psf"

THROTTLE_COMMAND = "fcs/throttle-cmd-norm[0]"
THROTTLE_POSITION = "fcs/throttle-pos-norm[0]"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(
            maximum,
            value,
        ),
    )


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


def figure17_temperature_f(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
):
    return (
        ambient_temp_r
        * (
            map_inhg
            / ambient_pressure_inhg
        ) ** TEMP_EXPONENT
        + HEAT_OFFSET_R
        - 459.67
    )


def map_residual(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
):
    tref_f = figure17_temperature_f(
        ambient_temp_r,
        ambient_pressure_inhg,
        map_inhg,
    )

    target = (
        FIG17_INTERCEPT
        + FIG17_SLOPE * tref_f
    )

    return (
        map_inhg
        - target
    )


def solve_figure17_map(
    ambient_temp_r,
    ambient_pressure_inhg,
):
    low = 30.0
    high = 38.0

    f_low = map_residual(
        ambient_temp_r,
        ambient_pressure_inhg,
        low,
    )

    f_high = map_residual(
        ambient_temp_r,
        ambient_pressure_inhg,
        high,
    )

    if f_low * f_high > 0.0:
        raise RuntimeError(
            "Figure-17 solution not bracketed"
        )

    for _ in range(60):
        mid = (
            low + high
        ) * 0.5

        f_mid = map_residual(
            ambient_temp_r,
            ambient_pressure_inhg,
            mid,
        )

        if f_low * f_mid <= 0.0:
            high = mid
            f_high = f_mid
        else:
            low = mid
            f_low = f_mid

    return (
        low + high
    ) * 0.5


def expected_ve(
    ambient_temp_r,
    ambient_pressure_inhg,
    target_map,
):
    x = (
        ambient_temp_r - 500.0
    ) / 100.0

    m = (
        ambient_pressure_inhg
        / target_map
    )

    ve = (
        VE_A0
        + VE_M1 * m
        + VE_M2 * m * m
        + VE_X1 * x
        + VE_XM * x * m
        + VE_X2 * x * x
    )

    return clamp(
        ve,
        0.55,
        0.92,
    )


def expected_cap(
    ambient_pressure_inhg,
    target_map,
):
    return clamp(
        0.43
        + 0.19
        * (
            target_map
            / ambient_pressure_inhg
        ),
        0.25,
        1.0,
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
        PROTOTYPE_XML,
        (
            root
            / "Systems"
            / "af1b-density-controller-stage1.xml"
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
        'file="af1b-density-controller-stage1" />\n'
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

    return root


def make_fdm(
    test_root,
    altitude_ft,
    temp_bias_f,
):
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(test_root),
        str(test_root / "Engines"),
        str(test_root / "Systems"),
        False,
    ):
        raise RuntimeError(
            "could not load temporary FDM"
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

    # Match the existing AF1B calibration diagnostics.
    fdm.set_property_value(
        RAM_AIR,
        0.0,
    )

    fdm.set_property_value(
        ENABLED,
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


def run_case(
    test_root,
    name,
    altitude_ft,
    temp_bias_f,
):
    fdm = make_fdm(
        test_root,
        altitude_ft,
        temp_bias_f,
    )

    # Verify disabled fallback before activating the experiment.
    disabled_ve = get(
        fdm,
        VE_PROP,
    )

    disabled_command = get(
        fdm,
        ENGINE_COMMAND_XML,
    )

    disabled_position = get(
        fdm,
        THROTTLE_POSITION,
    )

    disabled_ok = (
        abs(disabled_ve - 0.90) < 1e-9
        and abs(disabled_command - 0.55) < 1e-9
        and abs(disabled_position - 0.55) < 1e-6
    )

    # Activate Stage 1 only for the full-power qualification.
    fdm.set_property_value(
        ENABLED,
        1,
    )

    fdm.set_property_value(
        PILOT_THROTTLE,
        1.0,
    )

    run_for(
        fdm,
        6.0,
    )

    ambient_temp_r = get(
        fdm,
        TEMP_R,
    )

    ambient_pressure_inhg = (
        get(
            fdm,
            PRESSURE_PSF,
        )
        / PSF_PER_INHG
    )

    expected_map = solve_figure17_map(
        ambient_temp_r,
        ambient_pressure_inhg,
    )

    expected_tref_f = (
        figure17_temperature_f(
            ambient_temp_r,
            ambient_pressure_inhg,
            expected_map,
        )
    )

    expected_ve_value = expected_ve(
        ambient_temp_r,
        ambient_pressure_inhg,
        expected_map,
    )

    expected_cap_value = expected_cap(
        ambient_pressure_inhg,
        expected_map,
    )

    xml_map = get(
        fdm,
        TARGET_MAP_XML,
    )

    xml_tref_f = (
        get(
            fdm,
            TREF_R_XML,
        )
        - 459.67
    )

    xml_ve = get(
        fdm,
        VE_CAL_XML,
    )

    engine_ve = get(
        fdm,
        VE_PROP,
    )

    xml_cap = get(
        fdm,
        CAP_XML,
    )

    limited_throttle = get(
        fdm,
        LIMITED_THROTTLE_XML,
    )

    engine_command = get(
        fdm,
        ENGINE_COMMAND_XML,
    )

    throttle_position = get(
        fdm,
        THROTTLE_POSITION,
    )

    math_ok = (
        abs(
            xml_map
            - expected_map
        ) < 0.001
        and abs(
            xml_tref_f
            - expected_tref_f
        ) < 0.01
        and abs(
            xml_ve
            - expected_ve_value
        ) < 0.00001
        and abs(
            xml_cap
            - expected_cap_value
        ) < 0.00001
    )

    routing_ok = (
        abs(
            engine_ve
            - xml_ve
        ) < 1e-9
        and abs(
            limited_throttle
            - min(
                1.0,
                xml_cap,
            )
        ) < 1e-9
        and abs(
            engine_command
            - limited_throttle
        ) < 1e-9
        and abs(
            throttle_position
            - engine_command
        ) < 1e-6
    )

    return {
        "name": name,
        "alt": altitude_ft,
        "bias": temp_bias_f,
        "oat": (
            ambient_temp_r
            - 459.67
        ),
        "pamb": ambient_pressure_inhg,
        "target_py": expected_map,
        "target_xml": xml_map,
        "tref_py": expected_tref_f,
        "tref_xml": xml_tref_f,
        "ve_py": expected_ve_value,
        "ve_xml": xml_ve,
        "cap_py": expected_cap_value,
        "cap_xml": xml_cap,
        "thr_pos": throttle_position,
        "map_actual": get(
            fdm,
            MAP,
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
        "j": get(
            fdm,
            ADVANCE_RATIO,
        ),
        "disabled_ok": disabled_ok,
        "math_ok": math_ok,
        "routing_ok": routing_ok,
        "fdm": fdm,
    }


print(
    "MOONEY AF1B DENSITY CONTROLLER "
    "STAGE-1 XML TEST"
)
print(
    "=========================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "Stage 1: fixed-point Figure-17 target, "
    "static feed-forward cap, temporary VE surface"
)
print()

all_pass = True

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    results = []

    for case in TEST_CASES:
        result = run_case(
            test_root,
            *case,
        )

        results.append(
            result
        )

        status = (
            result["disabled_ok"]
            and result["math_ok"]
            and result["routing_ok"]
        )

        if not status:
            all_pass = False

        print(
            f"{result['name']:<11} "
            f"alt={result['alt']:5.0f}  "
            f"dT={result['bias']:+4.0f}  "
            f"OAT={result['oat']:6.1f}F  "
            f"Pamb={result['pamb']:5.2f}\""
        )

        print(
            f"    target: "
            f"py={result['target_py']:7.4f}  "
            f"xml={result['target_xml']:7.4f}  "
            f"err="
            f"{result['target_xml'] - result['target_py']:+.5f}\""
        )

        print(
            f"    Tref:   "
            f"py={result['tref_py']:7.3f}F  "
            f"xml={result['tref_xml']:7.3f}F"
        )

        print(
            f"    VE:     "
            f"py={result['ve_py']:.6f}  "
            f"xml={result['ve_xml']:.6f}"
        )

        print(
            f"    cap:    "
            f"py={result['cap_py']:.6f}  "
            f"xml={result['cap_xml']:.6f}  "
            f"thrPos={result['thr_pos']:.6f}"
        )

        print(
            f"    engine: "
            f"MAP={result['map_actual']:6.2f}\"  "
            f"RPM={result['rpm']:7.1f}  "
            f"HP={result['hp']:7.2f}  "
            f"blade={result['blade']:6.2f}  "
            f"J={result['j']:.3f}"
        )

        print(
            "    checks: "
            f"disabled={'PASS' if result['disabled_ok'] else 'FAIL'}  "
            f"math={'PASS' if result['math_ok'] else 'FAIL'}  "
            f"routing={'PASS' if result['routing_ok'] else 'FAIL'}"
        )

        print()

    # ----------------------------------------------------------
    # Explicitly prove the pilot-throttle MIN behavior and the
    # known one-FCS-frame throttle-position latency.
    #
    # Do this after the SL qualification result has already been
    # captured. VE behavior during these two frames is irrelevant.
    # ----------------------------------------------------------

    fdm = results[0]["fdm"]

    previous_command = get(
        fdm,
        ENGINE_COMMAND_XML,
    )

    fdm.set_property_value(
        PILOT_THROTTLE,
        0.50,
    )

    if not fdm.run():
        raise RuntimeError(
            "JSBSim stopped during pilot-min frame 1"
        )

    command_frame_1 = get(
        fdm,
        ENGINE_COMMAND_XML,
    )

    position_frame_1 = get(
        fdm,
        THROTTLE_POSITION,
    )

    if not fdm.run():
        raise RuntimeError(
            "JSBSim stopped during pilot-min frame 2"
        )

    position_frame_2 = get(
        fdm,
        THROTTLE_POSITION,
    )

    pilot_min_ok = (
        abs(
            command_frame_1
            - 0.50
        ) < 1e-9
        and abs(
            position_frame_1
            - previous_command
        ) < 1e-6
        and abs(
            position_frame_2
            - 0.50
        ) < 1e-6
    )

    if not pilot_min_ok:
        all_pass = False

    print(
        "PILOT THROTTLE CAP ROUTING"
    )
    print(
        "--------------------------"
    )

    print(
        f"previous controller command: "
        f"{previous_command:.6f}"
    )

    print(
        f"frame 1 command:             "
        f"{command_frame_1:.6f}"
    )

    print(
        f"frame 1 throttle position:   "
        f"{position_frame_1:.6f}"
    )

    print(
        f"frame 2 throttle position:   "
        f"{position_frame_2:.6f}"
    )

    print(
        "pilot MIN + one-frame latency: "
        + (
            "PASS"
            if pilot_min_ok
            else "FAIL"
        )
    )

    print()

print(
    "RESULT"
)
print(
    "------"
)

if all_pass:
    print(
        "AF1B DENSITY CONTROLLER "
        "STAGE-1 XML TEST PASS"
    )
else:
    raise SystemExit(
        "AF1B DENSITY CONTROLLER "
        "STAGE-1 XML TEST FAIL"
    )
