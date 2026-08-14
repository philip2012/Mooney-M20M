#!/usr/bin/env python3

from pathlib import Path
import os
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
ENGINE_FILE = "Lycoming-TIO-540-AF1B.xml"
PROP_FILE = "M20M-Propeller.xml"

DT = 1.0 / 120.0

PROP_IXX = float(
    os.environ.get(
        "AF1B_PROP_IXX",
        "2.15",
    )
)

BASE_VE = 0.865

INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

PSF_PER_INHG = 70.72620474785911

# Lycoming SI 1187J Figure 17 normal line,
# digitized to chart-reading precision.
FIG17_SLOPE = 0.030
FIG17_NORMAL_INTERCEPT = 32.15

TEMP_EXPONENT = 2.0 / 7.0

# Calibrated Figure-17 temperature surrogate.
# This is NOT asserted to be literal physical deck-temperature heat gain.
HEAT_OFFSET_R = 12.23

# ------------------------------------------------------------------
# Temporary FGPiston compensation surface.
#
# Independent variables are aligned with quantities involved in
# FGPiston's own airflow calculation:
#
#   x = normalized ambient temperature
#   m = ambient pressure / commanded MAP
#
# This remains an empirical FGPiston compensation surrogate,
# not physical Lycoming volumetric efficiency.
# ------------------------------------------------------------------

VE_A0 = 0.56412103
VE_M1 = 0.37457390
VE_M2 = -0.05280825
VE_X1 = 0.08601217
VE_XM = 0.01277322
VE_X2 = 0.00986585


# Conservative first dynamic-controller gains.
#
# These are diagnostic controller gains, not yet AF1B hardware data.
# Figure-17 feed-forward does most of the work.
#
# Do not apply proportional feedback immediately after a pilot-throttle
# step: MAP initially reflects the previous throttle condition, and a
# proportional kick drives the engine toward the native ~38 inHg ceiling.
KP = 0.0

# Slow trim only. This is still a diagnostic gain, not AF1B hardware data.
KI = 0.020

# Allow the manifold-pressure transient to develop before enabling
# feedback trim. Feed-forward remains active during this interval.
FEEDBACK_DELAY = 1.5

CONTROLLER_RATE_PER_SEC = 0.35

# Diagnostic only:
#   0.0 = instantaneous 0.55 -> 1.0 pilot-throttle step
#   1.0 = one-second pilot-throttle movement
PILOT_RAMP_SEC = float(
    os.environ.get(
        "AF1B_PILOT_RAMP_SEC",
        "0.0",
    )
)


TEST_CASES = (
    ("SL ISA",       0.0,      0.0),
    ("10K ISA-15",   10000.0, -15.0),
    ("10K ISA+15",   10000.0,  15.0),
    ("20K ISA-15",   20000.0, -15.0),
    ("20K ISA",      20000.0,   0.0),
    ("20K ISA+15",   20000.0,  15.0),
)


PILOT_THROTTLE = (
    "systems/powerplant-controls/engine/handles/throttle-norm"
)

ENGINE_THROTTLE = (
    "systems/powerplant-controls/engine/controller/"
    "throttle-to-engine-norm"
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

TEMP_R = "atmosphere/T-R"
PRESSURE_PSF = "atmosphere/P-psf"


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


def figure17_temperature_f(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
):
    pressure_ratio = (
        map_inhg
        / ambient_pressure_inhg
    )

    temp_r = (
        ambient_temp_r
        * pressure_ratio ** TEMP_EXPONENT
        + HEAT_OFFSET_R
    )

    return (
        temp_r - 459.67
    )


def figure17_map_from_temperature(
    temp_f,
):
    return (
        FIG17_NORMAL_INTERCEPT
        + FIG17_SLOPE * temp_f
    )


def map_residual(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
):
    temp_f = figure17_temperature_f(
        ambient_temp_r,
        ambient_pressure_inhg,
        map_inhg,
    )

    target = (
        figure17_map_from_temperature(
            temp_f
        )
    )

    return (
        map_inhg - target
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
            "Figure-17 MAP solution "
            "not bracketed"
        )

    for _ in range(50):
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


def effective_ve(
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


def throttle_feedforward(
    ambient_pressure_inhg,
    target_map,
):
    # Empirical starting estimate only.
    #
    # Closed-loop MAP feedback remains authoritative.
    pressure_ratio = (
        target_map
        / ambient_pressure_inhg
    )

    return clamp(
        0.43
        + 0.19 * pressure_ratio,
        0.25,
        1.0,
    )


def rate_limit(
    current,
    target,
    rate_per_second,
):
    maximum_delta = (
        rate_per_second * DT
    )

    delta = clamp(
        target - current,
        -maximum_delta,
        maximum_delta,
    )

    return (
        current + delta
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

    # --------------------------------------------------------------
    # Temporary engine candidate.
    # --------------------------------------------------------------

    engine_path = (
        root
        / "Engines"
        / ENGINE_FILE
    )

    engine_text = (
        engine_path.read_text()
    )

    old = (
        "    <air-intake-impedance-factor>"
        "1.0"
        "</air-intake-impedance-factor>\n"
    )

    if engine_text.count(old) != 1:
        raise RuntimeError(
            "Expected one explicit "
            "air-intake impedance setting"
        )

    engine_text = engine_text.replace(
        old,
        "",
        1,
    )

    old = (
        "<volumetric-efficiency>"
        "0.90"
        "</volumetric-efficiency>"
    )

    new = (
        "<volumetric-efficiency>"
        "0.865"
        "</volumetric-efficiency>"
    )

    if engine_text.count(old) != 1:
        raise RuntimeError(
            "Expected permanent VE=0.90 "
            "exactly once"
        )

    engine_text = engine_text.replace(
        old,
        new,
        1,
    )

    engine_path.write_text(
        engine_text
    )

    # --------------------------------------------------------------
    # Temporary propeller inertia candidate.
    #
    # Diagnostic sensitivity only. PROP_IXX is not being proposed
    # as permanent data here.
    # --------------------------------------------------------------

    prop_path = (
        root
        / "Engines"
        / PROP_FILE
    )

    prop_text = prop_path.read_text()

    old_ixx = (
        '<ixx unit="SLUG*FT2">'
        '2.15'
        '</ixx>'
    )

    new_ixx = (
        '<ixx unit="SLUG*FT2">'
        f'{PROP_IXX:.6f}'
        '</ixx>'
    )

    if prop_text.count(old_ixx) != 1:
        raise RuntimeError(
            "Expected propeller Ixx=2.15 "
            "exactly once"
        )

    prop_text = prop_text.replace(
        old_ixx,
        new_ixx,
        1,
    )

    prop_path.write_text(
        prop_text
    )

    # --------------------------------------------------------------
    # Temporary systems bridge.
    #
    # Pilot-facing throttle remains untouched.
    # Only the engine-facing routing is redirected through a local
    # controller property in this temporary copy.
    # --------------------------------------------------------------

    system_path = (
        root
        / "Systems"
        / "powerplant-controls.xml"
    )

    system_text = (
        system_path.read_text()
    )

    old = (
        '    <property value="0">'
        'systems/powerplant-controls/engine/'
        'handles/throttle-norm'
        '</property>\n'
    )

    new = (
        old
        + '    <property value="0">'
        'systems/powerplant-controls/engine/'
        'controller/throttle-to-engine-norm'
        '</property>\n'
    )

    if system_text.count(old) != 1:
        raise RuntimeError(
            "Could not find pilot throttle "
            "property initialization"
        )

    system_text = system_text.replace(
        old,
        new,
        1,
    )

    old = (
        "<input>"
        "systems/powerplant-controls/engine/"
        "handles/throttle-norm"
        "</input>"
    )

    new = (
        "<input>"
        "systems/powerplant-controls/engine/"
        "controller/throttle-to-engine-norm"
        "</input>"
    )

    if system_text.count(old) != 1:
        raise RuntimeError(
            "Could not find direct throttle "
            "routing exactly once"
        )

    system_text = system_text.replace(
        old,
        new,
        1,
    )

    system_path.write_text(
        system_text
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
        ENGINE_THROTTLE,
        0.55,
    )

    fdm.set_property_value(
        MAGNETOS,
        3,
    )

    fdm.set_property_value(
        VE_PROP,
        BASE_VE,
    )

    fdm.set_property_value(
        RAM_AIR,
        0.0,
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


def run_dynamic_case(
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

    target_map = (
        solve_figure17_map(
            ambient_temp_r,
            ambient_pressure_inhg,
        )
    )

    target_tref_f = (
        figure17_temperature_f(
            ambient_temp_r,
            ambient_pressure_inhg,
            target_map,
        )
    )

    ve_target = effective_ve(
        ambient_temp_r,
        ambient_pressure_inhg,
        target_map,
    )

    feedforward = (
        throttle_feedforward(
            ambient_pressure_inhg,
            target_map,
        )
    )

    # ----------------------------------------------------------
    # Precondition the full-power FGPiston compensation before the
    # throttle transient. This is diagnostic isolation only; it
    # prevents a simultaneous VE step from contaminating our
    # throttle/prop-governor transient measurement.
    # ----------------------------------------------------------

    fdm.set_property_value(
        VE_PROP,
        ve_target,
    )

    run_for(
        fdm,
        2.0,
    )

    initial_pilot_throttle = 0.55

    fdm.set_property_value(
        PILOT_THROTTLE,
        initial_pilot_throttle,
    )

    fdm.set_property_value(
        ENGINE_THROTTLE,
        initial_pilot_throttle,
    )

    # This is the density-controller CAP, not engine throttle.
    # It stays pre-positioned while pilot throttle can remain below it.
    controller_cap = feedforward

    integral = 0.0

    duration = 15.0

    samples = []

    steps = int(
        duration / DT
    )

    for step in range(steps):
        time_s = (
            step * DT
        )

        if PILOT_RAMP_SEC <= 0.0:
            pilot_throttle = 1.0
        else:
            ramp_fraction = clamp(
                time_s / PILOT_RAMP_SEC,
                0.0,
                1.0,
            )

            pilot_throttle = (
                initial_pilot_throttle
                + (
                    1.0
                    - initial_pilot_throttle
                )
                * ramp_fraction
            )

        fdm.set_property_value(
            PILOT_THROTTLE,
            pilot_throttle,
        )

        current_map = get(
            fdm,
            MAP,
        )

        error = (
            target_map
            - current_map
        )

        # Bumpless activation:
        #
        # During the initial MAP transient, trust the calibrated
        # feed-forward cap and do not integrate the large artificial
        # error inherited from the previous partial-throttle state.
        if time_s < FEEDBACK_DELAY:
            candidate_integral = integral

            raw_command = (
                feedforward
            )
        else:
            candidate_integral = (
                integral
                + KI * error * DT
            )

            raw_command = (
                feedforward
                + KP * error
                + candidate_integral
            )

        clipped_command = clamp(
            raw_command,
            0.25,
            1.0,
        )

        # Basic anti-windup.
        pushing_high = (
            raw_command > 1.0
            and error > 0.0
        )

        pushing_low = (
            raw_command < 0.25
            and error < 0.0
        )

        if not (
            pushing_high
            or pushing_low
        ):
            integral = (
                candidate_integral
            )

        # Move the CONTROLLER CAP independently of pilot throttle.
        controller_cap = rate_limit(
            controller_cap,
            clipped_command,
            CONTROLLER_RATE_PER_SEC,
        )

        # Pilot throttle may operate anywhere below the cap without
        # changing the controller's internally pre-positioned state.
        engine_throttle = min(
            pilot_throttle,
            controller_cap,
        )

        fdm.set_property_value(
            ENGINE_THROTTLE,
            engine_throttle,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "dynamic controller test"
            )

        samples.append(
            {
                "time": time_s,
                "map": get(
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
                "throttle": (
                    engine_throttle
                ),
            }
        )

    settled = [
        r
        for r in samples
        if r["time"] >= duration - 2.0
    ]

    def average(key):
        return (
            sum(
                r[key]
                for r in settled
            )
            / len(settled)
        )

    avg_map = average("map")
    avg_rpm = average("rpm")
    avg_hp = average("hp")
    avg_blade = average("blade")
    avg_j = average("j")
    avg_throttle = average(
        "throttle"
    )

    max_map = max(
        r["map"]
        for r in samples
    )

    peak_rpm_sample = max(
        samples,
        key=lambda r: r["rpm"],
    )

    max_rpm = peak_rpm_sample["rpm"]

    max_hp = max(
        r["hp"]
        for r in samples
    )

    max_blade = max(
        r["blade"]
        for r in samples
    )

    status = "PASS"

    reasons = []

    if abs(
        avg_map - target_map
    ) > 0.10:
        reasons.append("MAP")

    if abs(
        avg_rpm - 2575.0
    ) > 10.0:
        reasons.append("RPM")

    if abs(
        avg_hp - 270.0
    ) > 2.0:
        reasons.append("POWER")

    if avg_blade >= 44.45:
        reasons.append(
            "PROP LIMIT"
        )

    if reasons:
        status = ",".join(
            reasons
        )

    return {
        "name": name,
        "alt": altitude_ft,
        "bias": temp_bias_f,
        "oat": ambient_temp_r - 459.67,
        "pamb": ambient_pressure_inhg,
        "tref": target_tref_f,
        "target_map": target_map,
        "ve": ve_target,
        "feedforward": feedforward,
        "map": avg_map,
        "rpm": avg_rpm,
        "hp": avg_hp,
        "blade": avg_blade,
        "j": avg_j,
        "throttle": avg_throttle,
        "max_map": max_map,
        "max_rpm": max_rpm,
        "max_hp": max_hp,
        "max_blade": max_blade,
        "peak_rpm_time": peak_rpm_sample["time"],
        "peak_rpm_map": peak_rpm_sample["map"],
        "peak_rpm_hp": peak_rpm_sample["hp"],
        "peak_rpm_blade": peak_rpm_sample["blade"],
        "peak_rpm_throttle": peak_rpm_sample["throttle"],
        "status": status,
        "airbox": get(
            fdm,
            AIRBOX,
        ),
        "gph": get(
            fdm,
            FUEL_GPH,
        ),
    }


print(
    "MOONEY AF1B DYNAMIC CONTROLLER PROTOTYPE"
)
print(
    "========================================"
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    f"controller gains: "
    f"Kp={KP:.3f}, Ki={KI:.3f}"
)
print(
    f"controller rate: "
    f"{CONTROLLER_RATE_PER_SEC:.2f}/s"
)
print(
    f"feedback delay: "
    f"{FEEDBACK_DELAY:.2f} s"
)
print(
    f"pilot throttle ramp: "
    f"{PILOT_RAMP_SEC:.2f} s"
)
print(
    f"temporary prop Ixx: "
    f"{PROP_IXX:.3f} slug*ft^2"
)
print(
    "native turbo ceiling: repo 8.1 inHg "
    "rated-boost configuration"
)
print()

print(
    " case         alt   dT     OAT"
    "    MAPcmd  MAPavg    VEeff"
    "    thr     RPM      HP"
    "   blade  overshoot  status"
)
print(
    " ----------  -----  ----  ------"
    "  ------  ------  -------"
    "  -----  -------  ------"
    "  ------  ---------  ------"
)

with tempfile.TemporaryDirectory() as tmp:
    test_root = prepare_test_tree(
        Path(tmp)
    )

    for case in TEST_CASES:
        result = run_dynamic_case(
            test_root,
            *case,
        )

        overshoot = (
            result["max_map"]
            - result["target_map"]
        )

        print(
            f" {result['name']:<10}  "
            f"{result['alt']:5.0f}  "
            f"{result['bias']:+4.0f}  "
            f"{result['oat']:6.1f}  "
            f"{result['target_map']:6.2f}  "
            f"{result['map']:6.2f}  "
            f"{result['ve']:7.5f}  "
            f"{result['throttle']:5.3f}  "
            f"{result['rpm']:7.1f}  "
            f"{result['hp']:6.2f}  "
            f"{result['blade']:6.2f}  "
            f"{overshoot:+9.3f}  "
            f"{result['status']}"
        )

        print(
            f"              "
            f"Pamb={result['pamb']:.2f}\"  "
            f"Tref={result['tref']:.1f}F  "
            f"J={result['j']:.3f}  "
            f"Zair={result['airbox']:.5f}  "
            f"GPH={result['gph']:.2f}  "
            f"maxRPM={result['max_rpm']:.1f}  "
            f"maxHP={result['max_hp']:.1f}  "
            f"maxBlade={result['max_blade']:.2f}"
        )

        print(
            f"              "
            f"RPM peak @ {result['peak_rpm_time']:.3f}s: "
            f"MAP={result['peak_rpm_map']:.2f}\"  "
            f"HP={result['peak_rpm_hp']:.1f}  "
            f"blade={result['peak_rpm_blade']:.2f}deg  "
            f"thr={result['peak_rpm_throttle']:.3f}"
        )

print()
print(
    "DYNAMIC CONTROLLER PROTOTYPE COMPLETE"
)
