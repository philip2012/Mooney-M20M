#!/usr/bin/env python3

from pathlib import Path

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

# Figure 17, Lycoming SI 1187J, Curve 13495-A.
#
# Digitized from the scanned straight-line chart to chart-reading
# precision. These are NOT printed Lycoming equations.
FIG17_SLOPE = 0.030
FIG17_MIN_INTERCEPT = 31.90
FIG17_NORMAL_INTERCEPT = 32.15
FIG17_MAX_INTERCEPT = 32.40

FIG17_MIN_TEMP_F = 70.0
FIG17_MAX_TEMP_F = 170.0

# Sea-level rated-point anchor already established for this project.
SEA_LEVEL_TARGET_MAP = 35.00

# Effective compressor-temperature exponent for gamma ~= 1.4.
TEMP_EXPONENT = 2.0 / 7.0

PSF_PER_INHG = 70.72620474785911

ALTITUDES_FT = (
    0.0,
    5000.0,
    10000.0,
    15000.0,
    18000.0,
    20000.0,
    22000.0,
    25000.0,
)


def get_atmosphere(altitude_ft):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)

    if not fdm.load_model_with_paths(
        MODEL,
        str(REPO),
        str(REPO / "Engines"),
        str(REPO / "Systems"),
        False,
    ):
        raise RuntimeError("could not load Mooney FDM")

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        altitude_ft,
    )
    fdm.set_property_value(
        "ic/h-agl-ft",
        4.30,
    )
    fdm.set_property_value(
        "ic/vg-kts",
        0.0,
    )

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    temp_r = fdm.get_property_value(
        "atmosphere/T-R"
    )

    pressure_psf = fdm.get_property_value(
        "atmosphere/P-psf"
    )

    pressure_inhg = (
        pressure_psf / PSF_PER_INHG
    )

    return temp_r, pressure_inhg


def fig17_min_map(temp_f):
    return (
        FIG17_MIN_INTERCEPT
        + FIG17_SLOPE * temp_f
    )


def fig17_normal_map(temp_f):
    return (
        FIG17_NORMAL_INTERCEPT
        + FIG17_SLOPE * temp_f
    )


def fig17_max_map(temp_f):
    return (
        FIG17_MAX_INTERCEPT
        + FIG17_SLOPE * temp_f
    )


def fig17_temperature_from_normal_map(map_inhg):
    return (
        (map_inhg - FIG17_NORMAL_INTERCEPT)
        / FIG17_SLOPE
    )


def controller_temp_f(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
    heat_offset_r,
):
    pressure_ratio = (
        map_inhg / ambient_pressure_inhg
    )

    compressed_temp_r = (
        ambient_temp_r
        * pressure_ratio ** TEMP_EXPONENT
    )

    return (
        compressed_temp_r
        + heat_offset_r
        - 459.67
    )


def residual(
    ambient_temp_r,
    ambient_pressure_inhg,
    map_inhg,
    heat_offset_r,
):
    temp_f = controller_temp_f(
        ambient_temp_r,
        ambient_pressure_inhg,
        map_inhg,
        heat_offset_r,
    )

    target = fig17_normal_map(
        temp_f
    )

    return map_inhg - target


def solve_normal_map(
    ambient_temp_r,
    ambient_pressure_inhg,
    heat_offset_r,
):
    low = 30.0
    high = 38.0

    f_low = residual(
        ambient_temp_r,
        ambient_pressure_inhg,
        low,
        heat_offset_r,
    )

    f_high = residual(
        ambient_temp_r,
        ambient_pressure_inhg,
        high,
        heat_offset_r,
    )

    if f_low * f_high > 0.0:
        return None

    for _ in range(50):
        mid = (
            low + high
        ) * 0.5

        f_mid = residual(
            ambient_temp_r,
            ambient_pressure_inhg,
            mid,
            heat_offset_r,
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


print("MOONEY AF1B FIGURE-17 THERMODYNAMIC DIAGNOSTIC")
print("==============================================")
print(f"JSBSim version: {jsbsim.__version__}")
print()
print("Figure 17 digitization:")
print(
    "  minimum MAP = "
    "31.90 + 0.030 * temperature_F"
)
print(
    "  normal  MAP = "
    "32.15 + 0.030 * temperature_F"
)
print(
    "  maximum MAP = "
    "32.40 + 0.030 * temperature_F"
)
print()

# ------------------------------------------------------------
# Calibrate ONE surrogate parameter from the sea-level anchor.
# ------------------------------------------------------------

sl_temp_r, sl_pressure_inhg = get_atmosphere(
    0.0
)

sl_controller_temp_f = (
    fig17_temperature_from_normal_map(
        SEA_LEVEL_TARGET_MAP
    )
)

sl_controller_temp_r = (
    sl_controller_temp_f + 459.67
)

sl_ideal_compressed_temp_r = (
    sl_temp_r
    * (
        SEA_LEVEL_TARGET_MAP
        / sl_pressure_inhg
    ) ** TEMP_EXPONENT
)

heat_offset_r = (
    sl_controller_temp_r
    - sl_ideal_compressed_temp_r
)

print("SEA-LEVEL CALIBRATION")
print("---------------------")
print(
    f"ambient pressure: "
    f"{sl_pressure_inhg:.3f} inHg"
)
print(
    f"ambient temperature: "
    f"{sl_temp_r - 459.67:.2f} F"
)
print(
    f"rated MAP anchor: "
    f"{SEA_LEVEL_TARGET_MAP:.2f} inHg"
)
print(
    f"Figure-17 normal temperature: "
    f"{sl_controller_temp_f:.2f} F"
)
print(
    f"surrogate heat offset: "
    f"{heat_offset_r:.2f} F"
)
print()

print("ALTITUDE SWEEP")
print("--------------")
print(
    " altitude    OAT F     Pamb"
    "    Tctrl    MAPmin    MAPnorm"
    "    MAPmax   chart"
)
print(
    " --------  -------  -------"
    "  -------  --------  --------"
    "  --------  -----"
)

for altitude_ft in ALTITUDES_FT:
    temp_r, pressure_inhg = get_atmosphere(
        altitude_ft
    )

    map_normal = solve_normal_map(
        temp_r,
        pressure_inhg,
        heat_offset_r,
    )

    if map_normal is None:
        print(
            f"{altitude_ft:8.0f}  "
            "NO SOLUTION"
        )
        continue

    temp_f = controller_temp_f(
        temp_r,
        pressure_inhg,
        map_normal,
        heat_offset_r,
    )

    map_min = fig17_min_map(
        temp_f
    )

    map_max = fig17_max_map(
        temp_f
    )

    in_chart = (
        FIG17_MIN_TEMP_F
        <= temp_f
        <= FIG17_MAX_TEMP_F
    )

    print(
        f"{altitude_ft:8.0f}  "
        f"{temp_r - 459.67:7.1f}  "
        f"{pressure_inhg:7.2f}  "
        f"{temp_f:7.1f}  "
        f"{map_min:8.2f}  "
        f"{map_normal:8.2f}  "
        f"{map_max:8.2f}  "
        f"{'YES' if in_chart else 'NO'}"
    )

print()
print("REFERENCE CHECK")
print("---------------")

temp_20_r, pressure_20 = get_atmosphere(
    20000.0
)

map_20 = solve_normal_map(
    temp_20_r,
    pressure_20,
    heat_offset_r,
)

temp_20_f = controller_temp_f(
    temp_20_r,
    pressure_20,
    map_20,
    heat_offset_r,
)

print(
    f"20,000 ft predicted normal MAP: "
    f"{map_20:.3f} inHg"
)
print(
    f"20,000 ft controller temperature: "
    f"{temp_20_f:.1f} F"
)
print()
print("FIGURE-17 THERMODYNAMIC DIAGNOSTIC COMPLETE")
