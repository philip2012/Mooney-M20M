#!/usr/bin/env python3

"""
AF1B Stage-5A non-ISA capability-law comparison.

Diagnostic only.

This script DOES NOT alter the aircraft FDM or Stage-5A XML.

It compares:

A. FIXED:
   Figure-3-34 ISA capability independent of temperature.

B. PRESSURE-SCALED:
   Figure-3-34 ISA capability scaled by ambient-pressure
   ratio relative to the ISA case at the same altitude.

C. NATIVE-DELTA ORACLE:
   Figure-3-34 ISA capability plus the measured temperature
   shift of native JSBSim RA20 capability.

C is an oracle/benchmark, not a proposed production law.

The native capability measurements below are copied from the
qualified RA20 characterization produced by:

    af1b-ra20-native-turbo-capability-sweep.py

Values are intentionally limited to the precision printed by
that characterization. This comparison therefore identifies
architecture/trends, not final calibration constants.
"""

from af1b_lycoming_reference import (
    figure_3_34_post_critical_map,
)


# ----------------------------------------------------------
# Characterized native RA20 data.
#
# tuple:
#     ambient pressure inHg
#     ambient temperature deg F
#     native full-throttle MAP inHg
# ----------------------------------------------------------

NATIVE = {
    19000.0: {
        -15.0: (14.01, -23.7, 38.02),
          0.0: (14.34,  -8.7, 38.02),
         15.0: (14.66,   6.3, 38.02),
    },
    20000.0: {
        -15.0: (13.42, -27.3, 37.20),
          0.0: (13.76, -12.3, 38.02),
         15.0: (14.09,   2.7, 38.02),
    },
    21000.0: {
        -15.0: (12.85, -30.8, 35.62),
          0.0: (13.19, -15.8, 36.57),
         15.0: (13.53,  -0.8, 37.49),
    },
    22000.0: {
        -15.0: (12.30, -34.4, 34.09),
          0.0: (12.65, -19.4, 35.05),
         15.0: (12.98,  -4.4, 35.99),
    },
    23000.0: {
        -15.0: (11.77, -37.9, 32.62),
          0.0: (12.12, -22.9, 33.60),
         15.0: (12.46,  -7.9, 34.54),
    },
    24000.0: {
        -15.0: (11.25, -41.5, 31.20),
          0.0: (11.61, -26.5, 32.18),
         15.0: (11.95, -11.5, 33.13),
    },
    25000.0: {
        -15.0: (10.76, -45.1, 29.83),
          0.0: (11.12, -30.1, 30.81),
         15.0: (11.46, -15.1, 31.77),
    },
}


def clamp(value, low, high):
    return max(
        low,
        min(
            high,
            value,
        ),
    )


def figure17_target(
    ambient_inhg,
    ambient_f,
):
    """
    Solve the same fixed-point relation used by the Stage-5A XML:

        Tref,R =
            Tamb,R
            * (MAP / Pamb)^(2/7)
            + 12.23

        MAP =
            32.15
            + 0.030 * Tref,F

    Stage-5A clips the density target to 30..38 inHg.
    """

    ambient_r = (
        ambient_f
        + 459.67
    )

    target = 35.0

    for _ in range(100):
        reference_r = (
            ambient_r
            * (
                target
                / ambient_inhg
            ) ** (
                2.0 / 7.0
            )
            + 12.23
        )

        updated = (
            32.15
            + 0.030
            * (
                reference_r
                - 459.67
            )
        )

        updated = clamp(
            updated,
            30.0,
            38.0,
        )

        if abs(
            updated
            - target
        ) < 1.0e-10:
            return updated

        target = updated

    raise RuntimeError(
        "Figure-17 fixed-point iteration "
        "did not converge"
    )


def candidate_capabilities(
    altitude_ft,
    temp_bias,
):
    isa_cap = (
        figure_3_34_post_critical_map(
            altitude_ft
        )
    )

    pamb, temp_f, native = (
        NATIVE[altitude_ft][temp_bias]
    )

    isa_pamb, _, isa_native = (
        NATIVE[altitude_ft][0.0]
    )

    # A:
    # Current Stage-5A interpretation.
    fixed = isa_cap

    # B:
    # Preserve the ISA capability pressure ratio at a given
    # geometric altitude.
    pressure_scaled = (
        isa_cap
        * pamb
        / isa_pamb
    )

    pressure_scaled = clamp(
        pressure_scaled,
        0.0,
        38.0,
    )

    # C:
    # Preserve the measured native RA20 temperature delta.
    #
    # Oracle only. This is not proposed production logic.
    native_delta = (
        isa_cap
        + (
            native
            - isa_native
        )
    )

    native_delta = clamp(
        native_delta,
        0.0,
        38.0,
    )

    return (
        fixed,
        pressure_scaled,
        native_delta,
    )


print(
    "AF1B STAGE-5A NON-ISA "
    "CAPABILITY-LAW COMPARISON"
)
print(
    "=============================================="
)
print()
print(
    "A = fixed Figure-3-34 ISA ceiling"
)
print(
    "B = pressure-scaled Figure-3-34 ceiling"
)
print(
    "C = native-temperature-delta oracle"
)
print()


rows = []

for altitude_ft in sorted(NATIVE):
    isa_cap = (
        figure_3_34_post_critical_map(
            altitude_ft
        )
    )

    isa_native = (
        NATIVE[altitude_ft][0.0][2]
    )

    print(
        f"{altitude_ft / 1000:.0f}K FT"
    )
    print(
        " dT   Pamb   Native  Density"
        "     A       B       C"
        "    EffA    EffB    EffC"
    )
    print(
        "------------------------------------------------"
        "------------------------------"
    )

    for temp_bias in (
        -15.0,
        0.0,
        15.0,
    ):
        (
            pamb,
            temp_f,
            native,
        ) = NATIVE[
            altitude_ft
        ][
            temp_bias
        ]

        density = figure17_target(
            pamb,
            temp_f,
        )

        (
            cap_a,
            cap_b,
            cap_c,
        ) = candidate_capabilities(
            altitude_ft,
            temp_bias,
        )

        eff_a = min(
            density,
            cap_a,
        )

        eff_b = min(
            density,
            cap_b,
        )

        eff_c = min(
            density,
            cap_c,
        )

        native_shift = (
            native
            - isa_native
        )

        a_shift = (
            cap_a
            - isa_cap
        )

        b_shift = (
            cap_b
            - isa_cap
        )

        c_shift = (
            cap_c
            - isa_cap
        )

        b_oracle_error = (
            cap_b
            - cap_c
        )

        rows.append(
            {
                "altitude": altitude_ft,
                "temp_bias": temp_bias,
                "native_shift": native_shift,
                "a_shift": a_shift,
                "b_shift": b_shift,
                "c_shift": c_shift,
                "b_oracle_error": (
                    b_oracle_error
                ),
                "native": native,
                "density": density,
                "cap_a": cap_a,
                "cap_b": cap_b,
                "cap_c": cap_c,
                "eff_a": eff_a,
                "eff_b": eff_b,
                "eff_c": eff_c,
            }
        )

        print(
            f"{temp_bias:+3.0f}  "
            f"{pamb:5.2f}  "
            f"{native:6.2f}  "
            f"{density:7.2f}  "
            f"{cap_a:6.2f}  "
            f"{cap_b:6.2f}  "
            f"{cap_c:6.2f}  "
            f"{eff_a:6.2f}  "
            f"{eff_b:6.2f}  "
            f"{eff_c:6.2f}"
        )

    print()


# ----------------------------------------------------------
# Compare temperature-shift behavior above the crossover.
#
# 21K through 25K avoids the native 38-inHg clipping near
# critical altitude contaminating the comparison.
# ----------------------------------------------------------

postcritical = [
    row
    for row in rows
    if (
        row["altitude"]
        >= 21000.0
        and row["temp_bias"]
        != 0.0
    )
]


def mean_abs_error(
    key,
):
    return (
        sum(
            abs(
                row[key]
                - row["native_shift"]
            )
            for row in postcritical
        )
        / len(postcritical)
    )


a_mae = mean_abs_error(
    "a_shift"
)

b_mae = mean_abs_error(
    "b_shift"
)

c_mae = mean_abs_error(
    "c_shift"
)

max_b_oracle = max(
    abs(
        row["b_oracle_error"]
    )
    for row in postcritical
)


print(
    "POST-CRITICAL TEMPERATURE-SHIFT COMPARISON"
)
print(
    "------------------------------------------"
)
print(
    "21K through 25K, dT +/-15 F"
)
print()

print(
    "Mean absolute error versus measured "
    "native RA20 MAP shift:"
)
print(
    f"  A fixed:           {a_mae:.3f} inHg"
)
print(
    f"  B pressure-scaled: {b_mae:.3f} inHg"
)
print(
    f"  C native oracle:   {c_mae:.3f} inHg"
)
print()

print(
    "Maximum B-vs-C capability difference:"
)
print(
    f"  {max_b_oracle:.3f} inHg"
)
print()


# ----------------------------------------------------------
# Special critical-altitude inspection.
#
# Native RA20 is clipped at ~38.02 inHg around this region,
# so the native-delta oracle is not necessarily physically
# informative for the hot case.
# ----------------------------------------------------------

print(
    "20K CRITICAL-ALTITUDE INSPECTION"
)
print(
    "--------------------------------"
)

for row in rows:
    if row["altitude"] != 20000.0:
        continue

    print(
        f"dT={row['temp_bias']:+.0f}: "
        f"native={row['native']:.2f}  "
        f"density={row['density']:.2f}  "
        f"A={row['cap_a']:.2f}  "
        f"B={row['cap_b']:.2f}  "
        f"C={row['cap_c']:.2f}  "
        f"EffA={row['eff_a']:.2f}  "
        f"EffB={row['eff_b']:.2f}  "
        f"EffC={row['eff_c']:.2f}"
    )


print()
print(
    "INTERPRETATION"
)
print(
    "--------------"
)
print(
    "This script ranks candidate temperature responses only."
)
print(
    "It does NOT establish which non-ISA law is physically "
    "correct for the real AF1B installation."
)
print(
    "Candidate C is an oracle based on JSBSim's existing "
    "temperature response and must not be promoted directly "
    "to permanent aircraft logic."
)
