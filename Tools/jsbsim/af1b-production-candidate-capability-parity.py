#!/usr/bin/env python3

"""
AF1B Stage-5C internal capability parity test.

Purpose:
Compare the Stage-5C XML-produced maximum-map-inhg against
the already-qualified external Stage-5B candidate law:

    Figure-3-34 ISA MAP
    * Pamb(non-ISA) / Pamb(ISA)

This test does NOT qualify engine power or controller dynamics.
It isolates the Stage-5C XML implementation only.

Permanent aircraft XML is untouched.
"""

from pathlib import Path
import shutil
import tempfile

import jsbsim

from af1b_lycoming_reference import (
    figure_3_34_post_critical_map,
)


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"

SYSTEM_XML = (
    REPO
    / "Tools"
    / "jsbsim"
    / "af1b-density-controller-production-candidate.xml"
)

DT = 1.0 / 120.0

PSF_PER_INHG = 70.72620474785911

ALTITUDES_FT = tuple(
    float(altitude)
    for altitude in range(
        19000,
        25001,
        125,
    )
)

TEMP_BIASES_F = (
    -15.0,
    0.0,
    15.0,
)

BASE = "systems/af1b-density-controller"

FIG34_INTERNAL = (
    BASE
    + "/figure34-isa-maximum-map-inhg"
)

ISA_PRESSURE_INTERNAL = (
    BASE
    + "/isa-pressure-inhg"
)

PRESSURE_SCALE_INTERNAL = (
    BASE
    + "/pressure-scale"
)

MAXIMUM_MAP_INTERNAL = (
    BASE
    + "/maximum-map-inhg"
)


def get(
    fdm,
    prop,
):
    return fdm.get_property_value(
        prop
    )


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


def prepare_test_tree(
    root,
):
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
            / "af1b-density-controller-production-candidate.xml"
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
        'file="af1b-density-controller-production-candidate" />\n'
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


def make_probe(
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
            "Could not load temporary Mooney FDM"
        )

    fdm.set_property_value(
        "atmosphere/delta-T",
        temp_bias_f,
    )

    # Exact ASL placement.
    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        0.0,
    )

    fdm.set_property_value(
        "ic/h-sl-ft",
        altitude_ft,
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

    # Let the FCS/system functions evaluate.
    for _ in range(5):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "Stage-5C parity probe"
            )

    actual_altitude = get(
        fdm,
        "position/h-sl-ft",
    )

    if abs(
        actual_altitude
        - altitude_ft
    ) > 0.05:
        raise RuntimeError(
            "Altitude mismatch: "
            f"requested={altitude_ft:.3f}, "
            f"actual={actual_altitude:.3f}"
        )

    return {
        "altitude": actual_altitude,
        "pamb": (
            get(
                fdm,
                "atmosphere/P-psf",
            )
            / PSF_PER_INHG
        ),
        "fig34": get(
            fdm,
            FIG34_INTERNAL,
        ),
        "isa_pressure": get(
            fdm,
            ISA_PRESSURE_INTERNAL,
        ),
        "pressure_scale": get(
            fdm,
            PRESSURE_SCALE_INTERNAL,
        ),
        "maximum_map": get(
            fdm,
            MAXIMUM_MAP_INTERNAL,
        ),
    }


print(
    "AF1B PRODUCTION-CANDIDATE CAPABILITY PARITY"
)
print(
    "=========================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print()
print(
    "External oracle:"
)
print(
    "  Figure-3-34 ISA MAP"
)
print(
    "  * Pamb(non-ISA) / Pamb(ISA)"
)
print()


rows = []

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)

    prepare_test_tree(
        root
    )

    # Independent ISA atmosphere probes at the exact same
    # ASL coordinates used for Stage-5C evaluation.
    external_isa_pressure = {}

    for altitude_ft in ALTITUDES_FT:
        probe = make_probe(
            root,
            altitude_ft,
            0.0,
        )

        external_isa_pressure[
            altitude_ft
        ] = probe["pamb"]

    for temp_bias_f in TEMP_BIASES_F:
        for altitude_ft in ALTITUDES_FT:
            probe = make_probe(
                root,
                altitude_ft,
                temp_bias_f,
            )

            external_fig34 = (
                figure_3_34_post_critical_map(
                    altitude_ft
                )
            )

            external_scale = (
                probe["pamb"]
                / external_isa_pressure[
                    altitude_ft
                ]
            )

            external_maximum = clamp(
                external_fig34
                * external_scale,
                0.0,
                38.0,
            )

            rows.append(
                {
                    "bias": temp_bias_f,
                    "alt": altitude_ft,
                    "pamb": probe["pamb"],
                    "internal_fig34": (
                        probe["fig34"]
                    ),
                    "external_fig34": (
                        external_fig34
                    ),
                    "internal_isa": (
                        probe["isa_pressure"]
                    ),
                    "external_isa": (
                        external_isa_pressure[
                            altitude_ft
                        ]
                    ),
                    "internal_scale": (
                        probe["pressure_scale"]
                    ),
                    "external_scale": (
                        external_scale
                    ),
                    "internal_max": (
                        probe["maximum_map"]
                    ),
                    "external_max": (
                        external_maximum
                    ),
                }
            )


print(
    " dT   Alt   Pamb    ISAint   ISAext"
    "    ScaleInt  ScaleExt"
    "    MaxInt   MaxExt   Delta"
)
print(
    "------------------------------------------------------"
    "----------------------------------------"
)

for row in rows:
    delta = (
        row["internal_max"]
        - row["external_max"]
    )

    print(
        f"{row['bias']:+3.0f}  "
        f"{row['alt']/1000:4.0f}K  "
        f"{row['pamb']:6.3f}  "
        f"{row['internal_isa']:7.4f}  "
        f"{row['external_isa']:7.4f}  "
        f"{row['internal_scale']:8.6f}  "
        f"{row['external_scale']:8.6f}  "
        f"{row['internal_max']:7.3f}  "
        f"{row['external_max']:7.3f}  "
        f"{delta:+.5f}"
    )


max_fig34_error = max(
    abs(
        row["internal_fig34"]
        - row["external_fig34"]
    )
    for row in rows
)

max_isa_pressure_error = max(
    abs(
        row["internal_isa"]
        - row["external_isa"]
    )
    for row in rows
)

max_scale_error = max(
    abs(
        row["internal_scale"]
        - row["external_scale"]
    )
    for row in rows
)

max_capability_error = max(
    abs(
        row["internal_max"]
        - row["external_max"]
    )
    for row in rows
)


print()
print(
    "MAXIMUM PARITY ERRORS"
)
print(
    "---------------------"
)
print(
    f"Figure-3-34 MAP: "
    f"{max_fig34_error:.8f} inHg"
)
print(
    f"ISA pressure:     "
    f"{max_isa_pressure_error:.8f} inHg"
)
print(
    f"pressure scale:   "
    f"{max_scale_error:.10f}"
)
print(
    f"maximum MAP:      "
    f"{max_capability_error:.8f} inHg"
)


# Harness tolerances, not Lycoming tolerances.
FIG34_TOL = 0.001
ISA_PRESSURE_TOL = 0.003
PRESSURE_SCALE_TOL = 0.00025
MAXIMUM_MAP_TOL = 0.01

passed = (
    max_fig34_error
    <= FIG34_TOL
    and max_isa_pressure_error
    <= ISA_PRESSURE_TOL
    and max_scale_error
    <= PRESSURE_SCALE_TOL
    and max_capability_error
    <= MAXIMUM_MAP_TOL
)


print()
print(
    "RESULT"
)
print(
    "------"
)

if passed:
    print(
        "AF1B STAGE-5C INTERNAL "
        "CAPABILITY PARITY PASS"
    )
else:
    print(
        "AF1B STAGE-5C INTERNAL "
        "CAPABILITY PARITY FAIL"
    )

    raise SystemExit(1)
