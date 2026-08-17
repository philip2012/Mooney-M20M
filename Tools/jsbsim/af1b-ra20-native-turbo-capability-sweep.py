#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim

from af1b_lycoming_reference import (
    figure_3_34_post_critical_map,
)


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

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

TEMP_BIASES = (
    -15.0,
    0.0,
    15.0,
)

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
TEST_INFLOW_KTS = 125.0

MAP = "propulsion/engine[0]/map-inhg"
RPM = "propulsion/engine[0]/propeller-rpm"
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


def ramp_inflow(fdm, final_kts):
    for kts in (
        25.0,
        50.0,
        75.0,
        100.0,
        125.0,
        150.0,
        175.0,
    ):
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


def ramp_throttle(fdm):
    steps = int(
        1.0 / DT
    )

    for step in range(steps):
        fraction = (
            (step + 1)
            / steps
        )

        throttle = (
            0.55
            + 0.45 * fraction
        )

        fdm.set_property_value(
            PILOT_THROTTLE,
            throttle,
        )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during "
                "throttle ramp"
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


def make_fdm(
    root,
    altitude_ft,
    temp_bias,
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
        temp_bias,
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

    # Lycoming engine charts under qualification are zero ram.
    fdm.set_property_value(
        RAM_AIR,
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
        TEST_INFLOW_KTS,
    )

    run_for(
        fdm,
        1.0,
    )

    # Native full throttle.
    # No density-controller XML is present in this test.
    ramp_throttle(
        fdm
    )

    run_for(
        fdm,
        5.0,
    )

    return fdm


def snapshot(fdm):
    return {
        "rpm": get(
            fdm,
            RPM,
        ),
        "map": get(
            fdm,
            MAP,
        ),
        "blade": get(
            fdm,
            BLADE,
        ),
        "pamb": (
            get(
                fdm,
                "atmosphere/P-psf",
            )
            / PSF_PER_INHG
        ),
        "temp_f": (
            get(
                fdm,
                "atmosphere/T-R",
            )
            - 459.67
        ),
    }


print(
    "MOONEY AF1B RA20 NATIVE "
    "TURBO CAPABILITY SWEEP"
)
print(
    "=============================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print(
    "ratedaltitude1: permanent 20000 ft"
)
print(
    "density controller: ABSENT"
)
print(
    "VE compensation: ABSENT"
)
print(
    "pilot throttle: 1.00"
)
print(
    "prop command: full RPM"
)
print(
    "engine ram-air-factor: 0.0"
)
print(
    f"artificial prop inflow: "
    f"{TEST_INFLOW_KTS:.1f} kt"
)
print()


rows = []

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
            "Expected permanent cp_factor=1.25 "
            "exactly once in temporary prop copy"
        )

    for temp_bias in TEMP_BIASES:
        for altitude_ft in ALTITUDES_FT:
            candidates = []

            for cp_factor in CP_FACTORS:
                prop_text = (
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

                prop_path.write_text(
                    prop_text
                )

                fdm = make_fdm(
                    root,
                    altitude_ft,
                    temp_bias,
                )

                candidates.append(
                    (
                        cp_factor,
                        snapshot(fdm),
                    )
                )

            qualified_candidates = [
                row
                for row in candidates
                if (
                    abs(
                        row[1]["rpm"]
                        - TARGET_RPM
                    ) <= 5.0
                    and row[1]["blade"] < 44.45
                )
            ]

            if not qualified_candidates:
                detail = ", ".join(
                    (
                        f"CP={cp_factor:.2f}: "
                        f"RPM={sample['rpm']:.1f}, "
                        f"blade={sample['blade']:.2f}"
                    )
                    for cp_factor, sample in candidates
                )

                raise RuntimeError(
                    "No uncontaminated 2575-RPM prop-load "
                    f"point at dT={temp_bias:+.0f}, "
                    f"altitude={altitude_ft:.0f}: "
                    + detail
                )

            best_factor, best = min(
                qualified_candidates,
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

            rpm_error = (
                best["rpm"]
                - TARGET_RPM
            )

            valid = True

            reference = None
            delta = None

            if temp_bias == 0.0:
                reference = (
                    figure_3_34_post_critical_map(
                        altitude_ft
                    )
                )

                delta = (
                    best["map"]
                    - reference
                )

            rows.append(
                (
                    temp_bias,
                    altitude_ft,
                    best_factor,
                    best,
                    rpm_error,
                    reference,
                    delta,
                    valid,
                )
            )


print(
    " dT    Alt  CPfac     RPM   Blade"
    "    Pamb   TempF    MAP    Ref   Delta  Q"
)
print(
    "------------------------------------------------"
    "------------------------------------"
)

for (
    temp_bias,
    altitude_ft,
    cp_factor,
    s,
    rpm_error,
    reference,
    delta,
    valid,
) in rows:
    if reference is None:
        ref_text = "   -- "
        delta_text = "   -- "
    else:
        ref_text = (
            f"{reference:5.2f}"
        )
        delta_text = (
            f"{delta:+6.2f}"
        )

    print(
        f"{temp_bias:+4.0f}  "
        f"{altitude_ft / 1000:4.0f}K  "
        f"{cp_factor:5.2f}  "
        f"{s['rpm']:7.1f}  "
        f"{s['blade']:6.2f}  "
        f"{s['pamb']:6.2f}  "
        f"{s['temp_f']:6.1f}  "
        f"{s['map']:6.2f}  "
        f"{ref_text}  "
        f"{delta_text}  "
        f"{'VALID' if valid else 'INVALID'}"
    )


if not all(
    row[-1]
    for row in rows
):
    raise SystemExit(
        "RA20 native capability sweep has "
        "invalid prop-load points"
    )

print()
print(
    "RA20 NATIVE TURBO CAPABILITY "
    "CHARACTERIZATION COMPLETE"
)
