#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

ALTITUDE_FT = 20000.0
KTS_TO_FPS = 1.687809857

INITIAL_THROTTLE = 0.55
FINAL_THROTTLE = 1.00

BASE_CP_FACTOR = 1.25

CP_FACTORS = (
    1.000,
    1.125,
    1.250,   # permanent baseline
    1.375,
    1.500,
    1.625,
    1.750,
)

RAMP_TIMES = (
    1.0,
    4.0,
)

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"

PILOT = (
    "systems/powerplant-controls/engine/"
    "handles/throttle-norm"
)

PROP = (
    "systems/powerplant-controls/engine/"
    "handles/prop-norm"
)

MIXTURE = (
    "systems/powerplant-controls/engine/"
    "handles/mixture-norm"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/"
    "switches/magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/"
    "switches/battery-master"
)

RAM_AIR = "propulsion/engine[0]/ram-air-factor"


def get(fdm, name):
    return fdm.get_property_value(name)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


def prepare_tree(root, cp_factor):
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

    prop = (
        root
        / "Engines"
        / "M20M-Propeller.xml"
    )

    text = prop.read_text()

    old = (
        "<cp_factor>"
        "1.25"
        "</cp_factor>"
    )

    new = (
        "<cp_factor>"
        f"{cp_factor:.6f}"
        "</cp_factor>"
    )

    if text.count(old) != 1:
        raise RuntimeError(
            "Expected permanent cp_factor=1.25 once"
        )

    prop.write_text(
        text.replace(
            old,
            new,
            1,
        )
    )


def make_fdm(root):
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
        "ic/terrain-elevation-ft",
        ALTITUDE_FT - 4.30,
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
        PILOT,
        INITIAL_THROTTLE,
    )

    fdm.set_property_value(
        MAGNETOS,
        3,
    )

    run_for(
        fdm,
        0.1,
    )

    # Deterministic standalone engine startup.
    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    run_for(
        fdm,
        1.5,
    )

    # Establish the same 175 kt inflow condition.
    for kts in (
        25,
        50,
        75,
        100,
        125,
        150,
        175,
    ):
        fdm.set_property_value(
            "atmosphere/wind-north-fps",
            -kts * KTS_TO_FPS,
        )

        run_for(
            fdm,
            0.25,
        )

    run_for(
        fdm,
        3.0,
    )

    if get(fdm, RPM) < 1500.0:
        raise RuntimeError(
            "Transient initial condition "
            f"failed: RPM={get(fdm, RPM):.1f}"
        )

    return fdm


def run_case(cp_factor, ramp_sec):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        prepare_tree(
            root,
            cp_factor,
        )

        fdm = make_fdm(root)

        initial_rpm = get(
            fdm,
            RPM,
        )

        initial_blade = get(
            fdm,
            BLADE,
        )

        start = fdm.get_sim_time()

        samples = []

        duration = (
            ramp_sec
            + 6.0
        )

        steps = int(
            duration / DT
        )

        for _ in range(steps):
            elapsed = (
                fdm.get_sim_time()
                - start
            )

            fraction = min(
                1.0,
                elapsed / ramp_sec,
            )

            throttle = (
                INITIAL_THROTTLE
                + (
                    FINAL_THROTTLE
                    - INITIAL_THROTTLE
                )
                * fraction
            )

            fdm.set_property_value(
                PILOT,
                throttle,
            )

            if not fdm.run():
                raise RuntimeError(
                    "Simulation stopped "
                    "during transient"
                )

            samples.append({
                "time": (
                    fdm.get_sim_time()
                    - start
                ),
                "rpm": get(
                    fdm,
                    RPM,
                ),
                "blade": get(
                    fdm,
                    BLADE,
                ),
                "map": get(
                    fdm,
                    MAP,
                ),
                "hp": get(
                    fdm,
                    POWER,
                ),
            })

        peak = max(
            samples,
            key=lambda s: s["rpm"],
        )

        settled = samples[-1]

        return {
            "cp": cp_factor,
            "ramp": ramp_sec,
            "initial_rpm": initial_rpm,
            "initial_blade": initial_blade,
            "peak": peak,
            "settled": settled,
        }


print("MOONEY PROP CP-FACTOR TRANSIENT SWEEP")
print("=====================================")
print(f"JSBSim version: {jsbsim.__version__}")
print()
print(
    " CpFac factor ramp  initialRPM initialBlade "
    "peakRPM overRPM bladePeak settledRPM settledBlade"
)
print(
    " ----- ------ ----  ---------- ------------ "
    "------- ------- --------- ---------- ------------"
)

for ramp in RAMP_TIMES:
    for cp in CP_FACTORS:
        try:
            row = run_case(
                cp,
                ramp,
            )
        except RuntimeError as exc:
            print(
                f"{cp:6.3f} "
                f"{cp / BASE_CP_FACTOR:6.2f} "
                f"{ramp:4.1f}  SETUP FAIL: {exc}"
            )
            continue

        peak = row["peak"]
        settled = row["settled"]

        print(
            f"{cp:6.3f} "
            f"{cp / BASE_CP_FACTOR:6.2f} "
            f"{ramp:4.1f}  "
            f"{row['initial_rpm']:10.1f} "
            f"{row['initial_blade']:12.3f} "
            f"{peak['rpm']:7.1f} "
            f"{peak['rpm'] - 2575:+7.1f} "
            f"{peak['blade']:9.3f} "
            f"{settled['rpm']:10.1f} "
            f"{settled['blade']:12.3f}"
        )

print()
print("Permanent prop/FDM files modified: NO")
print("PROP CP-FACTOR TRANSIENT SWEEP COMPLETE")
