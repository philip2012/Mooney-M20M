#!/usr/bin/env python3

from pathlib import Path
import shutil
import tempfile

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

ALTITUDE_FT = 20000.0
INFLOW_KTS = 175.0
KTS_TO_FPS = 1.687809857

INITIAL_THROTTLE = 0.55
FINAL_THROTTLE = 1.00
RAMP_SEC = 1.0
PRE_HOLD_SEC = 3.0
POST_SEC = 6.0

BASE_IXX = 2.15

IXX_VALUES = (
    2.15,    # current baseline first
    1.6125,  # 0.75x
    1.075,   # 0.50x
    2.6875,  # 1.25x
    3.225,   # 1.50x
    4.30,    # 2.00x
    5.375,   # 2.50x
    6.45,    # 3.00x
)

RUNNING = "propulsion/engine[0]/set-running"
RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"

PILOT = (
    "systems/powerplant-controls/engine/handles/"
    "throttle-norm"
)

PROP = (
    "systems/powerplant-controls/engine/handles/"
    "prop-norm"
)

MIXTURE = (
    "systems/powerplant-controls/engine/handles/"
    "mixture-norm"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/switches/"
    "magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/"
    "switches/battery-master"
)

THROTTLE_CMD = "fcs/throttle-cmd-norm[0]"

LIMIT = (
    "systems/af1b-density-controller/"
    "throttle-limit-norm"
)

RAM_AIR = "propulsion/engine[0]/ram-air-factor"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )


def make_tree(root, ixx):
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
        '<ixx unit="SLUG*FT2">'
        '2.15'
        '</ixx>'
    )

    new = (
        '<ixx unit="SLUG*FT2">'
        f'{ixx:.6f}'
        '</ixx>'
    )

    if text.count(old) != 1:
        raise RuntimeError(
            "Expected permanent Ixx=2.15 once"
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

    fdm.set_property_value(
        "propulsion/set-running",
        0,
    )

    # set-running is the authoritative engine-state check.
    # Do not use a static RPM threshold here: this diagnostic is
    # intended to begin from an in-flight 175 kt condition.
    run_for(
        fdm,
        0.25,
    )

    if get(fdm, RUNNING) < 0.5:
        raise RuntimeError(
            "Engine 0 did not enter running state"
        )

    # Establish the intended in-flight propeller condition before
    # judging whether the transient precondition is valid.
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
        PRE_HOLD_SEC,
    )

    if get(fdm, RUNNING) < 0.5:
        raise RuntimeError(
            "Engine stopped while establishing inflight condition"
        )

    # We need a meaningful governor-working initial condition before
    # comparing Ixx transient response. This is deliberately checked
    # after inflow and settling, not during the static startup phase.
    if get(fdm, RPM) < 1500.0:
        raise RuntimeError(
            "Inflight transient precondition did not establish: "
            f"RPM={get(fdm, RPM):.1f}"
        )

    return fdm


def run_case(ixx):
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        make_tree(
            root,
            ixx,
        )

        fdm = make_fdm(root)

        initial = {
            "rpm": get(fdm, RPM),
            "blade": get(fdm, BLADE),
            "map": get(fdm, MAP),
            "hp": get(fdm, POWER),
        }

        start = fdm.get_sim_time()

        samples = []

        steps = int(
            (RAMP_SEC + POST_SEC)
            / DT
        )

        for _ in range(steps):
            elapsed = (
                fdm.get_sim_time()
                - start
            )

            fraction = min(
                1.0,
                elapsed / RAMP_SEC,
            )

            pilot = (
                INITIAL_THROTTLE
                + (
                    FINAL_THROTTLE
                    - INITIAL_THROTTLE
                )
                * fraction
            )

            fdm.set_property_value(
                PILOT,
                pilot,
            )

            if not fdm.run():
                raise RuntimeError(
                    "Simulation stopped"
                )

            samples.append({
                "time": (
                    fdm.get_sim_time()
                    - start
                ),
                "rpm": get(fdm, RPM),
                "blade": get(fdm, BLADE),
                "map": get(fdm, MAP),
                "hp": get(fdm, POWER),
                "cmd": get(
                    fdm,
                    THROTTLE_CMD,
                ),
                "limit": get(
                    fdm,
                    LIMIT,
                ),
            })

        peak = max(
            samples,
            key=lambda x: x["rpm"],
        )

        max_blade = max(
            samples,
            key=lambda x: x["blade"],
        )

        settled = samples[-1]

        return {
            "ixx": ixx,
            "initial": initial,
            "peak": peak,
            "max_blade": max_blade,
            "settled": settled,
        }


print(
    "MOONEY PROP IXX TRANSIENT SENSITIVITY SWEEP"
)
print(
    "==========================================="
)
print(
    f"JSBSim version: {jsbsim.__version__}"
)
print()
print(
    " Ixx    factor   initialRPM  peakRPM  "
    "overRPM  tPeak  bladePeak  maxBlade  "
    "settledRPM"
)
print(
    "------  ------   ----------  -------  "
    "-------  -----  ---------  --------  "
    "----------"
)

rows = []

for ixx in IXX_VALUES:
    try:
        row = run_case(ixx)
    except RuntimeError as exc:
        print(
            f"{ixx:6.3f}  "
            f"{ixx / BASE_IXX:6.2f}   "
            f"{'SETUP FAIL':>10}  "
            f"{'-':>7}  "
            f"{'-':>7}  "
            f"{'-':>5}  "
            f"{'-':>9}  "
            f"{'-':>8}  "
            f"{'-':>10}"
        )

        print(
            f"        setup failure: {exc}"
        )

        continue

    rows.append(row)

    peak = row["peak"]

    print(
        f"{ixx:6.3f}  "
        f"{ixx / BASE_IXX:6.2f}   "
        f"{row['initial']['rpm']:10.1f}  "
        f"{peak['rpm']:7.1f}  "
        f"{peak['rpm'] - 2575:+7.1f}  "
        f"{peak['time']:5.3f}  "
        f"{peak['blade']:9.3f}  "
        f"{row['max_blade']['blade']:8.3f}  "
        f"{row['settled']['rpm']:10.1f}"
    )

print()
print("DETAIL")
print("------")

for row in rows:
    p = row["peak"]

    print(
        f"Ixx={row['ixx']:.3f}: "
        f"peak={p['rpm']:.1f} RPM @ "
        f"{p['time']:.3f}s, "
        f"blade={p['blade']:.3f}, "
        f"MAP={p['map']:.2f}, "
        f"HP={p['hp']:.2f}"
    )

print()
print(
    "Permanent prop/FDM files modified: NO"
)
print(
    "PROP IXX TRANSIENT SENSITIVITY SWEEP COMPLETE"
)
