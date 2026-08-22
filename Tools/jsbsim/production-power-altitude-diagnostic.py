#!/usr/bin/env python3

from pathlib import Path
import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

KTS_TO_FPS = 1.687809857

INFLOW_KTS = 175.0

TEST_CASES = (
    (0.0,     0.0),
    (10000.0, 0.0),
    (15000.0, 0.0),
    (18000.0, 0.0),
    (19000.0, 0.0),
    (20000.0, 0.0),
    (22000.0, 0.0),
    (24000.0, 0.0),
    (25000.0, 0.0),

    # Temperature sensitivity around critical altitude.
    (20000.0, -15.0),
    (20000.0, +15.0),
    (22000.0, -15.0),
    (22000.0, +15.0),
)

THROTTLE = (
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

STARTER = (
    "systems/powerplant-controls/engine/"
    "switches/starter"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/"
    "switches/magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/"
    "switches/battery-master"
)

RUNNING = "propulsion/engine[0]/set-running"
RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
THRUST = "propulsion/engine[0]/thrust-lbs"

BASE = "systems/af1b-density-controller"

TARGET = BASE + "/target-map-inhg"
CONTROLLER_TARGET = (
    BASE + "/controller-target-map-inhg"
)
MAX_MAP = BASE + "/maximum-map-inhg"
THROTTLE_LIMIT = (
    BASE + "/throttle-limit-norm"
)

VE = (
    "propulsion/engine[0]/"
    "volumetric-efficiency"
)


def get(fdm, prop):
    return float(
        fdm.get_property_value(prop)
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


def make_fdm(
    altitude_ft,
    delta_t_c,
):
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(REPO),
        str(REPO / "Engines"),
        str(REPO / "Systems"),
        False,
    ):
        raise RuntimeError(
            "Could not load production FDM"
        )

    # JSBSim atmosphere/delta-T is a persistent
    # atmosphere-model temperature bias. The property
    # uses Fahrenheit/Rankine temperature-difference
    # units, so a Celsius delta is multiplied by 1.8.
    #
    # Set it before run_ic() so the complete atmosphere
    # state is initialized consistently.
    fdm.set_property_value(
        "atmosphere/delta-T",
        delta_t_c * 1.8,
    )

    # Put the aircraft on an artificial flat
    # surface at the requested pressure altitude.
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
        raise RuntimeError(
            "run_ic() failed"
        )

    # Hold the rigid body stationary while
    # exposing the propeller to flight inflow.
    fdm.set_property_value(
        "forces/hold-down",
        1.0,
    )

    fdm.set_property_value(
        "atmosphere/wind-north-fps",
        -INFLOW_KTS * KTS_TO_FPS,
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
        1.0,
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
        THROTTLE,
        0.15,
    )

    fdm.set_property_value(
        MAGNETOS,
        3.0,
    )

    fdm.set_property_value(
        STARTER,
        1.0,
    )

    start = fdm.get_sim_time()

    while get(fdm, RUNNING) < 0.5:
        if (
            fdm.get_sim_time()
            - start
            > 8.0
        ):
            raise RuntimeError(
                "Engine failed to start"
            )

        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped during start"
            )

    fdm.set_property_value(
        STARTER,
        0.0,
    )

    # Ramp to full pilot throttle after start.
    for i in range(1, 121):
        fdm.set_property_value(
            THROTTLE,
            0.15
            + 0.85 * i / 120.0,
        )

        if not fdm.run():
            raise RuntimeError(
                "Throttle ramp failed"
            )

    run_for(
        fdm,
        12.0,
    )

    return fdm


def run_case(
    altitude_ft,
    delta_t_c,
):
    fdm = make_fdm(
        altitude_ft,
        delta_t_c,
    )

    return {
        "alt": altitude_ft,
        "dt": delta_t_c,
        "oat_f": get(fdm, "atmosphere/T-R") - 459.67,
        "rpm": get(fdm, RPM),
        "map": get(fdm, MAP),
        "hp": get(fdm, POWER),
        "thrust": get(fdm, THRUST),
        "target": get(
            fdm,
            TARGET,
        ),
        "ctrl_target": get(
            fdm,
            CONTROLLER_TARGET,
        ),
        "max_map": get(
            fdm,
            MAX_MAP,
        ),
        "limit": get(
            fdm,
            THROTTLE_LIMIT,
        ),
        "ve": get(
            fdm,
            VE,
        ),
    }


def main():
    print(
        "MOONEY M20M PRODUCTION "
        "ALTITUDE POWER DIAGNOSTIC"
    )

    print("=" * 94)

    print(
        f"JSBSim version: "
        f"{jsbsim.__version__}"
    )

    print(
        f"Prop inflow: "
        f"{INFLOW_KTS:.0f} kt"
    )

    print(
        "Permanent production XML only."
    )

    print()

    rows = []

    for alt, dt in TEST_CASES:
        rows.append(
            run_case(
                alt,
                dt,
            )
        )

    print(
        " alt    dT     OAT     RPM     MAP      HP"
        "   target  ctrlT   maxMAP"
        "   limit      VE"
    )

    print(
        "-----  ----  ------  ------  ------  ------"
        "  ------  ------  ------"
        "  ------  -------"
    )

    for r in rows:
        print(
            f"{r['alt']:5.0f} "
            f"{r['dt']:+4.0f} "
            f"{r['oat_f']:7.1f} "
            f"{r['rpm']:7.1f} "
            f"{r['map']:7.2f} "
            f"{r['hp']:7.2f} "
            f"{r['target']:7.2f} "
            f"{r['ctrl_target']:7.2f} "
            f"{r['max_map']:7.2f} "
            f"{r['limit']:7.3f} "
            f"{r['ve']:8.5f}"
        )


if __name__ == "__main__":
    main()
