#!/usr/bin/env python3

from __future__ import annotations

from math import tan, radians
from pathlib import Path

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

RATE_HZ = 120
DT = 1.0 / RATE_HZ

SETTLE_TIME = 5.0
PRE_STEER_TIME = 0.50
TEST_TIME = 2.00
EVAL_START = 0.75

MAX_STEER_DEG = 13.0
WHEELBASE_FT = 79.563 / 12.0

FPS_PER_KT = 1.6878098571
G_FPS2 = 32.174

CASES = (
    (5.0, 0.25),
    (5.0, 0.50),
    (5.0, 1.00),

    (8.0, 0.25),
    (8.0, 0.50),
    (8.0, 1.00),

    (10.0, 0.25),
    (10.0, 0.50),
    (10.0, 1.00),

    # Mirror case for left/right symmetry.
    (8.0, -0.50),
)


def prop(fdm, name):
    return float(
        fdm.get_property_value(name)
    )


def make_fdm():
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    ok = fdm.load_model_with_paths(
        MODEL,
        str(REPO),
        str(REPO / "Engines"),
        str(REPO / "Systems"),
        False,
    )

    if not ok:
        raise RuntimeError(
            "Could not load Mooney FDM"
        )

    return fdm


def neutral_controls(fdm):
    fdm.set_property_value(
        "systems/powerplant-controls/"
        "engine/handles/throttle-norm",
        0.0,
    )

    fdm.set_property_value(
        "systems/powerplant-controls/"
        "engine/handles/mixture-norm",
        0.0,
    )

    for name in (
        "fcs/left-brake-cmd-norm",
        "fcs/right-brake-cmd-norm",
        "fcs/center-brake-cmd-norm",
        "fcs/rudder-cmd-norm",
        "fcs/steer-cmd-norm",
    ):
        fdm.set_property_value(
            name,
            0.0,
        )


def set_ic(
    fdm,
    agl_ft,
    pitch_deg,
    speed_kts,
):
    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        0.0,
    )

    fdm.set_property_value(
        "ic/h-agl-ft",
        agl_ft,
    )

    fdm.set_property_value(
        "ic/lat-gc-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/long-gc-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/phi-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/theta-deg",
        pitch_deg,
    )

    fdm.set_property_value(
        "ic/psi-true-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/vg-kts",
        speed_kts,
    )

    fdm.set_property_value(
        "ic/roc-fpm",
        0.0,
    )


def speed_kts(fdm):
    return (
        prop(
            fdm,
            "velocities/vg-fps",
        )
        / FPS_PER_KT
    )


def heading_delta(a, b):
    return (
        a - b + 180.0
    ) % 360.0 - 180.0


def wow(fdm):
    return [
        int(
            bool(
                prop(
                    fdm,
                    f"gear/unit[{i}]/WOW",
                )
            )
        )
        for i in range(3)
    ]


def settle_aircraft():
    fdm = make_fdm()
    neutral_controls(fdm)

    set_ic(
        fdm,
        agl_ft=4.30,
        pitch_deg=0.0,
        speed_kts=0.0,
    )

    if not fdm.run_ic():
        raise RuntimeError(
            "Static run_ic() failed"
        )

    while (
        fdm.get_sim_time()
        < SETTLE_TIME
    ):
        if not fdm.run():
            raise RuntimeError(
                "Static settle failed"
            )

    return (
        prop(
            fdm,
            "position/h-agl-ft",
        ),
        prop(
            fdm,
            "attitude/theta-deg",
        ),
    )


def run_case(
    settled_agl,
    settled_pitch,
    initial_speed,
    steer_cmd,
):
    fdm = make_fdm()
    neutral_controls(fdm)

    set_ic(
        fdm,
        settled_agl,
        settled_pitch,
        initial_speed,
    )

    if not fdm.run_ic():
        raise RuntimeError(
            "Rolling run_ic() failed"
        )

    while (
        fdm.get_sim_time()
        < PRE_STEER_TIME
    ):
        if not fdm.run():
            raise RuntimeError(
                "Pre-steer roll failed"
            )

    start_time = fdm.get_sim_time()

    start_heading = prop(
        fdm,
        "attitude/psi-deg",
    )

    fdm.set_property_value(
        "fcs/steer-cmd-norm",
        steer_cmd,
    )

    speeds = []
    yaw_rates = []

    max_roll = 0.0
    max_pitch_delta = 0.0
    max_main_diff = 0.0

    wow_dropouts = 0
    previous_wow = wow(fdm)

    actual_steer = 0.0

    while (
        fdm.get_sim_time()
        - start_time
        < TEST_TIME
    ):
        if not fdm.run():
            raise RuntimeError(
                "Steering test stopped"
            )

        elapsed = (
            fdm.get_sim_time()
            - start_time
        )

        roll = prop(
            fdm,
            "attitude/phi-deg",
        )

        pitch = prop(
            fdm,
            "attitude/theta-deg",
        )

        actual_steer = prop(
            fdm,
            "fcs/steer-pos-deg",
        )

        max_roll = max(
            max_roll,
            abs(roll),
        )

        max_pitch_delta = max(
            max_pitch_delta,
            abs(
                pitch
                - settled_pitch
            ),
        )

        left_comp = (
            prop(
                fdm,
                "gear/unit[1]/compression-ft",
            )
            * 12.0
        )

        right_comp = (
            prop(
                fdm,
                "gear/unit[2]/compression-ft",
            )
            * 12.0
        )

        max_main_diff = max(
            max_main_diff,
            abs(
                left_comp
                - right_comp
            ),
        )

        current_wow = wow(fdm)

        for old, new in zip(
            previous_wow,
            current_wow,
        ):
            if old == 1 and new == 0:
                wow_dropouts += 1

        previous_wow = current_wow

        if elapsed >= EVAL_START:
            speeds.append(
                speed_kts(fdm)
            )

            yaw_rates.append(
                prop(
                    fdm,
                    "velocities/r-rad_sec",
                )
            )

    final_heading = prop(
        fdm,
        "attitude/psi-deg",
    )

    mean_speed_kts = (
        sum(speeds) / len(speeds)
    )

    mean_speed_fps = (
        mean_speed_kts
        * FPS_PER_KT
    )

    mean_yaw = (
        sum(yaw_rates)
        / len(yaw_rates)
    )

    abs_mean_yaw = abs(
        mean_yaw
    )

    steer_rad = radians(
        actual_steer
    )

    expected_yaw = abs(
        mean_speed_fps
        / WHEELBASE_FT
        * tan(steer_rad)
    )

    if expected_yaw > 1e-9:
        yaw_ratio = (
            abs_mean_yaw
            / expected_yaw
        )
    else:
        yaw_ratio = 0.0

    if abs_mean_yaw > 1e-9:
        actual_radius = (
            mean_speed_fps
            / abs_mean_yaw
        )
    else:
        actual_radius = float("inf")

    if abs(tan(steer_rad)) > 1e-9:
        geometric_radius = abs(
            WHEELBASE_FT
            / tan(steer_rad)
        )
    else:
        geometric_radius = float(
            "inf"
        )

    lateral_accel_g = (
        mean_speed_fps
        * abs_mean_yaw
        / G_FPS2
    )

    return {
        "speed": initial_speed,
        "cmd": steer_cmd,
        "steer": actual_steer,
        "mean_speed": mean_speed_kts,
        "yaw": mean_yaw,
        "expected_yaw": expected_yaw,
        "ratio": yaw_ratio,
        "radius": actual_radius,
        "geo_radius": geometric_radius,
        "lat_g": lateral_accel_g,
        "heading": heading_delta(
            final_heading,
            start_heading,
        ),
        "roll": max_roll,
        "pitch_delta": max_pitch_delta,
        "main_diff": max_main_diff,
        "wow_dropouts": wow_dropouts,
        "final_wow": wow(fdm),
    }


def main():
    print(
        "MOONEY M20M STEERING / "
        "CORNERING SWEEP"
    )

    print("=" * 86)

    print(
        f"JSBSim rate: {RATE_HZ} Hz"
    )

    print(
        f"Wheelbase: "
        f"{WHEELBASE_FT:.4f} ft"
    )

    print(
        f"Max steering: "
        f"{MAX_STEER_DEG:.1f} deg"
    )

    print(
        "Production XML is NOT modified."
    )

    settled_agl, settled_pitch = (
        settle_aircraft()
    )

    print(
        f"Static equilibrium: "
        f"hAGL={settled_agl:.6f} ft, "
        f"pitch={settled_pitch:+.6f} deg"
    )

    print()

    results = []

    for speed, cmd in CASES:
        results.append(
            run_case(
                settled_agl,
                settled_pitch,
                speed,
                cmd,
            )
        )

    print(
        " V0   cmd  steer   Vavg   "
        "yaw     ideal   ratio  "
        "Ract   Rgeo   lat-g  "
        "roll   WOW"
    )

    print(
        "---- ----- ------ ------ "
        "------- ------- ------ "
        "------ ------ ------ "
        "------ ----"
    )

    for r in results:
        print(
            f"{r['speed']:4.1f} "
            f"{r['cmd']:+5.2f} "
            f"{r['steer']:+6.2f} "
            f"{r['mean_speed']:6.2f} "
            f"{r['yaw']:+7.3f} "
            f"{r['expected_yaw']:7.3f} "
            f"{r['ratio']:6.3f} "
            f"{r['radius']:6.1f} "
            f"{r['geo_radius']:6.1f} "
            f"{r['lat_g']:6.3f} "
            f"{r['roll']:6.3f} "
            f"{r['final_wow']}"
        )

    print()
    print("DETAIL")

    for r in results:
        print()
        print(
            f"{r['speed']:.1f} kt, "
            f"steer command "
            f"{r['cmd']:+.2f}"
        )

        print(
            f"  actual steer:      "
            f"{r['steer']:+.4f} deg"
        )

        print(
            f"  mean yaw rate:     "
            f"{r['yaw']:+.5f} rad/s"
        )

        print(
            f"  geometric yaw:     "
            f"{r['expected_yaw']:.5f} rad/s"
        )

        print(
            f"  yaw ratio:         "
            f"{r['ratio']:.4f}"
        )

        print(
            f"  actual radius:     "
            f"{r['radius']:.3f} ft"
        )

        print(
            f"  geometric radius:  "
            f"{r['geo_radius']:.3f} ft"
        )

        print(
            f"  lateral accel:     "
            f"{r['lat_g']:.4f} g"
        )

        print(
            f"  heading change:    "
            f"{r['heading']:+.4f} deg"
        )

        print(
            f"  max |roll|:        "
            f"{r['roll']:.4f} deg"
        )

        print(
            f"  max pitch delta:   "
            f"{r['pitch_delta']:.4f} deg"
        )

        print(
            f"  max main comp diff:"
            f" {r['main_diff']:.4f} in"
        )

        print(
            f"  WOW dropouts:      "
            f"{r['wow_dropouts']}"
        )


if __name__ == "__main__":
    main()
