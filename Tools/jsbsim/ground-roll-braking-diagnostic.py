#!/usr/bin/env python3

from __future__ import annotations

from math import degrees, pi
from pathlib import Path

import jsbsim


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"

RATE_HZ = 120
DT = 1.0 / RATE_HZ

MAX_SECONDS = 180.0
STOP_KTS = 0.50

# Previously validated static equilibrium.
INITIAL_H_AGL = 4.113112
INITIAL_PITCH_DEG = 0.109991

CASES = (
    (10.0, 0.00),
    (20.0, 0.00),
    (40.0, 0.00),
    (40.0, 0.25),
    (40.0, 0.50),
    (40.0, 1.00),
)

GEARS = (
    ("nose", 0),
    ("left-main", 1),
    ("right-main", 2),
)

KTS_TO_FPS = 1.687809857
FPS_TO_KTS = 1.0 / KTS_TO_FPS
G_FPS2 = 32.174


def prop(fdm, name):
    return float(
        fdm.get_property_value(name)
    )


def make_fdm(speed_kts, brake):
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
            "Could not load Mooney M20M FDM"
        )

    # Ideal flat standalone ground.
    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        0.0,
    )

    fdm.set_property_value(
        "ic/h-agl-ft",
        INITIAL_H_AGL,
    )

    fdm.set_property_value(
        "ic/phi-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/theta-deg",
        INITIAL_PITCH_DEG,
    )

    fdm.set_property_value(
        "ic/psi-true-deg",
        0.0,
    )

    fdm.set_property_value(
        "ic/vg-kts",
        speed_kts,
    )

    if not fdm.run_ic():
        raise RuntimeError(
            "run_ic() failed"
        )

    # Gear down.
    fdm.set_property_value(
        "systems/airframe-controls/"
        "gear/handle",
        1.0,
    )

    # Symmetric brakes.
    fdm.set_property_value(
        "fcs/left-brake-cmd-norm",
        brake,
    )

    fdm.set_property_value(
        "fcs/right-brake-cmd-norm",
        brake,
    )

    fdm.set_property_value(
        "fcs/center-brake-cmd-norm",
        brake,
    )

    return fdm


def get_speed_fps(fdm):
    return abs(
        prop(
            fdm,
            "velocities/vg-fps",
        )
    )


def get_sample(fdm, t):
    gears = {}

    for name, index in GEARS:
        base = f"gear/unit[{index}]"

        gears[name] = {
            "wow": int(
                round(
                    prop(
                        fdm,
                        f"{base}/WOW",
                    )
                )
            ),
            "comp_in": (
                prop(
                    fdm,
                    f"{base}/compression-ft",
                )
                * 12.0
            ),
        }

    return {
        "t": t,
        "speed_fps": get_speed_fps(fdm),
        "pitch_deg": prop(
            fdm,
            "attitude/theta-deg",
        ),
        "roll_deg": prop(
            fdm,
            "attitude/phi-deg",
        ),
        "heading_deg": prop(
            fdm,
            "attitude/psi-deg",
        ),
        "gears": gears,
    }


def unwrap_heading_delta(
    heading_deg,
    initial_heading_deg,
):
    delta = (
        heading_deg
        - initial_heading_deg
        + 180.0
    ) % 360.0 - 180.0

    return delta


def run_case(speed_kts, brake):
    fdm = make_fdm(
        speed_kts,
        brake,
    )

    samples = []

    distance_ft = 0.0

    max_decel_fps2 = 0.0
    decel_values = []

    initial_heading = None

    peak_pitch = float("-inf")
    min_pitch = float("inf")

    peak_roll_abs = 0.0
    peak_heading_abs = 0.0

    max_lr_comp_diff = 0.0

    wow_dropouts = {
        name: 0
        for name, _ in GEARS
    }

    previous_wow = {
        name: None
        for name, _ in GEARS
    }

    stopped = False
    stop_time = None

    max_steps = int(
        MAX_SECONDS / DT
    )

    previous_speed = None

    for step in range(max_steps):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )

        t = (step + 1) * DT

        sample = get_sample(
            fdm,
            t,
        )

        samples.append(sample)

        speed = sample["speed_fps"]

        if previous_speed is not None:
            distance_ft += (
                0.5
                * (
                    previous_speed
                    + speed
                )
                * DT
            )

            accel = (
                speed
                - previous_speed
            ) / DT

            decel = max(
                0.0,
                -accel,
            )

            decel_values.append(decel)

        previous_speed = speed

        pitch = sample["pitch_deg"]

        peak_pitch = max(
            peak_pitch,
            pitch,
        )

        min_pitch = min(
            min_pitch,
            pitch,
        )

        peak_roll_abs = max(
            peak_roll_abs,
            abs(sample["roll_deg"]),
        )

        if initial_heading is None:
            initial_heading = (
                sample["heading_deg"]
            )

        heading_delta = (
            unwrap_heading_delta(
                sample["heading_deg"],
                initial_heading,
            )
        )

        peak_heading_abs = max(
            peak_heading_abs,
            abs(heading_delta),
        )

        left_comp = (
            sample["gears"][
                "left-main"
            ]["comp_in"]
        )

        right_comp = (
            sample["gears"][
                "right-main"
            ]["comp_in"]
        )

        max_lr_comp_diff = max(
            max_lr_comp_diff,
            abs(
                left_comp
                - right_comp
            ),
        )

        for name, _ in GEARS:
            wow = (
                sample["gears"][name][
                    "wow"
                ]
            )

            if (
                previous_wow[name] == 1
                and wow == 0
            ):
                wow_dropouts[name] += 1

            previous_wow[name] = wow

        if (
            speed * FPS_TO_KTS
            <= STOP_KTS
        ):
            stopped = True
            stop_time = t
            break

    if decel_values:
        # Reject the most extreme single-step
        # numerical spikes by evaluating a
        # short moving average.
        window_steps = max(
            1,
            int(0.10 / DT),
        )

        smoothed = []

        if (
            len(decel_values)
            >= window_steps
        ):
            for i in range(
                len(decel_values)
                - window_steps
                + 1
            ):
                smoothed.append(
                    sum(
                        decel_values[
                            i:
                            i + window_steps
                        ]
                    )
                    / window_steps
                )
        else:
            smoothed = decel_values

        max_decel_fps2 = max(
            smoothed
        )

    final_speed_kts = (
        samples[-1]["speed_fps"]
        * FPS_TO_KTS
    )

    initial_speed_fps = (
        speed_kts * KTS_TO_FPS
    )

    final_speed_fps = (
        samples[-1]["speed_fps"]
    )

    elapsed = samples[-1]["t"]

    if distance_ft > 0.0:
        # Energy-equivalent average
        # longitudinal deceleration:
        #
        # v1^2 = v0^2 - 2 a s
        avg_decel_fps2 = max(
            0.0,
            (
                initial_speed_fps ** 2
                - final_speed_fps ** 2
            )
            / (
                2.0
                * distance_ft
            ),
        )
    else:
        avg_decel_fps2 = 0.0

    final = samples[-1]

    return {
        "speed_kts": speed_kts,
        "brake": brake,
        "stopped": stopped,
        "time": elapsed,
        "stop_time": stop_time,
        "distance_ft": distance_ft,
        "final_speed_kts": final_speed_kts,
        "avg_decel_fps2": avg_decel_fps2,
        "avg_decel_g": (
            avg_decel_fps2
            / G_FPS2
        ),
        "max_decel_fps2": max_decel_fps2,
        "max_decel_g": (
            max_decel_fps2
            / G_FPS2
        ),
        "pitch_min": min_pitch,
        "pitch_max": peak_pitch,
        "pitch_range": (
            peak_pitch
            - min_pitch
        ),
        "peak_roll_abs": peak_roll_abs,
        "peak_heading_abs": peak_heading_abs,
        "max_lr_comp_diff": (
            max_lr_comp_diff
        ),
        "wow_dropouts": wow_dropouts,
        "final_comp": {
            name:
            final["gears"][name][
                "comp_in"
            ]
            for name, _ in GEARS
        },
    }


def fmt_stop(result):
    if result["stopped"]:
        return "YES"

    return "NO"


def main():
    print(
        "MOONEY M20M GROUND ROLL / "
        "BRAKING DIAGNOSTIC"
    )

    print("=" * 78)

    print(
        f"JSBSim rate: {RATE_HZ} Hz"
    )

    print(
        f"Stop threshold: "
        f"{STOP_KTS:.2f} kt"
    )

    print(
        "Production XML is NOT modified."
    )

    print()

    results = []

    for speed_kts, brake in CASES:
        result = run_case(
            speed_kts,
            brake,
        )

        results.append(result)

    print(
        "speed brake stop    time   dist_ft "
        " avg_g  peak_g pitch_rng "
        "hdg_dev lr_comp"
    )

    print(
        "----- ----- ---- ------- -------- "
        "------ ------- --------- "
        "------- -------"
    )

    for r in results:
        print(
            f"{r['speed_kts']:5.1f} "
            f"{r['brake']:5.2f} "
            f"{fmt_stop(r):>4} "
            f"{r['time']:7.2f} "
            f"{r['distance_ft']:8.1f} "
            f"{r['avg_decel_g']:6.3f} "
            f"{r['max_decel_g']:7.3f} "
            f"{r['pitch_range']:9.4f} "
            f"{r['peak_heading_abs']:7.4f} "
            f"{r['max_lr_comp_diff']:7.4f}"
        )

    for r in results:
        print()
        print("=" * 78)

        print(
            f"{r['speed_kts']:.1f} kt, "
            f"brakes {r['brake']:.0%}"
        )

        print("=" * 78)

        print(
            f"stopped: "
            f"{fmt_stop(r)}"
        )

        print(
            f"elapsed: "
            f"{r['time']:.3f} s"
        )

        print(
            f"distance: "
            f"{r['distance_ft']:.3f} ft"
        )

        print(
            f"final speed: "
            f"{r['final_speed_kts']:.4f} kt"
        )

        print(
            f"average deceleration: "
            f"{r['avg_decel_fps2']:.4f} "
            f"ft/s² "
            f"({r['avg_decel_g']:.4f} g)"
        )

        print(
            f"peak 0.10-s deceleration: "
            f"{r['max_decel_fps2']:.4f} "
            f"ft/s² "
            f"({r['max_decel_g']:.4f} g)"
        )

        print(
            f"pitch min/max: "
            f"{r['pitch_min']:+.5f} / "
            f"{r['pitch_max']:+.5f} deg"
        )

        print(
            f"pitch range: "
            f"{r['pitch_range']:.5f} deg"
        )

        print(
            f"peak |roll|: "
            f"{r['peak_roll_abs']:.6f} deg"
        )

        print(
            f"peak heading deviation: "
            f"{r['peak_heading_abs']:.6f} deg"
        )

        print(
            "max left/right main "
            "compression difference: "
            f"{r['max_lr_comp_diff']:.6f} in"
        )

        print(
            "WOW dropouts: "
            f"nose={r['wow_dropouts']['nose']} "
            f"left={r['wow_dropouts']['left-main']} "
            f"right={r['wow_dropouts']['right-main']}"
        )

        print()

        print(
            "final compression:"
        )

        for name, _ in GEARS:
            print(
                f"  {name:<11} "
                f"{r['final_comp'][name]:.4f} in"
            )


if __name__ == "__main__":
    main()
