#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from math import sqrt

import jsbsim


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"

RATE_HZ = 120
DT = 1.0 / RATE_HZ

TEST_SECONDS = 12.0

# Increasing small-drop tests.
INITIAL_H_AGL = (
    4.25,
    4.35,
    4.50,
)

GEARS = (
    ("nose", 0),
    ("left-main", 1),
    ("right-main", 2),
)


def prop(fdm, name):
    return float(fdm.get_property_value(name))


def make_fdm(initial_h_agl):
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
        raise RuntimeError("Could not load Mooney M20M FDM")

    fdm.set_property_value(
        "ic/terrain-elevation-ft",
        0.0,
    )

    fdm.set_property_value(
        "ic/h-agl-ft",
        initial_h_agl,
    )

    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)

    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    # Gear down.
    fdm.set_property_value(
        "systems/airframe-controls/gear/handle",
        1.0,
    )

    # Hold brakes during the vertical test.
    for name in (
        "fcs/left-brake-cmd-norm",
        "fcs/right-brake-cmd-norm",
        "fcs/center-brake-cmd-norm",
    ):
        fdm.set_property_value(name, 1.0)

    # Engine safely inactive.
    for name, value in (
        (
            "systems/powerplant-controls/"
            "engine/handles/throttle-norm",
            0.0,
        ),
        (
            "systems/powerplant-controls/"
            "engine/handles/mixture-norm",
            0.0,
        ),
        (
            "systems/powerplant-controls/"
            "engine/switches/magnetos",
            0.0,
        ),
    ):
        try:
            fdm.set_property_value(name, value)
        except Exception:
            pass

    return fdm


def run_case(initial_h_agl):
    fdm = make_fdm(initial_h_agl)

    t = 0.0

    first_contact = None
    all_contact = None

    max_abs_hdot = 0.0

    min_pitch = float("inf")
    max_pitch = float("-inf")

    max_comp = {
        name: 0.0
        for name, _ in GEARS
    }

    max_abs_comp_vel = {
        name: 0.0
        for name, _ in GEARS
    }

    settle_candidate = None
    settled_at = None

    steps = int(TEST_SECONDS / DT)

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError(
                "JSBSim stopped unexpectedly"
            )

        t += DT

        hdot = prop(
            fdm,
            "velocities/h-dot-fps",
        )

        pitch = prop(
            fdm,
            "attitude/theta-deg",
        )

        max_abs_hdot = max(
            max_abs_hdot,
            abs(hdot),
        )

        min_pitch = min(
            min_pitch,
            pitch,
        )

        max_pitch = max(
            max_pitch,
            pitch,
        )

        wow_values = []
        comp_vel_values = []

        for name, index in GEARS:
            base = f"gear/unit[{index}]"

            wow = int(
                round(
                    prop(
                        fdm,
                        f"{base}/WOW",
                    )
                )
            )

            comp = prop(
                fdm,
                f"{base}/compression-ft",
            )

            comp_vel = prop(
                fdm,
                f"{base}/compression-velocity-fps",
            )

            wow_values.append(wow)
            comp_vel_values.append(comp_vel)

            max_comp[name] = max(
                max_comp[name],
                comp,
            )

            max_abs_comp_vel[name] = max(
                max_abs_comp_vel[name],
                abs(comp_vel),
            )

        if first_contact is None and any(wow_values):
            first_contact = t

        if all_contact is None and all(wow_values):
            all_contact = t

        # Require genuinely quiet suspension for
        # one continuous second.
        quiet = (
            all(wow_values)
            and abs(hdot) < 0.02
            and all(
                abs(v) < 0.01
                for v in comp_vel_values
            )
        )

        if quiet:
            if settle_candidate is None:
                settle_candidate = t

            elif (
                t - settle_candidate >= 1.0
                and settled_at is None
            ):
                settled_at = settle_candidate

        else:
            settle_candidate = None

    final_h = prop(
        fdm,
        "position/h-agl-ft",
    )

    final_pitch = prop(
        fdm,
        "attitude/theta-deg",
    )

    final_comp = {}

    for name, index in GEARS:
        final_comp[name] = (
            prop(
                fdm,
                f"gear/unit[{index}]/compression-ft",
            )
            * 12.0
        )

    return {
        "initial": initial_h_agl,
        "first_contact": first_contact,
        "all_contact": all_contact,
        "settled_at": settled_at,
        "max_hdot": max_abs_hdot,
        "pitch_range": max_pitch - min_pitch,
        "final_h": final_h,
        "final_pitch": final_pitch,
        "max_comp": {
            k: v * 12.0
            for k, v in max_comp.items()
        },
        "final_comp": final_comp,
        "max_comp_vel": max_abs_comp_vel,
    }


def fmt_time(value):
    if value is None:
        return "   ---"
    return f"{value:6.3f}"


def main():
    print(
        "Mooney M20M ground-contact drop diagnostic"
    )

    print(f"Repository: {REPO}")
    print("Production XML is NOT modified.")
    print(f"Rate: {RATE_HZ} Hz")
    print()

    print(
        "hAGL   first   allWOW   settle   "
        "max|hdot|  pitch_rng"
    )

    print(
        "-----  ------  -------  -------  "
        "---------  ---------"
    )

    results = []

    for h in INITIAL_H_AGL:
        result = run_case(h)
        results.append(result)

        print(
            f"{h:5.2f}  "
            f"{fmt_time(result['first_contact'])}  "
            f"{fmt_time(result['all_contact'])}  "
            f"{fmt_time(result['settled_at'])}  "
            f"{result['max_hdot']:9.4f}  "
            f"{result['pitch_range']:9.4f}"
        )

    for result in results:
        print()
        print("=" * 72)
        print(
            f"Initial h-AGL: "
            f"{result['initial']:.2f} ft"
        )
        print("=" * 72)

        print(
            f"final h-AGL: "
            f"{result['final_h']:.6f} ft"
        )

        print(
            f"final pitch: "
            f"{result['final_pitch']:+.6f} deg"
        )

        print()
        print(
            "gear         maxcomp(in) "
            "finalcomp(in) "
            "max|comp-v|(ft/s)"
        )

        for name, _ in GEARS:
            print(
                f"{name:<11} "
                f"{result['max_comp'][name]:>11.4f} "
                f"{result['final_comp'][name]:>13.4f} "
                f"{result['max_comp_vel'][name]:>17.5f}"
            )


if __name__ == "__main__":
    main()
