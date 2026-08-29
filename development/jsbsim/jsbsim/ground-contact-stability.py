#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from statistics import mean, pstdev

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

SETTLE_SECONDS = 12.0
SAMPLE_SECONDS = 10.0
INITIAL_H_AGL_FT = 4.35

RATES_HZ = (60, 120, 240)

GEARS = (
    ("nose", 0),
    ("left-main", 1),
    ("right-main", 2),
)


def get(fdm, prop):
    return float(fdm.get_property_value(prop))


def make_fdm(dt):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(dt)

    ok = fdm.load_model_with_paths(
        MODEL,
        str(REPO),
        str(REPO / "Engines"),
        str(REPO / "Systems"),
        False,
    )

    if not ok:
        raise RuntimeError("Could not load Mooney FDM")

    fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
    fdm.set_property_value("ic/h-agl-ft", INITIAL_H_AGL_FT)

    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)

    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    fdm.set_property_value(
        "systems/airframe-controls/gear/handle",
        1.0,
    )

    for prop in (
        "fcs/left-brake-cmd-norm",
        "fcs/right-brake-cmd-norm",
        "fcs/center-brake-cmd-norm",
    ):
        fdm.set_property_value(prop, 1.0)

    for prop, value in (
        (
            "systems/powerplant-controls/engine/handles/throttle-norm",
            0.0,
        ),
        (
            "systems/powerplant-controls/engine/handles/mixture-norm",
            0.0,
        ),
        (
            "systems/powerplant-controls/engine/switches/magnetos",
            0.0,
        ),
    ):
        try:
            fdm.set_property_value(prop, value)
        except Exception:
            pass

    return fdm


def run_seconds(fdm, seconds, dt):
    steps = int(round(seconds / dt))

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def stats(values):
    return {
        "mean": mean(values),
        "min": min(values),
        "max": max(values),
        "range": max(values) - min(values),
        "std": pstdev(values),
    }


def run_case(rate_hz):
    dt = 1.0 / rate_hz
    fdm = make_fdm(dt)

    # Short gear-state establishment period.
    run_seconds(fdm, 0.25, dt)

    # Normal drop and settling period.
    run_seconds(fdm, SETTLE_SECONDS, dt)

    data = {
        "h_dot": [],
        "pitch": [],
        "roll": [],
        "h_agl": [],
    }

    for name, index in GEARS:
        data[f"{name}_comp"] = []
        data[f"{name}_vel"] = []
        data[f"{name}_wow"] = []

    steps = int(round(SAMPLE_SECONDS / dt))

    for _ in range(steps):
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")

        data["h_dot"].append(
            get(fdm, "velocities/h-dot-fps")
        )

        data["pitch"].append(
            get(fdm, "attitude/theta-deg")
        )

        data["roll"].append(
            get(fdm, "attitude/phi-deg")
        )

        data["h_agl"].append(
            get(fdm, "position/h-agl-ft")
        )

        for name, index in GEARS:
            base = f"gear/unit[{index}]"

            data[f"{name}_comp"].append(
                get(fdm, f"{base}/compression-ft")
            )

            data[f"{name}_vel"].append(
                get(fdm, f"{base}/compression-velocity-fps")
            )

            data[f"{name}_wow"].append(
                int(round(get(fdm, f"{base}/WOW")))
            )

    print()
    print("=" * 78)
    print(f"{rate_hz} Hz")
    print("=" * 78)

    hdot = stats(data["h_dot"])
    pitch = stats(data["pitch"])
    roll = stats(data["roll"])
    hagl = stats(data["h_agl"])

    print(
        f"h-dot: mean={hdot['mean']:+.7f} ft/s  "
        f"range={hdot['range']:.7f}  "
        f"std={hdot['std']:.7f}"
    )

    print(
        f"pitch: mean={pitch['mean']:+.7f} deg  "
        f"range={pitch['range']:.7f}  "
        f"std={pitch['std']:.7f}"
    )

    print(
        f"roll:  mean={roll['mean']:+.7f} deg  "
        f"range={roll['range']:.7f}  "
        f"std={roll['std']:.7f}"
    )

    print(
        f"h-AGL: mean={hagl['mean']:.7f} ft  "
        f"range={hagl['range']:.7f}  "
        f"std={hagl['std']:.7f}"
    )

    print()
    print(
        "gear         mean(in)   min(in)   max(in)   "
        "range(in)   comp-v RMS(ft/s)   WOW changes"
    )

    for name, index in GEARS:
        comp_in = [
            value * 12.0
            for value in data[f"{name}_comp"]
        ]

        velocities = data[f"{name}_vel"]
        wow = data[f"{name}_wow"]

        c = stats(comp_in)

        rms_vel = (
            sum(v * v for v in velocities) / len(velocities)
        ) ** 0.5

        wow_changes = sum(
            1
            for a, b in zip(wow, wow[1:])
            if a != b
        )

        print(
            f"{name:<11} "
            f"{c['mean']:>9.4f} "
            f"{c['min']:>9.4f} "
            f"{c['max']:>9.4f} "
            f"{c['range']:>10.4f} "
            f"{rms_vel:>17.7f} "
            f"{wow_changes:>13d}"
        )


def main():
    print("Mooney M20M ground-contact stability diagnostic")
    print(f"Repository: {REPO}")
    print("Production XML is NOT modified.")
    print(f"Settle period: {SETTLE_SECONDS:.1f} s")
    print(f"Measurement window: {SAMPLE_SECONDS:.1f} s")
    print(f"Rates: {RATES_HZ}")

    for rate in RATES_HZ:
        run_case(rate)


if __name__ == "__main__":
    main()
