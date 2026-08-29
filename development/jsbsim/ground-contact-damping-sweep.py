#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import jsbsim


REPO = Path(__file__).resolve().parents[2]

MODEL = "FDM/Mooney-M20M"
GROUND_XML_REL = Path(
    "FDM/Config/MooneyM20M-ground-reactions.xml"
)

RATE_HZ = 120
DT = 1.0 / RATE_HZ

INITIAL_H_AGL = 4.50
TEST_SECONDS = 12.0

DAMPING_MULTIPLIERS = (
    1.00,
    0.75,
    0.50,
    0.35,
)

GEARS = (
    ("nose", 0),
    ("left-main", 1),
    ("right-main", 2),
)

SETTLE_HDOT_FPS = 0.02
SETTLE_COMP_VEL_FPS = 0.01
SETTLE_COMP_ERR_IN = 0.02
SETTLE_HOLD_SEC = 1.0

OSC_HDOT_DEADBAND = 0.02


def prop(fdm, name):
    return float(fdm.get_property_value(name))


def patch_damping(xml_path: Path, multiplier: float):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    values = {}

    for contact in root.findall("contact"):
        name = contact.get("name")

        if name not in {
            "nose",
            "left-main",
            "right-main",
        }:
            continue

        comp = contact.find("damping_coeff")
        rebound = contact.find("damping_coeff_rebound")

        if comp is None or rebound is None:
            raise RuntimeError(
                f"{name}: missing damping coefficient"
            )

        base_comp = float(comp.text.strip())
        base_rebound = float(rebound.text.strip())

        new_comp = base_comp * multiplier
        new_rebound = base_rebound * multiplier

        comp.text = f"{new_comp:.6f}"
        rebound.text = f"{new_rebound:.6f}"

        values[name] = {
            "compression": new_comp,
            "rebound": new_rebound,
        }

    expected = {
        "nose",
        "left-main",
        "right-main",
    }

    if set(values) != expected:
        raise RuntimeError(
            "Could not find all three gear contacts"
        )

    tree.write(
        xml_path,
        encoding="UTF-8",
        xml_declaration=True,
    )

    return values


def make_temp_aircraft(multiplier: float):
    temp = tempfile.TemporaryDirectory(
        prefix="mooney-ground-damping-"
    )

    temp_root = Path(temp.name)

    shutil.copytree(
        REPO / "FDM",
        temp_root / "FDM",
    )

    xml_path = temp_root / GROUND_XML_REL

    damping = patch_damping(
        xml_path,
        multiplier,
    )

    return temp, temp_root, damping


def make_fdm(temp_root: Path):
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    ok = fdm.load_model_with_paths(
        MODEL,
        str(temp_root),
        str(REPO / "Engines"),
        str(REPO / "Systems"),
        False,
    )

    if not ok:
        raise RuntimeError(
            "Could not load temporary Mooney FDM"
        )

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
        "systems/airframe-controls/gear/handle",
        1.0,
    )

    for name in (
        "fcs/left-brake-cmd-norm",
        "fcs/right-brake-cmd-norm",
        "fcs/center-brake-cmd-norm",
    ):
        fdm.set_property_value(name, 1.0)

    return fdm


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
            "comp_ft": prop(
                fdm,
                f"{base}/compression-ft",
            ),
            "comp_v": prop(
                fdm,
                (
                    f"{base}/"
                    "compression-velocity-fps"
                ),
            ),
        }

    return {
        "t": t,
        "h_agl": prop(
            fdm,
            "position/h-agl-ft",
        ),
        "hdot": prop(
            fdm,
            "velocities/h-dot-fps",
        ),
        "pitch": prop(
            fdm,
            "attitude/theta-deg",
        ),
        "gears": gears,
    }


def final_static(samples):
    count = int(1.0 / DT)
    tail = samples[-count:]

    static = {}

    for name, _ in GEARS:
        static[name] = (
            sum(
                s["gears"][name]["comp_ft"]
                for s in tail
            )
            / len(tail)
            * 12.0
        )

    return static


def find_settle_time(
    samples,
    all_contact_t,
    static_comp,
):
    if all_contact_t is None:
        return None

    hold_steps = int(
        SETTLE_HOLD_SEC / DT
    )

    for i, s in enumerate(samples):
        if s["t"] < all_contact_t:
            continue

        end = i + hold_steps

        if end > len(samples):
            break

        window = samples[i:end]

        good = True

        for w in window:
            if abs(w["hdot"]) >= SETTLE_HDOT_FPS:
                good = False
                break

            for name, _ in GEARS:
                gear = w["gears"][name]

                if not gear["wow"]:
                    good = False
                    break

                if (
                    abs(gear["comp_v"])
                    >= SETTLE_COMP_VEL_FPS
                ):
                    good = False
                    break

                comp_in = (
                    gear["comp_ft"] * 12.0
                )

                if (
                    abs(
                        comp_in
                        - static_comp[name]
                    )
                    >= SETTLE_COMP_ERR_IN
                ):
                    good = False
                    break

            if not good:
                break

        if good:
            return s["t"] - all_contact_t

    return None


def count_vertical_oscillations(
    samples,
    start_t,
    stop_after=None,
):
    if start_t is None:
        return 0

    previous_sign = 0
    sign_changes = 0

    for s in samples:
        if s["t"] < start_t:
            continue

        if (
            stop_after is not None
            and s["t"] > start_t + stop_after
        ):
            break

        v = s["hdot"]

        if v > OSC_HDOT_DEADBAND:
            sign = 1
        elif v < -OSC_HDOT_DEADBAND:
            sign = -1
        else:
            continue

        if (
            previous_sign != 0
            and sign != previous_sign
        ):
            sign_changes += 1

        previous_sign = sign

    # One full bounce cycle is approximately
    # two velocity sign changes.
    return sign_changes / 2.0


def gear_transient(
    samples,
    name,
    first_contact_t,
    static_in,
):
    relevant = [
        s
        for s in samples
        if (
            first_contact_t is not None
            and s["t"] >= first_contact_t
        )
    ]

    if not relevant:
        return {
            "peak": 0.0,
            "overshoot": 0.0,
            "undershoot": 0.0,
            "max_comp_v": 0.0,
        }

    comp = [
        s["gears"][name]["comp_ft"] * 12.0
        for s in relevant
    ]

    comp_v = [
        abs(s["gears"][name]["comp_v"])
        for s in relevant
    ]

    peak_index = max(
        range(len(comp)),
        key=lambda i: comp[i],
    )

    peak = comp[peak_index]

    after_peak = []

    for s in relevant[peak_index:]:
        gear = s["gears"][name]

        if gear["wow"]:
            after_peak.append(
                gear["comp_ft"] * 12.0
            )

    min_after_peak = (
        min(after_peak)
        if after_peak
        else static_in
    )

    return {
        "peak": peak,
        "overshoot": max(
            0.0,
            peak - static_in,
        ),
        "undershoot": max(
            0.0,
            static_in - min_after_peak,
        ),
        "max_comp_v": max(comp_v),
    }


def run_case(multiplier):
    temp, temp_root, damping = (
        make_temp_aircraft(multiplier)
    )

    try:
        fdm = make_fdm(temp_root)

        samples = []

        first_contact = None
        all_contact = None

        previous_wow = {
            name: 0
            for name, _ in GEARS
        }

        impact_hdot = None

        steps = int(
            TEST_SECONDS / DT
        )

        for step in range(steps):
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

            wow = [
                sample["gears"][name]["wow"]
                for name, _ in GEARS
            ]

            if (
                first_contact is None
                and any(wow)
            ):
                first_contact = t
                impact_hdot = abs(
                    sample["hdot"]
                )

            if (
                all_contact is None
                and all(wow)
            ):
                all_contact = t

            for name, _ in GEARS:
                previous_wow[name] = (
                    sample["gears"][name]["wow"]
                )

        static_comp = final_static(
            samples
        )

        settle = find_settle_time(
            samples,
            all_contact,
            static_comp,
        )

        oscillations = (
            count_vertical_oscillations(
                samples,
                all_contact,
                None
                if settle is None
                else settle
                + SETTLE_HOLD_SEC,
            )
        )

        pitch_values = [
            s["pitch"]
            for s in samples
        ]

        transient = {}

        for name, _ in GEARS:
            transient[name] = gear_transient(
                samples,
                name,
                first_contact,
                static_comp[name],
            )

        final = samples[-1]

        return {
            "multiplier": multiplier,
            "damping": damping,
            "first_contact": first_contact,
            "all_contact": all_contact,
            "impact_hdot": impact_hdot,
            "settle": settle,
            "oscillations": oscillations,
            "pitch_range": (
                max(pitch_values)
                - min(pitch_values)
            ),
            "static": static_comp,
            "transient": transient,
            "final_h": final["h_agl"],
            "final_pitch": final["pitch"],
        }

    finally:
        temp.cleanup()


def fmt(value, width=7, precision=3):
    if value is None:
        return " " * (width - 3) + "---"

    return (
        f"{value:{width}.{precision}f}"
    )


def main():
    print(
        "MOONEY M20M DAMPING SENSITIVITY SWEEP"
    )
    print("=" * 72)

    print(
        f"JSBSim rate: {RATE_HZ} Hz"
    )

    print(
        f"Drop initial h-AGL: "
        f"{INITIAL_H_AGL:.2f} ft"
    )

    print(
        "Production aircraft XML is NEVER modified."
    )

    print()

    results = []

    for multiplier in DAMPING_MULTIPLIERS:
        result = run_case(
            multiplier
        )

        results.append(result)

    print(
        " mult  impact  settle  cycles  "
        "pitch_rng  nose_over  main_over"
    )

    print(
        " ----  ------  ------  ------  "
        "---------  ---------  ---------"
    )

    for r in results:
        main_over = (
            r["transient"]["left-main"][
                "overshoot"
            ]
            + r["transient"]["right-main"][
                "overshoot"
            ]
        ) / 2.0

        print(
            f" {r['multiplier']:>4.2f}  "
            f"{fmt(r['impact_hdot'], 6, 3)}  "
            f"{fmt(r['settle'], 6, 3)}  "
            f"{r['oscillations']:>6.1f}  "
            f"{r['pitch_range']:>9.4f}  "
            f"{r['transient']['nose']['overshoot']:>9.4f}  "
            f"{main_over:>9.4f}"
        )

    for r in results:
        print()
        print("=" * 72)

        print(
            f"DAMPING MULTIPLIER "
            f"{r['multiplier']:.2f}x"
        )

        print("=" * 72)

        nose_d = r["damping"]["nose"]

        main_d = r["damping"][
            "left-main"
        ]

        print(
            "nose damping: "
            f"{nose_d['compression']:.1f} / "
            f"{nose_d['rebound']:.1f} "
            "lb/ft/s"
        )

        print(
            "main damping: "
            f"{main_d['compression']:.1f} / "
            f"{main_d['rebound']:.1f} "
            "lb/ft/s"
        )

        print(
            f"impact |h-dot|: "
            f"{r['impact_hdot']:.4f} ft/s"
        )

        print(
            "settle after all-WOW: "
            + (
                f"{r['settle']:.4f} s"
                if r["settle"] is not None
                else "NOT SETTLED"
            )
        )

        print(
            f"vertical cycles: "
            f"{r['oscillations']:.1f}"
        )

        print(
            f"pitch range: "
            f"{r['pitch_range']:.5f} deg"
        )

        print(
            f"final h-AGL: "
            f"{r['final_h']:.6f} ft"
        )

        print(
            f"final pitch: "
            f"{r['final_pitch']:+.6f} deg"
        )

        print()

        print(
            "gear         static   peak    "
            "over    rebound-under  "
            "max|comp-v|"
        )

        for name, _ in GEARS:
            tr = r["transient"][name]

            print(
                f"{name:<11} "
                f"{r['static'][name]:>7.4f} "
                f"{tr['peak']:>7.4f} "
                f"{tr['overshoot']:>7.4f} "
                f"{tr['undershoot']:>13.4f} "
                f"{tr['max_comp_v']:>11.4f}"
            )


if __name__ == "__main__":
    main()
