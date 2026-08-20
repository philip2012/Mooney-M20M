#!/usr/bin/env python3
"""
Mooney M20M ground-contact diagnostic.

Purpose
-------
Compare the untouched production ground-reaction geometry against a temporary
datum-corrected X-station candidate without modifying production XML.

The script:
  * loads the real FDM/Engines/Systems at 120 Hz;
  * keeps the current Z=-51.0 in baseline;
  * settles the aircraft from a small height above level ground;
  * reports WOW, AGL, compression, compression velocity and estimated strut load;
  * compares settled wheel-load fractions against simple static geometry;
  * patches only a temporary copy for the candidate run.

No production aircraft file is modified.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
GROUND_REL = Path("FDM/Config/MooneyM20M-ground-reactions.xml")
MASS_REL = Path("FDM/Config/MooneyM20M-mass-balance.xml")

DT = 1.0 / 120.0
SETTLE_SECONDS = 12.0
INITIAL_H_AGL_FT = 4.35

# Candidate derived from Mooney stationing:
# datum is approximately 13 in aft of the nose-gear trunnion/axle,
# and the known wheelbase is approximately 79.563 in.
CANDIDATE_NOSE_X_IN = -13.000
CANDIDATE_MAIN_X_IN = 66.563

GEAR_NAMES = ("nose", "left-main", "right-main")


@dataclass(frozen=True)
class GearConfig:
    name: str
    index: int
    x_in: float
    y_in: float
    z_in: float
    spring_lb_ft: float
    damp_lb_ft_s: float
    rebound_lb_ft_s: float


@dataclass(frozen=True)
class Variant:
    name: str
    nose_x_in: float | None = None
    main_x_in: float | None = None


VARIANTS = (
    Variant("CURRENT"),
    Variant(
        "DATUM_X_CANDIDATE",
        nose_x_in=CANDIDATE_NOSE_X_IN,
        main_x_in=CANDIDATE_MAIN_X_IN,
    ),
)


def get(fdm: jsbsim.FGFDMExec, prop: str) -> float:
    return float(fdm.get_property_value(prop))


def run_for(fdm: jsbsim.FGFDMExec, seconds: float) -> None:
    end = fdm.get_sim_time() + seconds
    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def read_cg_x_in(root: Path) -> float:
    tree = ET.parse(root / MASS_REL)
    mb = tree.getroot()

    for loc in mb.findall("location"):
        if loc.attrib.get("name") == "CG":
            node = loc.find("x")
            if node is None or node.text is None:
                break
            return float(node.text)

    raise RuntimeError("Could not find mass_balance/location[@name='CG']/x")


def read_gear_config(root: Path) -> list[GearConfig]:
    tree = ET.parse(root / GROUND_REL)
    gr = tree.getroot()

    result: list[GearConfig] = []

    for index, expected_name in enumerate(GEAR_NAMES):
        contact = gr.find(f"contact[@name='{expected_name}']")
        if contact is None:
            raise RuntimeError(f"Missing contact {expected_name!r}")

        loc = contact.find("location")
        if loc is None:
            raise RuntimeError(f"Missing location for {expected_name}")

        def num(parent: ET.Element, tag: str) -> float:
            el = parent.find(tag)
            if el is None or el.text is None:
                raise RuntimeError(f"Missing {tag} for {expected_name}")
            return float(el.text)

        result.append(
            GearConfig(
                name=expected_name,
                index=index,
                x_in=num(loc, "x"),
                y_in=num(loc, "y"),
                z_in=num(loc, "z"),
                spring_lb_ft=num(contact, "spring_coeff"),
                damp_lb_ft_s=num(contact, "damping_coeff"),
                rebound_lb_ft_s=num(contact, "damping_coeff_rebound"),
            )
        )

    return result


def patch_candidate_x(root: Path, variant: Variant) -> None:
    if variant.nose_x_in is None or variant.main_x_in is None:
        return

    path = root / GROUND_REL
    tree = ET.parse(path)
    gr = tree.getroot()

    desired = {
        "nose": variant.nose_x_in,
        "left-main": variant.main_x_in,
        "right-main": variant.main_x_in,
    }

    for name, x_value in desired.items():
        contact = gr.find(f"contact[@name='{name}']")
        if contact is None:
            raise RuntimeError(f"Missing contact {name!r}")

        x = contact.find("location/x")
        if x is None:
            raise RuntimeError(f"Missing x location for {name}")

        x.text = f"{x_value:.3f}"

    tree.write(path, encoding="UTF-8", xml_declaration=True)


def prepare_test_tree(temp_root: Path, variant: Variant) -> Path:
    shutil.copytree(REPO / "FDM", temp_root / "FDM")
    shutil.copytree(REPO / "Engines", temp_root / "Engines")
    shutil.copytree(REPO / "Systems", temp_root / "Systems")

    patch_candidate_x(temp_root, variant)
    return temp_root


def make_fdm(test_root: Path) -> jsbsim.FGFDMExec:
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(test_root),
        str(test_root / "Engines"),
        str(test_root / "Systems"),
        False,
    ):
        raise RuntimeError("Could not load temporary FDM")

    # Level, stationary, just above the existing -51 in contact baseline.
    fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
    fdm.set_property_value("ic/h-agl-ft", INITIAL_H_AGL_FT)
    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)
    fdm.set_property_value("ic/vg-kts", 0.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    # Make the intended test configuration explicit.
    fdm.set_property_value("systems/airframe-controls/gear/handle", 1.0)
    fdm.set_property_value("fcs/left-brake-cmd-norm", 1.0)
    fdm.set_property_value("fcs/right-brake-cmd-norm", 1.0)
    fdm.set_property_value("fcs/center-brake-cmd-norm", 1.0)

    # Engine controls to a benign stopped state.
    for prop, value in (
        ("systems/powerplant-controls/engine/handles/throttle-norm", 0.0),
        ("systems/powerplant-controls/engine/handles/mixture-norm", 0.0),
        ("systems/powerplant-controls/engine/switches/magnetos", 0.0),
    ):
        try:
            fdm.set_property_value(prop, value)
        except Exception:
            pass

    return fdm


def estimated_strut_load_lbs(
    cfg: GearConfig,
    compression_ft: float,
    compression_velocity_fps: float,
) -> float:
    # Mirrors the sign convention of FGLGear::ComputeVerticalStrutForce()
    # for the current linear damping configuration.
    damping = (
        cfg.damp_lb_ft_s
        if compression_velocity_fps >= 0.0
        else cfg.rebound_lb_ft_s
    )

    load = (
        cfg.spring_lb_ft * compression_ft
        + damping * compression_velocity_fps
    )

    return max(0.0, load)


def static_geometry_fractions(
    cg_x_in: float,
    gear: list[GearConfig],
) -> tuple[float, float, float]:
    nose = gear[0]
    left = gear[1]
    right = gear[2]

    main_x = 0.5 * (left.x_in + right.x_in)
    wheelbase = main_x - nose.x_in

    if abs(wheelbase) < 1e-9:
        raise RuntimeError("Zero wheelbase")

    nose_fraction = (main_x - cg_x_in) / wheelbase
    mains_fraction = 1.0 - nose_fraction

    return (
        nose_fraction,
        0.5 * mains_fraction,
        0.5 * mains_fraction,
    )


def report_variant(variant: Variant) -> dict:
    with tempfile.TemporaryDirectory(prefix="m20m-ground-") as tmp:
        root = prepare_test_tree(Path(tmp), variant)
        gear = read_gear_config(root)
        cg_x_in = read_cg_x_in(root)

        fdm = make_fdm(root)

        # Let gear state settle before the drop/settling window.
        run_for(fdm, 0.25)

        max_abs_vertical_speed = 0.0
        max_compressions = [0.0, 0.0, 0.0]

        for _ in range(int(SETTLE_SECONDS / DT)):
            if not fdm.run():
                raise RuntimeError("JSBSim stopped unexpectedly")

            try:
                max_abs_vertical_speed = max(
                    max_abs_vertical_speed,
                    abs(get(fdm, "velocities/h-dot-fps")),
                )
            except Exception:
                pass

            for i in range(3):
                c = get(fdm, f"gear/unit[{i}]/compression-ft")
                max_compressions[i] = max(max_compressions[i], abs(c))

        weight_lbs = get(fdm, "inertia/weight-lbs")
        theta_deg = get(fdm, "attitude/theta-deg")
        phi_deg = get(fdm, "attitude/phi-deg")
        h_agl_ft = get(fdm, "position/h-agl-ft")

        rows = []
        total_est_load = 0.0

        for cfg, max_c in zip(gear, max_compressions):
            base = f"gear/unit[{cfg.index}]"
            compression = get(fdm, f"{base}/compression-ft")
            comp_vel = get(fdm, f"{base}/compression-velocity-fps")
            wow = int(round(get(fdm, f"{base}/WOW")))
            agl = get(fdm, f"{base}/AGL-ft")

            # Read JSBSim's own runtime contact location as a cross-check.
            x_runtime = get(fdm, f"{base}/x-position")
            y_runtime = get(fdm, f"{base}/y-position")
            z_runtime = get(fdm, f"{base}/z-position")

            est_load = estimated_strut_load_lbs(
                cfg,
                compression,
                comp_vel,
            )
            total_est_load += est_load

            rows.append(
                {
                    "name": cfg.name,
                    "wow": wow,
                    "x_in": x_runtime,
                    "y_in": y_runtime,
                    "z_in": z_runtime,
                    "agl_ft": agl,
                    "compression_ft": compression,
                    "compression_in": compression * 12.0,
                    "compression_velocity_fps": comp_vel,
                    "max_compression_in": max_c * 12.0,
                    "estimated_strut_load_lbs": est_load,
                }
            )

        theoretical = static_geometry_fractions(cg_x_in, gear)

        print()
        print("=" * 78)
        print(variant.name)
        print("=" * 78)
        print(f"CG X:                {cg_x_in:9.3f} in")
        print(f"Weight:              {weight_lbs:9.2f} lb")
        print(f"Settled pitch:       {theta_deg:9.4f} deg")
        print(f"Settled roll:        {phi_deg:9.4f} deg")
        print(f"Reference h-AGL:     {h_agl_ft:9.4f} ft")
        print(f"Peak |h-dot| seen:   {max_abs_vertical_speed:9.4f} ft/s")
        print()
        print(
            "gear         WOW      X(in)      Y(in)      Z(in)   "
            "comp(in)   maxcomp(in)   est.load(lb)   load%"
        )

        for row, geom_frac in zip(rows, theoretical):
            load_pct = (
                100.0 * row["estimated_strut_load_lbs"] / total_est_load
                if total_est_load > 1e-9
                else 0.0
            )

            print(
                f"{row['name']:<11} "
                f"{row['wow']:>3d} "
                f"{row['x_in']:>10.3f} "
                f"{row['y_in']:>10.3f} "
                f"{row['z_in']:>10.3f} "
                f"{row['compression_in']:>10.3f} "
                f"{row['max_compression_in']:>13.3f} "
                f"{row['estimated_strut_load_lbs']:>14.1f} "
                f"{load_pct:>7.2f}"
            )

        print()
        print("Simple rigid static geometry prediction:")
        print(f"  nose:       {100.0 * theoretical[0]:7.2f}%")
        print(f"  left-main:  {100.0 * theoretical[1]:7.2f}%")
        print(f"  right-main: {100.0 * theoretical[2]:7.2f}%")
        print()
        print(f"Estimated strut-load sum: {total_est_load:.1f} lb")
        print(f"Aircraft weight:          {weight_lbs:.1f} lb")
        print(
            "Load-sum error:           "
            f"{total_est_load - weight_lbs:+.1f} lb "
            f"({100.0 * (total_est_load / weight_lbs - 1.0):+.2f}%)"
        )

        return {
            "variant": variant.name,
            "cg_x_in": cg_x_in,
            "weight_lbs": weight_lbs,
            "theta_deg": theta_deg,
            "phi_deg": phi_deg,
            "h_agl_ft": h_agl_ft,
            "gear": rows,
            "theoretical_fractions": theoretical,
            "estimated_strut_load_sum_lbs": total_est_load,
        }


def main() -> None:
    print("Mooney M20M ground-contact diagnostic")
    print(f"Repository: {REPO}")
    print("Production XML is NOT modified.")
    print(f"DT: {DT:.9f} s ({1.0 / DT:.0f} Hz)")
    print(f"Settle time: {SETTLE_SECONDS:.1f} s")
    print(f"Initial h-AGL: {INITIAL_H_AGL_FT:.2f} ft")

    results = [report_variant(v) for v in VARIANTS]

    print()
    print("=" * 78)
    print("A/B SUMMARY")
    print("=" * 78)

    for result in results:
        rows = result["gear"]
        total = result["estimated_strut_load_sum_lbs"]

        if total > 1e-9:
            fracs = [
                100.0 * row["estimated_strut_load_lbs"] / total
                for row in rows
            ]
        else:
            fracs = [0.0, 0.0, 0.0]

        print(
            f"{result['variant']:<20} "
            f"pitch={result['theta_deg']:+7.3f} deg  "
            f"nose={fracs[0]:6.2f}%  "
            f"L={fracs[1]:6.2f}%  "
            f"R={fracs[2]:6.2f}%  "
            f"load-sum={total:8.1f} lb"
        )

    print()
    print("Interpretation:")
    print("  * Do not tune Z, springs or damping from this run alone.")
    print("  * First verify that the datum-X candidate gives sensible static loads.")
    print("  * Then fix FlightGear compression normalization separately.")
    print("  * Only after visual/FDM contact agreement should Z and suspension rates")
    print("    be re-qualified.")


if __name__ == "__main__":
    main()
