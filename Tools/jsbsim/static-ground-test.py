#!/usr/bin/env python3

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

GROUND_XML = REPO / "FDM/Config/MooneyM20M-ground-reactions.xml"

DT = 1.0 / 120.0
SETTLE_TIME = 15.0


def load_ground_config():
    root = ET.parse(GROUND_XML).getroot()

    contacts = {}

    for contact in root.findall("contact"):
        name = contact.attrib["name"]

        location = contact.find("location")
        spring = contact.find("spring_coeff")

        contacts[name] = {
            "x_in": float(location.findtext("x")),
            "y_in": float(location.findtext("y")),
            "z_in": float(location.findtext("z")),
            "spring_lb_ft": float(spring.text),
        }

    required = {"nose", "left-main", "right-main"}

    if set(contacts) != required:
        raise RuntimeError(
            f"Expected contacts {sorted(required)}, "
            f"found {sorted(contacts)}"
        )

    return contacts


def gear_snapshot(fdm):
    gr = fdm.get_ground_reactions()
    count = gr.get_num_gear_units()

    if count != len(GEAR_NAMES):
        raise RuntimeError(
            f"JSBSim reports {count} gear units, "
            f"expected {len(GEAR_NAMES)}"
        )

    rows = []

    for i, name in enumerate(GEAR_NAMES):
        base = f"gear/unit[{i}]"

        rows.append({
            "name": name,
            "wow": bool(
                fdm.get_property_value(f"{base}/WOW")
            ),
            "compression_in": (
                fdm.get_property_value(f"{base}/compression-ft") * 12.0
            ),
            "compression_velocity_fps": (
                fdm.get_property_value(
                    f"{base}/compression-velocity-fps"
                )
            ),
        })

    return rows


def print_snapshot(fdm, title):
    print()
    print(title)
    print("-" * len(title))

    print(
        f"t={fdm.get_sim_time():.3f} s  "
        f"AGL={fdm.get_property_value('position/h-agl-ft'):.4f} ft  "
        f"roll={fdm.get_property_value('attitude/phi-deg'):.3f} deg  "
        f"pitch={fdm.get_property_value('attitude/theta-deg'):.3f} deg"
    )

    for row in gear_snapshot(fdm):
        print(
            f"{row['name']:10s} "
            f"WOW={int(row['wow'])}  "
            f"compression={row['compression_in']:.4f} in  "
            f"velocity={row['compression_velocity_fps']:+.6f} ft/s"
        )


contacts = load_ground_config()
GEAR_NAMES = tuple(contacts.keys())

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
    raise SystemExit("FAIL: could not load Mooney FDM")


# Current gear geometry has all three contact patches 51 inches below CG.
# Start just above uncompressed ground contact and let gravity settle it.
contact_height_ft = -contacts["nose"]["z_in"] / 12.0

fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
fdm.set_property_value("ic/h-agl-ft", contact_height_ft + 0.05)

fdm.set_property_value("ic/lat-gc-deg", 0.0)
fdm.set_property_value("ic/long-gc-deg", 0.0)

fdm.set_property_value("ic/phi-deg", 0.0)
fdm.set_property_value("ic/theta-deg", 0.0)
fdm.set_property_value("ic/psi-true-deg", 0.0)

fdm.set_property_value("ic/vg-kts", 0.0)
fdm.set_property_value("ic/roc-fpm", 0.0)

if not fdm.run_ic():
    raise SystemExit("FAIL: run_ic() failed")


weight = fdm.get_property_value("inertia/weight-lbs")
cg_x = fdm.get_property_value("inertia/cg-x-in")

nose_x = contacts["nose"]["x_in"]
main_x = contacts["left-main"]["x_in"]

wheelbase = main_x - nose_x

nose_load = weight * (main_x - cg_x) / wheelbase
main_load = (weight - nose_load) / 2.0

expected = {
    "nose": (
        nose_load,
        nose_load / contacts["nose"]["spring_lb_ft"] * 12.0
    ),
    "left-main": (
        main_load,
        main_load / contacts["left-main"]["spring_lb_ft"] * 12.0
    ),
    "right-main": (
        main_load,
        main_load / contacts["right-main"]["spring_lb_ft"] * 12.0
    ),
}

print("MOONEY STATIC GROUND TEST")
print("=========================")
print(f"JSBSim version: {jsbsim.__version__}")
print(f"dt:             {DT:.8f} s")
print(f"settle time:    {SETTLE_TIME:.1f} s")
print(f"weight:         {weight:.2f} lb")
print(f"CG X:           {cg_x:.4f} in")
print()

print("Analytical static benchmark:")
for name, (load, compression) in expected.items():
    print(
        f"{name:10s} "
        f"load≈{load:.2f} lb  "
        f"compression≈{compression:.4f} in"
    )

print_snapshot(fdm, "Initial state")

next_report = 1.0

while fdm.get_sim_time() < SETTLE_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: JSBSim stopped unexpectedly")

    agl = fdm.get_property_value("position/h-agl-ft")
    roll = fdm.get_property_value("attitude/phi-deg")
    pitch = fdm.get_property_value("attitude/theta-deg")

    if not all(math.isfinite(v) for v in (agl, roll, pitch)):
        raise SystemExit("FAIL: non-finite aircraft state")

    if fdm.get_sim_time() + DT / 2 >= next_report:
        print_snapshot(fdm, f"State near {next_report:.0f} s")
        next_report += 2.0


print_snapshot(fdm, "Final settled state")

print()
print("Comparison")
print("----------")

final = {row["name"]: row for row in gear_snapshot(fdm)}

for name in ("nose", "left-main", "right-main"):
    measured = final[name]["compression_in"]
    target = expected[name][1]

    print(
        f"{name:10s} "
        f"measured={measured:.4f} in  "
        f"benchmark={target:.4f} in  "
        f"delta={measured - target:+.4f} in"
    )

print()
print("STATIC GROUND TEST COMPLETE")
