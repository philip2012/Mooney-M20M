#!/usr/bin/env python3

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"
GROUND_XML = REPO / "FDM/Config/MooneyM20M-ground-reactions.xml"

DT = 1.0 / 120.0
STATIC_SETTLE_TIME = 5.0

START_SPEED_KTS = 10.0
PRE_BRAKE_TIME = 1.0
TEST_TIME = 3.0

LEFT_BRAKE = 0.5
RIGHT_BRAKE = 0.0

FPS_PER_KT = 1.6878098571


def load_contact_names():
    root = ET.parse(GROUND_XML).getroot()
    names = [c.attrib["name"] for c in root.findall("contact")]

    expected = ["nose", "left-main", "right-main"]

    if names != expected:
        raise RuntimeError(
            f"Expected contact order {expected}, found {names}"
        )

    return names


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
        raise RuntimeError("Could not load Mooney FDM")

    return fdm


def configure_neutral(fdm):
    # Engine off.
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/throttle-norm",
        0.0,
    )
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/mixture-norm",
        0.0,
    )

    # No brakes initially.
    fdm.set_property_value("fcs/left-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/right-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/center-brake-cmd-norm", 0.0)

    # No steering or rudder input.
    fdm.set_property_value("fcs/steer-cmd-norm", 0.0)
    fdm.set_property_value("fcs/rudder-cmd-norm", 0.0)


def set_ic(fdm, agl_ft, pitch_deg, speed_kts):
    fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
    fdm.set_property_value("ic/h-agl-ft", agl_ft)

    fdm.set_property_value("ic/lat-gc-deg", 0.0)
    fdm.set_property_value("ic/long-gc-deg", 0.0)

    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", pitch_deg)
    fdm.set_property_value("ic/psi-true-deg", 0.0)

    fdm.set_property_value("ic/vg-kts", speed_kts)
    fdm.set_property_value("ic/roc-fpm", 0.0)


def speed_kts(fdm):
    return (
        fdm.get_property_value("velocities/vg-fps")
        / FPS_PER_KT
    )


def heading_delta_deg(current, initial):
    return (current - initial + 180.0) % 360.0 - 180.0


def wow(fdm):
    return [
        int(bool(fdm.get_property_value(f"gear/unit[{i}]/WOW")))
        for i in range(3)
    ]


def gear_forces(fdm, names):
    gr = fdm.get_ground_reactions()

    rows = []

    for i, name in enumerate(names):
        gear = gr.get_gear_unit(i)

        rows.append({
            "name": name,
            "fx": gear.get_body_x_force(),
            "fy": gear.get_body_y_force(),
            "fz": gear.get_body_z_force(),
        })

    return rows


names = load_contact_names()

# ------------------------------------------------------------
# Find natural static attitude.
# ------------------------------------------------------------

settle = make_fdm()
configure_neutral(settle)

set_ic(
    settle,
    agl_ft=4.30,
    pitch_deg=0.0,
    speed_kts=0.0,
)

if not settle.run_ic():
    raise SystemExit("FAIL: settle run_ic() failed")

while settle.get_sim_time() < STATIC_SETTLE_TIME:
    if not settle.run():
        raise SystemExit("FAIL: static settle stopped")

settled_agl = settle.get_property_value("position/h-agl-ft")
settled_pitch = settle.get_property_value("attitude/theta-deg")


# ------------------------------------------------------------
# Rolling differential-brake case.
# ------------------------------------------------------------

fdm = make_fdm()
configure_neutral(fdm)

set_ic(
    fdm,
    agl_ft=settled_agl,
    pitch_deg=settled_pitch,
    speed_kts=START_SPEED_KTS,
)

if not fdm.run_ic():
    raise SystemExit("FAIL: rolling run_ic() failed")


print("MOONEY DIFFERENTIAL BRAKING TEST")
print("================================")
print(f"JSBSim version:  {jsbsim.__version__}")
print(f"start speed:     {START_SPEED_KTS:.2f} kt")
print(f"left brake:      {LEFT_BRAKE:.2f}")
print(f"right brake:     {RIGHT_BRAKE:.2f}")
print(f"test duration:   {TEST_TIME:.2f} s")
print(f"settled AGL:     {settled_agl:.4f} ft")
print(f"settled pitch:   {settled_pitch:.4f} deg")


while fdm.get_sim_time() < PRE_BRAKE_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: pre-brake roll stopped")


start_time = fdm.get_sim_time()
start_speed = speed_kts(fdm)
start_heading = fdm.get_property_value("attitude/psi-deg")

fdm.set_property_value(
    "fcs/left-brake-cmd-norm",
    LEFT_BRAKE,
)
fdm.set_property_value(
    "fcs/right-brake-cmd-norm",
    RIGHT_BRAKE,
)


print()
print("DIFFERENTIAL BRAKE APPLIED")
print("--------------------------")
print(f"speed:       {start_speed:.4f} kt")
print(f"heading:     {start_heading:.4f} deg")
print(f"WOW:         {wow(fdm)}")


next_report = 0.0
max_abs_roll = 0.0
max_abs_yaw_rate = 0.0

while fdm.get_sim_time() - start_time < TEST_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: JSBSim stopped unexpectedly")

    elapsed = fdm.get_sim_time() - start_time

    speed = speed_kts(fdm)
    roll = fdm.get_property_value("attitude/phi-deg")
    pitch = fdm.get_property_value("attitude/theta-deg")
    heading = fdm.get_property_value("attitude/psi-deg")
    yaw_rate = fdm.get_property_value("velocities/r-rad_sec")

    values = (
        speed,
        roll,
        pitch,
        heading,
        yaw_rate,
    )

    if not all(math.isfinite(v) for v in values):
        raise SystemExit("FAIL: non-finite state")

    max_abs_roll = max(max_abs_roll, abs(roll))
    max_abs_yaw_rate = max(
        max_abs_yaw_rate,
        abs(yaw_rate),
    )

    if elapsed + DT / 2.0 >= next_report:
        rows = gear_forces(fdm, names)

        print()
        print(
            f"t={elapsed:4.2f} s  "
            f"speed={speed:6.3f} kt  "
            f"WOW={wow(fdm)}  "
            f"roll={roll:+.4f} deg  "
            f"pitch={pitch:+.4f} deg  "
            f"heading={heading:+.4f} deg  "
            f"yaw-rate={yaw_rate:+.5f} rad/s"
        )

        for row in rows:
            print(
                f"  {row['name']:10s} "
                f"Fx={row['fx']:+10.3f} lb  "
                f"Fy={row['fy']:+10.3f} lb  "
                f"Fz={row['fz']:+10.3f} lb"
            )

        next_report += 0.5


final_speed = speed_kts(fdm)
final_heading = fdm.get_property_value("attitude/psi-deg")

print()
print("RESULT")
print("------")
print(f"initial speed:       {start_speed:.4f} kt")
print(f"final speed:         {final_speed:.4f} kt")
print(
    f"heading change:      "
    f"{heading_delta_deg(final_heading, start_heading):+.4f} deg"
)
print(f"max abs roll:        {max_abs_roll:.4f} deg")
print(
    f"max abs yaw rate:    "
    f"{max_abs_yaw_rate:.6f} rad/s"
)
print(f"final WOW:           {wow(fdm)}")

print()
print("DIFFERENTIAL BRAKING TEST COMPLETE")
