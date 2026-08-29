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

START_SPEED_KTS = 15.0
PRE_BRAKE_TIME = 1.0
MAX_BRAKE_TIME = 8.0
STOP_THRESHOLD_KTS = 0.5

BRAKE_COMMAND = 1.0

FPS_PER_KT = 1.6878098571


def load_contact_names():
    root = ET.parse(GROUND_XML).getroot()
    names = [c.attrib["name"] for c in root.findall("contact")]

    if names != ["nose", "left-main", "right-main"]:
        raise RuntimeError(
            f"Unexpected contact order: {names}"
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
        "systems/powerplant-controls/engine/handles/throttle-norm", 0.0
    )
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/mixture-norm", 0.0
    )

    # No brakes initially.
    fdm.set_property_value("fcs/left-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/right-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/center-brake-cmd-norm", 0.0)

    # Straight.
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


def wow(fdm):
    return [
        int(bool(fdm.get_property_value(f"gear/unit[{i}]/WOW")))
        for i in range(3)
    ]


def forces(fdm, names):
    gr = fdm.get_ground_reactions()

    rows = []

    for i, name in enumerate(names):
        unit = gr.get_gear_unit(i)

        rows.append({
            "name": name,
            "fx": unit.get_body_x_force(),
            "fy": unit.get_body_y_force(),
            "fz": unit.get_body_z_force(),
        })

    return rows


names = load_contact_names()

# ----------------------------------------------------------------------
# Establish natural static attitude.
# ----------------------------------------------------------------------

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


# ----------------------------------------------------------------------
# Fresh rolling case.
# ----------------------------------------------------------------------

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


print("MOONEY SYMMETRIC BRAKING TEST")
print("=============================")
print(f"JSBSim version:       {jsbsim.__version__}")
print(f"dt:                   {DT:.8f} s")
print(f"initial speed:        {START_SPEED_KTS:.2f} kt")
print(f"pre-brake roll:       {PRE_BRAKE_TIME:.2f} s")
print(f"brake command:        {BRAKE_COMMAND:.2f}")
print(f"stop threshold:       {STOP_THRESHOLD_KTS:.2f} kt")
print(f"settled AGL:          {settled_agl:.4f} ft")
print(f"settled pitch:        {settled_pitch:.4f} deg")


# Let the fresh rolling state stabilize before brake application.

while fdm.get_sim_time() < PRE_BRAKE_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: pre-brake roll stopped")

brake_start_time = fdm.get_sim_time()
brake_start_speed = speed_kts(fdm)
brake_start_heading = fdm.get_property_value("attitude/psi-deg")

fdm.set_property_value(
    "fcs/left-brake-cmd-norm",
    BRAKE_COMMAND,
)
fdm.set_property_value(
    "fcs/right-brake-cmd-norm",
    BRAKE_COMMAND,
)

print()
print("BRAKES APPLIED")
print("--------------")
print(f"time:                 {brake_start_time:.4f} s")
print(f"speed:                {brake_start_speed:.4f} kt")
print(f"WOW:                  {wow(fdm)}")


distance_ft = 0.0
next_report = 0.0

max_main_fx_difference = 0.0
stopped = False

while fdm.get_sim_time() - brake_start_time < MAX_BRAKE_TIME:
    previous_speed_fps = fdm.get_property_value("velocities/vg-fps")

    if not fdm.run():
        raise SystemExit("FAIL: JSBSim stopped unexpectedly")

    current_speed = speed_kts(fdm)
    current_speed_fps = fdm.get_property_value("velocities/vg-fps")

    distance_ft += (
        0.5 * (previous_speed_fps + current_speed_fps) * DT
    )

    roll = fdm.get_property_value("attitude/phi-deg")
    pitch = fdm.get_property_value("attitude/theta-deg")
    heading = fdm.get_property_value("attitude/psi-deg")

    if not all(
        math.isfinite(v)
        for v in (
            current_speed,
            roll,
            pitch,
            heading,
            distance_ft,
        )
    ):
        raise SystemExit("FAIL: non-finite state")

    rows = forces(fdm, names)

    main_fx_difference = abs(
        rows[1]["fx"] - rows[2]["fx"]
    )

    max_main_fx_difference = max(
        max_main_fx_difference,
        main_fx_difference,
    )

    elapsed = fdm.get_sim_time() - brake_start_time

    if elapsed + DT / 2.0 >= next_report:
        print()
        print(
            f"brake t={elapsed:5.2f} s  "
            f"speed={current_speed:7.3f} kt  "
            f"WOW={wow(fdm)}  "
            f"roll={roll:+.4f} deg  "
            f"pitch={pitch:+.4f} deg  "
            f"heading={heading:+.4f} deg"
        )

        for row in rows:
            print(
                f"  {row['name']:10s} "
                f"Fx={row['fx']:+10.3f} lb  "
                f"Fy={row['fy']:+10.3f} lb  "
                f"Fz={row['fz']:+10.3f} lb"
            )

        next_report += 0.5

    if current_speed <= STOP_THRESHOLD_KTS:
        stopped = True
        break


brake_elapsed = fdm.get_sim_time() - brake_start_time
final_speed = speed_kts(fdm)
final_heading = fdm.get_property_value("attitude/psi-deg")

average_decel = (
    brake_start_speed - final_speed
) / brake_elapsed


print()
print("RESULT")
print("------")
print(f"stopped threshold reached: {stopped}")
print(f"brake start speed:         {brake_start_speed:.4f} kt")
print(f"final speed:               {final_speed:.4f} kt")
print(f"braking elapsed:           {brake_elapsed:.4f} s")
print(f"approx braking distance:   {distance_ft:.2f} ft")
print(f"average deceleration:      {average_decel:.4f} kt/s")
print(
    f"heading drift:             "
    f"{final_heading - brake_start_heading:+.4f} deg"
)
print(
    f"max main Fx asymmetry:     "
    f"{max_main_fx_difference:.6f} lb"
)
print(f"final WOW:                 {wow(fdm)}")

print()
print("SYMMETRIC BRAKING TEST COMPLETE")
