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
ROLL_TIME = 12.0
START_SPEED_KTS = 15.0

FPS_PER_KT = 1.6878098571
G_FPS2 = 32.174


def load_contacts():
    root = ET.parse(GROUND_XML).getroot()
    contacts = []

    for c in root.findall("contact"):
        contacts.append({
            "name": c.attrib["name"],
            "rolling_friction": float(c.findtext("rolling_friction")),
        })

    if len(contacts) != 3:
        raise RuntimeError(
            f"Expected 3 ground contacts, found {len(contacts)}"
        )

    return contacts


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


def configure_controls(fdm):
    # Engine off.
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/throttle-norm", 0.0
    )
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/mixture-norm", 0.0
    )

    # No braking.
    fdm.set_property_value("fcs/left-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/right-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/center-brake-cmd-norm", 0.0)

    # Straight ahead.
    fdm.set_property_value("fcs/steer-cmd-norm", 0.0)
    fdm.set_property_value("fcs/rudder-cmd-norm", 0.0)


def set_common_ic(fdm, agl_ft, pitch_deg, speed_kts):
    fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
    fdm.set_property_value("ic/h-agl-ft", agl_ft)

    fdm.set_property_value("ic/lat-gc-deg", 0.0)
    fdm.set_property_value("ic/long-gc-deg", 0.0)

    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", pitch_deg)
    fdm.set_property_value("ic/psi-true-deg", 0.0)

    fdm.set_property_value("ic/vg-kts", speed_kts)
    fdm.set_property_value("ic/roc-fpm", 0.0)


def wow_values(fdm):
    return [
        int(bool(fdm.get_property_value(f"gear/unit[{i}]/WOW")))
        for i in range(3)
    ]


def speed_kts(fdm):
    return (
        fdm.get_property_value("velocities/vg-fps")
        / FPS_PER_KT
    )


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


contacts = load_contacts()
names = [c["name"] for c in contacts]

# ----------------------------------------------------------------------
# Phase 1: establish the aircraft's natural static attitude.
# ----------------------------------------------------------------------

settle = make_fdm()
configure_controls(settle)

# Same starting geometry as the static-ground harness.
set_common_ic(
    settle,
    agl_ft=4.30,
    pitch_deg=0.0,
    speed_kts=0.0,
)

if not settle.run_ic():
    raise SystemExit("FAIL: static run_ic() failed")

while settle.get_sim_time() < STATIC_SETTLE_TIME:
    if not settle.run():
        raise SystemExit("FAIL: static settle stopped")

settled_agl = settle.get_property_value("position/h-agl-ft")
settled_pitch = settle.get_property_value("attitude/theta-deg")

print("MOONEY GROUND ROLL TEST")
print("=======================")
print(f"JSBSim version:       {jsbsim.__version__}")
print(f"dt:                   {DT:.8f} s")
print(f"start speed:          {START_SPEED_KTS:.2f} kt")
print(f"roll duration:        {ROLL_TIME:.1f} s")
print(f"settled AGL:          {settled_agl:.4f} ft")
print(f"settled pitch:        {settled_pitch:.4f} deg")

weight = settle.get_property_value("inertia/weight-lbs")

mu_values = [c["rolling_friction"] for c in contacts]

if max(mu_values) - min(mu_values) > 1e-12:
    print("Rolling coefficients differ; simple benchmark disabled.")
    expected_force = None
    expected_decel = None
else:
    mu = mu_values[0]
    expected_force = weight * mu
    expected_decel = mu * G_FPS2 / FPS_PER_KT

    print(f"weight:               {weight:.2f} lb")
    print(f"rolling friction mu:  {mu:.5f}")
    print(f"rough force benchmark:{expected_force:.2f} lb")
    print(f"rough decel benchmark:{expected_decel:.4f} kt/s")


# ----------------------------------------------------------------------
# Phase 2: start a fresh FDM at that static attitude, rolling at 15 kt.
# ----------------------------------------------------------------------

fdm = make_fdm()
configure_controls(fdm)

set_common_ic(
    fdm,
    agl_ft=settled_agl,
    pitch_deg=settled_pitch,
    speed_kts=START_SPEED_KTS,
)

if not fdm.run_ic():
    raise SystemExit("FAIL: rolling run_ic() failed")

initial_heading = fdm.get_property_value("attitude/psi-deg")
initial_speed = speed_kts(fdm)

samples = []

next_report = 0.0

while fdm.get_sim_time() < ROLL_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: JSBSim stopped unexpectedly")

    t = fdm.get_sim_time()
    v = speed_kts(fdm)
    roll = fdm.get_property_value("attitude/phi-deg")
    pitch = fdm.get_property_value("attitude/theta-deg")
    heading = fdm.get_property_value("attitude/psi-deg")

    values = (t, v, roll, pitch, heading)

    if not all(math.isfinite(x) for x in values):
        raise SystemExit("FAIL: non-finite state")

    samples.append((t, v))

    if t + DT / 2.0 >= next_report:
        forces = gear_forces(fdm, names)
        total_fx = sum(r["fx"] for r in forces)

        print()
        print(
            f"t={t:5.2f} s  "
            f"speed={v:7.3f} kt  "
            f"WOW={wow_values(fdm)}  "
            f"roll={roll:+.4f} deg  "
            f"pitch={pitch:+.4f} deg  "
            f"heading={heading:+.4f} deg"
        )

        for row in forces:
            print(
                f"  {row['name']:10s} "
                f"Fx={row['fx']:+9.3f} lb  "
                f"Fy={row['fy']:+9.3f} lb  "
                f"Fz={row['fz']:+9.3f} lb"
            )

        print(f"  total gear Fx={total_fx:+9.3f} lb")

        next_report += 2.0


final_speed = speed_kts(fdm)
final_heading = fdm.get_property_value("attitude/psi-deg")

elapsed = fdm.get_sim_time()
average_decel = (initial_speed - final_speed) / elapsed

print()
print("RESULT")
print("------")
print(f"initial speed:       {initial_speed:.4f} kt")
print(f"final speed:         {final_speed:.4f} kt")
print(f"elapsed:             {elapsed:.4f} s")
print(f"average deceleration:{average_decel:.4f} kt/s")
print(
    f"heading drift:       "
    f"{final_heading - initial_heading:+.4f} deg"
)
print(f"final WOW:           {wow_values(fdm)}")

if expected_decel is not None:
    print(
        f"rough rolling-only benchmark: "
        f"{expected_decel:.4f} kt/s"
    )
    print(
        f"difference from benchmark:    "
        f"{average_decel - expected_decel:+.4f} kt/s"
    )

print()
print("GROUND ROLL TEST COMPLETE")
