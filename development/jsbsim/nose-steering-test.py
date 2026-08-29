#!/usr/bin/env python3

from pathlib import Path
import math

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
SETTLE_TIME = 5.0

START_SPEED_KTS = 8.0
PRE_STEER_TIME = 1.0
TEST_TIME = 3.0

STEER_COMMAND = 0.5
EXPECTED_STEER_DEG = 6.5

FPS_PER_KT = 1.6878098571


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


def neutral_controls(fdm):
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/throttle-norm", 0.0
    )
    fdm.set_property_value(
        "systems/powerplant-controls/engine/handles/mixture-norm", 0.0
    )

    fdm.set_property_value("fcs/left-brake-cmd-norm", 0.0)
    fdm.set_property_value("fcs/right-brake-cmd-norm", 0.0)

    fdm.set_property_value("fcs/rudder-cmd-norm", 0.0)
    fdm.set_property_value("fcs/steer-cmd-norm", 0.0)


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
    return fdm.get_property_value("velocities/vg-fps") / FPS_PER_KT


def wow(fdm):
    return [
        int(bool(fdm.get_property_value(f"gear/unit[{i}]/WOW")))
        for i in range(3)
    ]


def heading_delta(current, initial):
    return (current - initial + 180.0) % 360.0 - 180.0


# Establish static attitude.
settle = make_fdm()
neutral_controls(settle)

set_ic(
    settle,
    agl_ft=4.30,
    pitch_deg=0.0,
    speed_kts=0.0,
)

if not settle.run_ic():
    raise SystemExit("FAIL: settle run_ic() failed")

while settle.get_sim_time() < SETTLE_TIME:
    if not settle.run():
        raise SystemExit("FAIL: settle stopped")

settled_agl = settle.get_property_value("position/h-agl-ft")
settled_pitch = settle.get_property_value("attitude/theta-deg")


# Fresh rolling test.
fdm = make_fdm()
neutral_controls(fdm)

set_ic(
    fdm,
    agl_ft=settled_agl,
    pitch_deg=settled_pitch,
    speed_kts=START_SPEED_KTS,
)

if not fdm.run_ic():
    raise SystemExit("FAIL: rolling run_ic() failed")


print("MOONEY NOSE STEERING TEST")
print("=========================")
print(f"JSBSim version:      {jsbsim.__version__}")
print(f"start speed:         {START_SPEED_KTS:.2f} kt")
print(f"steer command:       {STEER_COMMAND:+.2f}")
print(f"expected steer angle:{EXPECTED_STEER_DEG:+.2f} deg")
print(f"test duration:       {TEST_TIME:.2f} s")


while fdm.get_sim_time() < PRE_STEER_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: pre-steer roll stopped")


start_time = fdm.get_sim_time()
start_heading = fdm.get_property_value("attitude/psi-deg")

fdm.set_property_value("fcs/steer-cmd-norm", STEER_COMMAND)


next_report = 0.0
max_abs_roll = 0.0

while fdm.get_sim_time() - start_time < TEST_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: JSBSim stopped")

    elapsed = fdm.get_sim_time() - start_time

    speed = speed_kts(fdm)
    roll = fdm.get_property_value("attitude/phi-deg")
    pitch = fdm.get_property_value("attitude/theta-deg")
    heading = fdm.get_property_value("attitude/psi-deg")
    yaw_rate = fdm.get_property_value("velocities/r-rad_sec")
    steer_deg = fdm.get_property_value("fcs/steer-pos-deg")

    values = (
        speed,
        roll,
        pitch,
        heading,
        yaw_rate,
        steer_deg,
    )

    if not all(math.isfinite(v) for v in values):
        raise SystemExit("FAIL: non-finite state")

    max_abs_roll = max(max_abs_roll, abs(roll))

    if elapsed + DT / 2.0 >= next_report:
        print()
        print(
            f"t={elapsed:4.2f} s  "
            f"speed={speed:6.3f} kt  "
            f"steer={steer_deg:+7.3f} deg  "
            f"WOW={wow(fdm)}  "
            f"roll={roll:+.4f} deg  "
            f"pitch={pitch:+.4f} deg  "
            f"heading={heading:+.4f} deg  "
            f"yaw-rate={yaw_rate:+.5f} rad/s"
        )

        next_report += 0.5


final_heading = fdm.get_property_value("attitude/psi-deg")
final_steer = fdm.get_property_value("fcs/steer-pos-deg")

print()
print("RESULT")
print("------")
print(f"final steer angle:  {final_steer:+.4f} deg")
print(
    f"steer error:        "
    f"{final_steer - EXPECTED_STEER_DEG:+.4f} deg"
)
print(
    f"heading change:     "
    f"{heading_delta(final_heading, start_heading):+.4f} deg"
)
print(f"max abs roll:       {max_abs_roll:.4f} deg")
print(f"final WOW:          {wow(fdm)}")

print()
print("NOSE STEERING TEST COMPLETE")
