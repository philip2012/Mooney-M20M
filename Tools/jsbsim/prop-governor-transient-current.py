#!/usr/bin/env python3

from pathlib import Path
import math
import os
import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
KTS_TO_FPS = 1.687809857

ALTITUDE_FT = 20000.0
TEMP_BIAS_F = 0.0
INFLOW_KTS = 175.0

INITIAL_THROTTLE = float(
    os.environ.get(
        "MOONEY_INITIAL_THROTTLE",
        "0.55",
    )
)

FINAL_THROTTLE = float(
    os.environ.get(
        "MOONEY_FINAL_THROTTLE",
        "1.00",
    )
)
THROTTLE_RAMP_SEC = float(
    os.environ.get(
        "MOONEY_THROTTLE_RAMP_SEC",
        "1.0",
    )
)
PRE_RAMP_HOLD_SEC = 3.0
POST_RAMP_SEC = 6.0

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
BLADE = "propulsion/engine[0]/blade-angle"

PILOT_THROTTLE = (
    "systems/powerplant-controls/engine/handles/throttle-norm"
)

THROTTLE_COMMAND = "fcs/throttle-cmd-norm[0]"

THROTTLE_LIMIT = (
    "systems/af1b-density-controller/throttle-limit-norm"
)

PROP = (
    "systems/powerplant-controls/engine/handles/prop-norm"
)

MIXTURE = (
    "systems/powerplant-controls/engine/handles/mixture-norm"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/switches/magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/switches/battery-master"
)

RAM_AIR = "propulsion/engine[0]/ram-air-factor"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def ramp_inflow(fdm):
    for kts in (
        25.0,
        50.0,
        75.0,
        100.0,
        125.0,
        150.0,
        175.0,
    ):
        fdm.set_property_value(
            "atmosphere/wind-north-fps",
            -kts * KTS_TO_FPS,
        )
        run_for(fdm, 0.25)


fdm = jsbsim.FGFDMExec(None)
fdm.set_debug_level(0)
fdm.set_dt(DT)

if not fdm.load_model_with_paths(
    MODEL,
    str(REPO),
    str(REPO / "Engines"),
    str(REPO / "Systems"),
    False,
):
    raise SystemExit("FAIL: could not load permanent Mooney FDM")


fdm.set_property_value(
    "atmosphere/delta-T",
    TEMP_BIAS_F,
)

# Put ASL altitude at exactly 20,000 ft while holding the
# aircraft 4.30 ft above the artificial terrain surface.
fdm.set_property_value(
    "ic/terrain-elevation-ft",
    ALTITUDE_FT - 4.30,
)

fdm.set_property_value(
    "ic/h-agl-ft",
    4.30,
)

fdm.set_property_value("ic/phi-deg", 0.0)
fdm.set_property_value("ic/theta-deg", 0.0)
fdm.set_property_value("ic/psi-true-deg", 0.0)
fdm.set_property_value("ic/vg-kts", 0.0)

if not fdm.run_ic():
    raise SystemExit("FAIL: run_ic() failed")

fdm.set_property_value("forces/hold-down", 1)

fdm.set_property_value(
    "atmosphere/wind-north-fps",
    0.0,
)
fdm.set_property_value(
    "atmosphere/wind-east-fps",
    0.0,
)
fdm.set_property_value(
    "atmosphere/wind-down-fps",
    0.0,
)

# Lycoming zero-ram engine-chart condition.
fdm.set_property_value(RAM_AIR, 0.0)

fdm.set_property_value(BATTERY, 1)
fdm.set_property_value(MIXTURE, 1.0)
fdm.set_property_value(PROP, 1.0)
fdm.set_property_value(PILOT_THROTTLE, INITIAL_THROTTLE)
fdm.set_property_value(MAGNETOS, 3)

run_for(fdm, 0.1)

# Deterministically establish engine 0 as running.
fdm.set_property_value(
    "propulsion/set-running",
    0,
)

run_for(fdm, 1.5)

if get(fdm, RPM) < 1000.0:
    raise SystemExit("FAIL: engine did not establish")

ramp_inflow(fdm)

run_for(
    fdm,
    PRE_RAMP_HOLD_SEC,
)

print(
    f"pre-ramp RPM:        {get(fdm, RPM):.1f}"
)
print(
    f"pre-ramp blade:      {get(fdm, BLADE):.3f} deg"
)


print("MOONEY PERMANENT PROP GOVERNOR TRANSIENT DIAGNOSTIC")
print("===================================================")
print(f"JSBSim version:       {jsbsim.__version__}")
print(f"altitude:             {ALTITUDE_FT:.0f} ft")
print(f"temperature bias:     {TEMP_BIAS_F:+.0f} F")
print(f"inflow:               {INFLOW_KTS:.0f} kt")
print(f"prop command:         {get(fdm, PROP):.2f}")
print(f"initial throttle:     {INITIAL_THROTTLE:.2f}")
print(f"final throttle:       {FINAL_THROTTLE:.2f}")
print(f"throttle ramp:        {THROTTLE_RAMP_SEC:.2f} s")
print()


samples = []

start_time = fdm.get_sim_time()

steps = int(
    (THROTTLE_RAMP_SEC + POST_RAMP_SEC)
    / DT
)

for step in range(steps):
    elapsed_before = (
        fdm.get_sim_time()
        - start_time
    )

    fraction = min(
        1.0,
        elapsed_before
        / THROTTLE_RAMP_SEC,
    )

    pilot = (
        INITIAL_THROTTLE
        + (
            FINAL_THROTTLE
            - INITIAL_THROTTLE
        )
        * fraction
    )

    fdm.set_property_value(
        PILOT_THROTTLE,
        pilot,
    )

    if not fdm.run():
        raise SystemExit(
            "FAIL: simulation stopped during transient"
        )

    elapsed = (
        fdm.get_sim_time()
        - start_time
    )

    sample = {
        "time": elapsed,
        "pilot": get(
            fdm,
            PILOT_THROTTLE,
        ),
        "command": get(
            fdm,
            THROTTLE_COMMAND,
        ),
        "limit": get(
            fdm,
            THROTTLE_LIMIT,
        ),
        "rpm": get(
            fdm,
            RPM,
        ),
        "map": get(
            fdm,
            MAP,
        ),
        "hp": get(
            fdm,
            POWER,
        ),
        "blade": get(
            fdm,
            BLADE,
        ),
    }

    if not all(
        math.isfinite(v)
        for v in sample.values()
    ):
        raise SystemExit(
            "FAIL: non-finite transient state"
        )

    samples.append(sample)


peak = max(
    samples,
    key=lambda s: s["rpm"],
)

max_blade = max(
    samples,
    key=lambda s: s["blade"],
)

settled = samples[-1]

rpm_over = (
    peak["rpm"] - 2575.0
)

print("PEAK RPM")
print("--------")
print(
    f"time:             {peak['time']:.3f} s"
)
print(
    f"RPM:              {peak['rpm']:.1f}"
)
print(
    f"overspeed:        {rpm_over:+.1f} RPM"
)
print(
    f"blade angle:       {peak['blade']:.3f} deg"
)
print(
    f"MAP:               {peak['map']:.2f} inHg"
)
print(
    f"power:             {peak['hp']:.2f} HP"
)
print(
    f"pilot throttle:    {peak['pilot']:.4f}"
)
print(
    f"throttle command:  {peak['command']:.4f}"
)
print(
    f"controller limit:  {peak['limit']:.4f}"
)

print()
print("MAXIMUM BLADE ANGLE DURING TRANSIENT")
print("------------------------------------")
print(
    f"time:             {max_blade['time']:.3f} s"
)
print(
    f"blade angle:       {max_blade['blade']:.3f} deg"
)
print(
    f"RPM:               {max_blade['rpm']:.1f}"
)

print()
print("SETTLED END STATE")
print("-----------------")
print(
    f"RPM:               {settled['rpm']:.1f}"
)
print(
    f"blade angle:       {settled['blade']:.3f} deg"
)
print(
    f"MAP:               {settled['map']:.2f} inHg"
)
print(
    f"power:             {settled['hp']:.2f} HP"
)
print(
    f"throttle command:  {settled['command']:.4f}"
)

print()
print("RESULT")
print("------")

if peak["blade"] >= 44.45:
    print(
        "coarse-pitch saturation at RPM peak: YES"
    )
else:
    print(
        "coarse-pitch saturation at RPM peak: NO"
    )

print(
    "permanent prop/FDM files modified: NO"
)

print()
print(
    "PROP GOVERNOR TRANSIENT DIAGNOSTIC COMPLETE"
)
