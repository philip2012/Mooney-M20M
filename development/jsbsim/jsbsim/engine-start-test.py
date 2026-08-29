#!/usr/bin/env python3

from pathlib import Path
import math

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
SETTLE_TIME = 5.0
MAX_START_TIME = 8.0
POST_START_TIME = 3.0
MAX_SHUTDOWN_TIME = 6.0

ENGINE_RUNNING = "propulsion/engine[0]/set-running"
PROP_RPM = "propulsion/engine[0]/propeller-rpm"
POWER_HP = "propulsion/engine[0]/power-hp"
MAP_INHG = "propulsion/engine[0]/map-inhg"

START_ALLOWED = (
    "systems/powerplant-controls/engine/states/start-allowed"
)
STARTER_CMD = "propulsion/starter_cmd"


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


def value(fdm, prop):
    return fdm.get_property_value(prop)


def report(fdm, label):
    vals = [
        value(fdm, PROP_RPM),
        value(fdm, POWER_HP),
        value(fdm, MAP_INHG),
    ]

    if not all(math.isfinite(x) for x in vals):
        raise SystemExit("FAIL: non-finite engine state")

    print(
        f"{label:18s} "
        f"t={fdm.get_sim_time():7.3f}  "
        f"allowed={value(fdm, START_ALLOWED):.0f}  "
        f"starter={value(fdm, STARTER_CMD):.0f}  "
        f"running={value(fdm, ENGINE_RUNNING):.0f}  "
        f"rpm={value(fdm, PROP_RPM):8.1f}  "
        f"hp={value(fdm, POWER_HP):8.2f}  "
        f"MAP={value(fdm, MAP_INHG):6.2f}"
    )


fdm = make_fdm()

# ------------------------------------------------------------
# Initial ground state.
# ------------------------------------------------------------

fdm.set_property_value("ic/terrain-elevation-ft", 0.0)
fdm.set_property_value("ic/h-agl-ft", 4.30)
fdm.set_property_value("ic/phi-deg", 0.0)
fdm.set_property_value("ic/theta-deg", 0.0)
fdm.set_property_value("ic/psi-true-deg", 0.0)
fdm.set_property_value("ic/vg-kts", 0.0)

fdm.set_property_value("fcs/left-brake-cmd-norm", 1.0)
fdm.set_property_value("fcs/right-brake-cmd-norm", 1.0)
fdm.set_property_value("fcs/center-brake-cmd-norm", 1.0)

if not fdm.run_ic():
    raise SystemExit("FAIL: run_ic() failed")

while fdm.get_sim_time() < SETTLE_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: ground settle stopped")


# ------------------------------------------------------------
# Cold-start configuration.
# ------------------------------------------------------------

fdm.set_property_value(
    "systems/powerplant-controls/electrical/switches/battery-master",
    1,
)
fdm.set_property_value(
    "systems/powerplant-controls/electrical/switches/alternator",
    0,
)
fdm.set_property_value(
    "systems/powerplant-controls/engine/handles/mixture-norm",
    1.0,
)
fdm.set_property_value(
    "systems/powerplant-controls/engine/handles/prop-norm",
    1.0,
)
fdm.set_property_value(
    "systems/powerplant-controls/engine/handles/throttle-norm",
    0.15,
)
fdm.set_property_value(
    "systems/powerplant-controls/engine/switches/magnetos",
    3,
)
fdm.set_property_value(
    "systems/powerplant-controls/engine/switches/starter",
    1,
)


print("MOONEY COLD ENGINE START TEST")
print("=============================")

start_begin = fdm.get_sim_time()
next_report = start_begin
running_time = None

while fdm.get_sim_time() - start_begin < MAX_START_TIME:
    if not fdm.run():
        raise SystemExit("FAIL: simulation stopped during start")

    if fdm.get_sim_time() + DT / 2.0 >= next_report:
        report(fdm, "cranking")
        next_report += 0.5

    if value(fdm, ENGINE_RUNNING) >= 0.5:
        running_time = fdm.get_sim_time()
        break

if running_time is None:
    report(fdm, "start failed")
    raise SystemExit("ENGINE START FAIL")


print()
report(fdm, "ENGINE STARTED")


# ------------------------------------------------------------
# Starter release.
# ------------------------------------------------------------

fdm.set_property_value(
    "systems/powerplant-controls/engine/switches/starter",
    0,
)

post_start_begin = fdm.get_sim_time()
next_report = post_start_begin

while fdm.get_sim_time() - post_start_begin < POST_START_TIME:
    if not fdm.run():
        raise SystemExit(
            "FAIL: simulation stopped after engine start"
        )

    if fdm.get_sim_time() + DT / 2.0 >= next_report:
        report(fdm, "starter released")
        next_report += 0.5

    if value(fdm, ENGINE_RUNNING) < 0.5:
        report(fdm, "engine died")
        raise SystemExit(
            "FAIL: engine died after starter release"
        )

if value(fdm, STARTER_CMD) != 0.0:
    raise SystemExit(
        "FAIL: starter command remained engaged"
    )

print()
print("STARTER RELEASE: PASS")


# ------------------------------------------------------------
# Magneto shutdown.
# ------------------------------------------------------------

fdm.set_property_value(
    "systems/powerplant-controls/engine/switches/magnetos",
    0,
)

shutdown_begin = fdm.get_sim_time()
next_report = shutdown_begin
shutdown_time = None

while fdm.get_sim_time() - shutdown_begin < MAX_SHUTDOWN_TIME:
    if not fdm.run():
        raise SystemExit(
            "FAIL: simulation stopped during shutdown"
        )

    if fdm.get_sim_time() + DT / 2.0 >= next_report:
        report(fdm, "magnetos off")
        next_report += 0.5

    if value(fdm, ENGINE_RUNNING) < 0.5:
        shutdown_time = fdm.get_sim_time()
        break

if shutdown_time is None:
    report(fdm, "shutdown failed")
    raise SystemExit("MAGNETO SHUTDOWN FAIL")


print()
report(fdm, "ENGINE STOPPED")

print()
print("RESULT")
print("------")
print(
    f"start time:     "
    f"{running_time - start_begin:.3f} s"
)
print(
    f"starter cmd:    "
    f"{value(fdm, STARTER_CMD):.0f}"
)
print(
    f"running final:  "
    f"{value(fdm, ENGINE_RUNNING):.0f}"
)
print(
    f"shutdown time:  "
    f"{shutdown_time - shutdown_begin:.3f} s"
)

print()
print("COLD ENGINE START TEST PASS")
