#!/usr/bin/env python3

from pathlib import Path
import math

import jsbsim


REPO = Path(__file__).resolve().parents[2]
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0
GROUND_HEIGHT_FT = 4.30
SETTLE_TIME = 2.0

ALTITUDES_FT = (0.0, 5000.0, 10000.0, 15000.0, 20000.0)
MIXTURE_POINTS = (1.00, 0.75)

# Convert atmosphere/P-psf to p_amb / 101325 Pa.
PSF_TO_STANDARD_PRESSURE_RATIO = 0.000472541416

THROTTLE_HANDLE = (
    "systems/powerplant-controls/engine/handles/throttle-norm"
)
MIXTURE_HANDLE = (
    "systems/powerplant-controls/engine/handles/mixture-norm"
)
PROP_HANDLE = (
    "systems/powerplant-controls/engine/handles/prop-norm"
)
MAGNETOS = (
    "systems/powerplant-controls/engine/switches/magnetos"
)
BATTERY = (
    "systems/powerplant-controls/electrical/switches/battery-master"
)

ENGINE_MIXTURE = "fcs/mixture-cmd-norm[0]"
PRESSURE = "atmosphere/P-psf"
AFR = "propulsion/engine[0]/AFR"
RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
RUNNING = "propulsion/engine[0]/set-running"


def get(fdm, prop):
    return fdm.get_property_value(prop)


def run_for(fdm, seconds):
    end = fdm.get_sim_time() + seconds

    while fdm.get_sim_time() < end:
        if not fdm.run():
            raise RuntimeError("JSBSim stopped unexpectedly")


def create_fdm(altitude_ft, mixture):
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
        raise RuntimeError("could not load Mooney FDM")

    # Put the aircraft on an artificial level surface at the requested
    # pressure altitude. This keeps the propulsion test static while
    # allowing JSBSim's atmosphere model to supply altitude-dependent
    # ambient pressure.
    fdm.set_property_value("ic/terrain-elevation-ft", altitude_ft)
    fdm.set_property_value("ic/h-agl-ft", GROUND_HEIGHT_FT)
    fdm.set_property_value("ic/phi-deg", 0.0)
    fdm.set_property_value("ic/theta-deg", 0.0)
    fdm.set_property_value("ic/psi-true-deg", 0.0)
    fdm.set_property_value("ic/vg-kts", 0.0)

    fdm.set_property_value("fcs/left-brake-cmd-norm", 1.0)
    fdm.set_property_value("fcs/right-brake-cmd-norm", 1.0)
    fdm.set_property_value("fcs/center-brake-cmd-norm", 1.0)

    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed")

    fdm.set_property_value(BATTERY, 1)
    fdm.set_property_value(MIXTURE_HANDLE, mixture)
    fdm.set_property_value(PROP_HANDLE, 1.0)
    fdm.set_property_value(THROTTLE_HANDLE, 0.20)
    fdm.set_property_value(MAGNETOS, 3)

    # Let the aircraft systems propagate the pilot-facing controls into
    # JSBSim before forcing the engine into its running state.
    run_for(fdm, 0.1)

    # JSBSim's set-running interface gives us a deterministic engine state
    # without requiring a physically questionable 20,000-ft cold start.
    fdm.set_property_value("propulsion/set-running", 0)

    run_for(fdm, SETTLE_TIME)

    return fdm


print("MOONEY MIXTURE ALTITUDE TEST")
print("============================")
print(f"JSBSim version: {jsbsim.__version__}")
print()

all_pass = True

for pilot_mixture in MIXTURE_POINTS:
    print(f"PILOT MIXTURE {pilot_mixture:.2f}")
    print("--------------------------------------------------------------------------")
    print(
        " altitude   P psf   pressure ratio   engine mix   expected    AFR      RPM"
    )
    print(
        " --------  ------   --------------   ----------   --------   ------  -------"
    )

    afr_values = []

    for altitude_ft in ALTITUDES_FT:
        fdm = create_fdm(altitude_ft, pilot_mixture)

        pressure = get(fdm, PRESSURE)
        pressure_ratio = (
            pressure * PSF_TO_STANDARD_PRESSURE_RATIO
        )

        expected_mix = max(
            0.0,
            min(
                1.0,
                pilot_mixture * pressure_ratio,
            ),
        )

        actual_mix = get(fdm, ENGINE_MIXTURE)
        afr = get(fdm, AFR)
        rpm = get(fdm, RPM)
        running = get(fdm, RUNNING)

        values = (
            pressure,
            pressure_ratio,
            expected_mix,
            actual_mix,
            afr,
            rpm,
            running,
        )

        if not all(math.isfinite(x) for x in values):
            print(
                f"FAIL: non-finite value at "
                f"{altitude_ft:.0f} ft / mixture {pilot_mixture:.2f}"
            )
            all_pass = False
            continue

        mix_error = abs(actual_mix - expected_mix)

        if mix_error > 0.003:
            print(
                f"FAIL: mixture command error {mix_error:.6f} at "
                f"{altitude_ft:.0f} ft"
            )
            all_pass = False

        if running < 0.5:
            print(
                f"FAIL: engine not running at "
                f"{altitude_ft:.0f} ft"
            )
            all_pass = False

        afr_values.append(afr)

        print(
            f"{altitude_ft:8.0f}  "
            f"{pressure:6.1f}   "
            f"{pressure_ratio:14.4f}   "
            f"{actual_mix:10.4f}   "
            f"{expected_mix:8.4f}   "
            f"{afr:6.3f}  "
            f"{rpm:7.1f}"
        )

    if afr_values:
        afr_spread = max(afr_values) - min(afr_values)

        print()
        print(f"AFR altitude spread: {afr_spread:.4f}")

        if afr_spread > 0.25:
            print("AFR invariance:       FAIL")
            all_pass = False
        else:
            print("AFR invariance:       PASS")

    print()


print("RESULT")
print("------")

if all_pass:
    print("engine mixture pressure compensation: PASS")
    print("AFR altitude invariance:              PASS")
    print()
    print("MIXTURE ALTITUDE TEST PASS")
else:
    print("MIXTURE ALTITUDE TEST FAIL")
    raise SystemExit(1)
