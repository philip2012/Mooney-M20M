#!/usr/bin/env python3

from pathlib import Path
import math
import statistics

import jsbsim
import os


REPO = Path(__file__).resolve().parents[2]

ENGINE_DIR = Path(
    os.environ.get(
        "M20M_ENGINE_DIR",
        str(REPO / "Engines"),
    )
)

AIRCRAFT_ROOT = Path(
    os.environ.get(
        "M20M_AIRCRAFT_ROOT",
        str(REPO),
    )
)
MODEL = "FDM/Mooney-M20M"

DT = 1.0 / 120.0

START_ALTITUDE_FT = float(
    os.environ.get(
        "M20M_START_ALT_FT",
        "5000",
    )
)

TARGET_IAS_KTS = tuple(
    float(v)
    for v in os.environ.get(
        "M20M_CLIMB_IAS",
        "90,100,110,120",
    ).split(",")
)

SETTLE_SEC = float(
    os.environ.get(
        "M20M_CLIMB_SETTLE_SEC",
        "25.0",
    )
)

SAMPLE_SEC = 10.0


# ------------------------------------------------------------
# Test-only controller gains.
#
# These are NOT aircraft/autopilot parameters.
# They exist only to establish approximately steady climb
# conditions at a commanded IAS.
# ------------------------------------------------------------

# Outer-loop convergence gain.
#
# Test harness only. This has no connection to the
# aircraft's real control system.
SPEED_PITCH_RATE = 0.040

PITCH_KP = 0.080
PITCH_Q_DAMP = 0.050

ROLL_KP = 0.050
ROLL_P_DAMP = 0.030


THROTTLE = (
    "systems/powerplant-controls/engine/"
    "handles/throttle-norm"
)

MIXTURE = (
    "systems/powerplant-controls/engine/"
    "handles/mixture-norm"
)

PROP = (
    "systems/powerplant-controls/engine/"
    "handles/prop-norm"
)

MAGNETOS = (
    "systems/powerplant-controls/engine/"
    "switches/magnetos"
)

BATTERY = (
    "systems/powerplant-controls/electrical/"
    "switches/battery-master"
)

GEAR_HANDLE = (
    "systems/airframe-controls/gear/handle"
)

FLAP_SELECTOR = (
    "systems/airframe-controls/flaps/selector"
)

RUNNING = "propulsion/engine[0]/set-running"

IAS = "velocities/vc-kts"
TAS = "velocities/vtrue-kts"
HDOT = "velocities/h-dot-fps"

ALTITUDE = "position/h-sl-ft"

THETA = "attitude/theta-deg"
PHI = "attitude/phi-deg"

Q_RATE = "velocities/q-aero-rad_sec"
P_RATE = "velocities/p-aero-rad_sec"

ALPHA = "aero/alpha-deg"

ELEVATOR = "fcs/elevator-cmd-norm"
AILERON = "fcs/aileron-cmd-norm"
RUDDER = "fcs/rudder-cmd-norm"

RPM = "propulsion/engine[0]/propeller-rpm"
MAP = "propulsion/engine[0]/map-inhg"
POWER = "propulsion/engine[0]/power-hp"
THRUST = "propulsion/engine[0]/thrust-lbs"
ADVANCE_RATIO = "propulsion/engine[0]/advance-ratio"
BLADE_ANGLE = "propulsion/engine[0]/blade-angle"

AERO_DRAG = "aero/drag-lbs"
CD = "aero/coefficient/CD"
QBAR = "aero/qbar-psf"

WEIGHT = "inertia/weight-lbs"

GEAR_POS = "gear/gear-pos-norm"
FLAP_POS = "fcs/flap-pos-rad"


def clamp(value, low, high):
    return max(
        low,
        min(
            high,
            value,
        ),
    )


def get(fdm, prop):
    return float(
        fdm.get_property_value(prop)
    )


def setv(fdm, prop, value):
    fdm.set_property_value(
        prop,
        float(value),
    )


def make_fdm(target_ias):
    fdm = jsbsim.FGFDMExec(None)

    fdm.set_debug_level(0)
    fdm.set_dt(DT)

    if not fdm.load_model_with_paths(
        MODEL,
        str(AIRCRAFT_ROOT),
        str(ENGINE_DIR),
        str(REPO / "Systems"),
        False,
    ):
        raise RuntimeError(
            "Production FDM failed to load"
        )

    # --------------------------------------------------------
    # Airborne initial condition.
    # --------------------------------------------------------

    setv(
        fdm,
        "ic/h-sl-ft",
        START_ALTITUDE_FT,
    )

    setv(
        fdm,
        "ic/vc-kts",
        target_ias,
    )

    # A modest initial climb path merely reduces the initial
    # controller transient. It is not a performance assumption.
    setv(
        fdm,
        "ic/gamma-deg",
        5.0,
    )

    setv(
        fdm,
        "ic/phi-deg",
        0.0,
    )

    setv(
        fdm,
        "ic/psi-true-deg",
        0.0,
    )

    # JSBSim-supported all-engines-running initialization.
    setv(
        fdm,
        "propulsion/set-running",
        -1.0,
    )

    if not fdm.run_ic():
        raise RuntimeError(
            "run_ic() failed"
        )

    # --------------------------------------------------------
    # Production configuration.
    # --------------------------------------------------------

    setv(
        fdm,
        BATTERY,
        1.0,
    )

    setv(
        fdm,
        MIXTURE,
        1.0,
    )

    setv(
        fdm,
        PROP,
        1.0,
    )

    setv(
        fdm,
        MAGNETOS,
        3.0,
    )

    setv(
        fdm,
        FLAP_SELECTOR,
        0.0,
    )

    setv(
        fdm,
        GEAR_HANDLE,
        0.0,
    )

    setv(
        fdm,
        RUDDER,
        0.0,
    )

    # Begin below full power and ramp after initialization.
    setv(
        fdm,
        THROTTLE,
        0.50,
    )

    return fdm


def controller_step(
    fdm,
    target_ias,
    theta_cmd,
):
    ias = get(
        fdm,
        IAS,
    )

    theta = get(
        fdm,
        THETA,
    )

    phi = get(
        fdm,
        PHI,
    )

    q_deg_sec = math.degrees(
        get(
            fdm,
            Q_RATE,
        )
    )

    p_deg_sec = math.degrees(
        get(
            fdm,
            P_RATE,
        )
    )

    # --------------------------------------------------------
    # Outer loop:
    #
    # Too fast -> increase commanded pitch.
    # Too slow -> decrease commanded pitch.
    # --------------------------------------------------------

    speed_error = (
        ias
        - target_ias
    )

    theta_cmd += (
        SPEED_PITCH_RATE
        * speed_error
        * DT
    )

    theta_cmd = clamp(
        theta_cmd,
        -3.0,
        25.0,
    )

    # --------------------------------------------------------
    # Inner pitch loop.
    #
    # In this FDM negative elevator command produces the
    # nose-up pitching moment required for a positive pitch
    # error. Pitch-rate feedback damps the response.
    # --------------------------------------------------------

    theta_error = (
        theta_cmd
        - theta
    )

    elevator = (
        -PITCH_KP
        * theta_error

        + PITCH_Q_DAMP
        * q_deg_sec
    )

    elevator = clamp(
        elevator,
        -0.80,
        0.80,
    )

    # --------------------------------------------------------
    # Wings-level controller.
    #
    # Test harness only. No heading controller is used.
    # --------------------------------------------------------

    aileron = (
        -ROLL_KP
        * phi

        - ROLL_P_DAMP
        * p_deg_sec
    )

    aileron = clamp(
        aileron,
        -0.60,
        0.60,
    )

    setv(
        fdm,
        ELEVATOR,
        elevator,
    )

    setv(
        fdm,
        AILERON,
        aileron,
    )

    setv(
        fdm,
        RUDDER,
        0.0,
    )

    return theta_cmd


def run_case(target_ias):
    fdm = make_fdm(
        target_ias,
    )

    # Test-controller feed-forward only.
    #
    # The first diagnostic run showed that slower climb
    # conditions naturally require a larger pitch attitude.
    # Starting every case at 5 deg created an unnecessarily
    # long convergence transient.
    #
    # This value does NOT prescribe aircraft performance;
    # the feedback loop remains responsible for the final
    # steady IAS.
    theta_cmd = (
        6.0
        + 0.24
        * (120.0 - target_ias)
    )

    # --------------------------------------------------------
    # Smoothly bring pilot throttle to full.
    # --------------------------------------------------------

    ramp_frames = int(
        2.0 / DT
    )

    for frame in range(
        ramp_frames
    ):
        fraction = (
            (frame + 1)
            / ramp_frames
        )

        setv(
            fdm,
            THROTTLE,
            0.50
            + 0.50 * fraction,
        )

        theta_cmd = controller_step(
            fdm,
            target_ias,
            theta_cmd,
        )

        if not fdm.run():
            raise RuntimeError(
                "Simulation stopped during throttle ramp"
            )

    # --------------------------------------------------------
    # Settle.
    # --------------------------------------------------------

    settle_frames = int(
        SETTLE_SEC / DT
    )

    for _ in range(
        settle_frames
    ):
        theta_cmd = controller_step(
            fdm,
            target_ias,
            theta_cmd,
        )

        if not fdm.run():
            raise RuntimeError(
                "Simulation stopped during settle"
            )

        if get(fdm, RUNNING) < 0.5:
            raise RuntimeError(
                "Engine stopped"
            )

        ias = get(
            fdm,
            IAS,
        )

        if (
            ias < 65.0
            or ias > 180.0
        ):
            raise RuntimeError(
                f"Controller departure: IAS={ias:.1f}"
            )

    # --------------------------------------------------------
    # Measurement window.
    # --------------------------------------------------------

    samples = []

    start_alt = get(
        fdm,
        ALTITUDE,
    )

    sample_frames = int(
        SAMPLE_SEC / DT
    )

    for _ in range(
        sample_frames
    ):
        theta_cmd = controller_step(
            fdm,
            target_ias,
            theta_cmd,
        )

        if not fdm.run():
            raise RuntimeError(
                "Simulation stopped during sampling"
            )

        tas_kts = get(
            fdm,
            TAS,
        )

        hdot_fps = get(
            fdm,
            HDOT,
        )

        tas_fps = (
            tas_kts
            * 1.687809857
        )

        if tas_fps > 1.0:
            ratio = clamp(
                hdot_fps / tas_fps,
                -1.0,
                1.0,
            )

            gamma_deg = math.degrees(
                math.asin(
                    ratio
                )
            )
        else:
            gamma_deg = 0.0

        samples.append(
            {
                "ias": get(fdm, IAS),
                "tas": tas_kts,
                "vs": hdot_fps * 60.0,
                "gamma": gamma_deg,
                "alpha": get(fdm, ALPHA),
                "theta": get(fdm, THETA),
                "elevator": get(fdm, ELEVATOR),
                "phi": get(fdm, PHI),
                "rpm": get(fdm, RPM),
                "map": get(fdm, MAP),
                "hp": get(fdm, POWER),
                "thrust": get(fdm, THRUST),
                "advance_ratio": get(fdm, ADVANCE_RATIO),
                "blade_angle": get(fdm, BLADE_ANGLE),
                "drag": get(fdm, AERO_DRAG),
                "cd": get(fdm, CD),
                "qbar": get(fdm, QBAR),
                "alt": get(fdm, ALTITUDE),
                "weight": get(fdm, WEIGHT),
                "gear": get(fdm, GEAR_POS),
                "flap": get(fdm, FLAP_POS),
            }
        )

    end_alt = get(
        fdm,
        ALTITUDE,
    )

    def avg(name):
        return statistics.mean(
            row[name]
            for row in samples
        )

    def std(name):
        return statistics.pstdev(
            row[name]
            for row in samples
        )

    # Compare the first and last 20 percent of the
    # measurement window. Standard deviation alone
    # cannot distinguish oscillation from a slow drift.
    trend_count = max(
        1,
        len(samples) // 5,
    )

    ias_first = statistics.mean(
        row["ias"]
        for row in samples[:trend_count]
    )

    ias_last = statistics.mean(
        row["ias"]
        for row in samples[-trend_count:]
    )

    ias_drift = (
        ias_last
        - ias_first
    )

    tas_avg = avg("tas")
    thrust_avg = avg("thrust")
    drag_avg = avg("drag")
    weight_avg = avg("weight")
    vs_avg = avg("vs")

    tas_fps_avg = (
        tas_avg
        * 1.687809857
    )

    prop_hp = (
        thrust_avg
        * tas_fps_avg
        / 550.0
    )

    drag_hp = (
        drag_avg
        * tas_fps_avg
        / 550.0
    )

    climb_hp = (
        weight_avg
        * vs_avg
        / 33000.0
    )

    power_residual_hp = (
        prop_hp
        - drag_hp
        - climb_hp
    )

    energy_vs = (
        vs_avg
        + (
            power_residual_hp
            * 33000.0
            / weight_avg
        )
    )

    return {
        "target": target_ias,

        "ias": avg("ias"),
        "ias_std": std("ias"),
        "ias_drift": ias_drift,

        "tas": tas_avg,

        "vs": avg("vs"),
        "vs_std": std("vs"),

        "vs_alt": (
            end_alt
            - start_alt
        ) / SAMPLE_SEC * 60.0,

        "gamma": avg("gamma"),

        "alpha": avg("alpha"),
        "theta": avg("theta"),

        "elevator": avg("elevator"),

        "phi_abs": max(
            abs(row["phi"])
            for row in samples
        ),

        "rpm": avg("rpm"),
        "map": avg("map"),
        "hp": avg("hp"),
        "thrust": avg("thrust"),
        "advance_ratio": avg("advance_ratio"),
        "blade_angle": avg("blade_angle"),
        "drag": avg("drag"),
        "cd": avg("cd"),
        "qbar": avg("qbar"),

        "prop_hp": prop_hp,
        "drag_hp": drag_hp,
        "climb_hp": climb_hp,
        "power_residual_hp": power_residual_hp,
        "energy_vs": energy_vs,

        "alt": avg("alt"),
        "weight": avg("weight"),

        "gear": max(
            row["gear"]
            for row in samples
        ),

        "flap": max(
            abs(row["flap"])
            for row in samples
        ),
    }


def main():
    print(
        "MOONEY M20M PRODUCTION "
        "CLIMB SPEED SWEEP"
    )

    print("=" * 100)

    print(
        f"JSBSim version: "
        f"{jsbsim.__version__}"
    )

    print(
        "Production FDM, full pilot throttle, "
        "full prop, mixture full."
    )

    print(
        "Controller is test-only and modifies "
        "elevator/aileron commands only."
    )

    print()

    results = []

    for target in TARGET_IAS_KTS:
        print(
            f"Running {target:.0f} KIAS..."
        )

        results.append(
            run_case(
                target,
            )
        )

    print()
    print(
        " tgt   IAS   sIAS    TAS"
        "      VS    sVS   VSalt"
        "  gamma  alpha  theta"
        "    elev"
    )

    print(
        "----  ----  -----  -----"
        "  ------  -----  ------"
        "  -----  -----  -----"
        "  ------"
    )

    for r in results:
        print(
            f"{r['target']:4.0f} "
            f"{r['ias']:5.1f} "
            f"{r['ias_std']:6.2f} "
            f"{r['tas']:6.1f} "
            f"{r['vs']:7.0f} "
            f"{r['vs_std']:6.1f} "
            f"{r['vs_alt']:7.0f} "
            f"{r['gamma']:6.2f} "
            f"{r['alpha']:6.2f} "
            f"{r['theta']:6.2f} "
            f"{r['elevator']:7.3f}"
        )

    print()
    print(
        " tgt     alt     wt     RPM"
        "     MAP      HP   |phi|"
        "   gear   flap"
    )

    print(
        "----  ------  ------  -------"
        "  ------  ------  ------"
        "  -----  -----"
    )

    for r in results:
        print(
            f"{r['target']:4.0f} "
            f"{r['alt']:7.0f} "
            f"{r['weight']:7.1f} "
            f"{r['rpm']:8.1f} "
            f"{r['map']:7.2f} "
            f"{r['hp']:7.2f} "
            f"{r['phi_abs']:7.2f} "
            f"{r['gear']:6.3f} "
            f"{r['flap']:6.3f}"
        )

    print()
    print(
        " tgt      J   blade  thrust    drag"
        "   propHP  dragHP    eta  climbHP"
        "      CD    qbar"
    )

    print(
        "----  -----  ------  ------  ------"
        "  -------  ------  -----  -------"
        "  ------  ------"
    )

    for r in results:
        tas_fps = r["tas"] * 1.687809857

        prop_hp = (
            r["thrust"] * tas_fps / 550.0
        )

        drag_hp = (
            r["drag"] * tas_fps / 550.0
        )

        eta = (
            prop_hp / r["hp"]
            if r["hp"] > 1.0
            else 0.0
        )

        climb_hp = (
            r["weight"] * r["vs"] / 33000.0
        )

        print(
            f"{r['target']:4.0f} "
            f"{r['advance_ratio']:6.3f} "
            f"{r['blade_angle']:7.2f} "
            f"{r['thrust']:7.1f} "
            f"{r['drag']:7.1f} "
            f"{prop_hp:8.1f} "
            f"{drag_hp:7.1f} "
            f"{eta:6.3f} "
            f"{climb_hp:8.1f} "
            f"{r['cd']:7.4f} "
            f"{r['qbar']:7.2f}"
        )

    print()
    print("VALIDITY CHECK")

    valid = True

    for r in results:
        case_ok = (
            abs(
                r["ias"]
                - r["target"]
            ) <= 2.0

            and r["ias_std"] <= 1.5

            and abs(
                r["ias_drift"]
            ) <= 0.5

            and abs(
                r["power_residual_hp"]
            ) <= 10.0

            and abs(
                r["vs"]
                - r["vs_alt"]
            ) <= 75.0

            and r["gear"] <= 0.01

            and r["flap"] <= 0.01

            and r["phi_abs"] <= 10.0
        )

        print(
            f"{r['target']:4.0f} KIAS: "
            + (
                "VALID"
                if case_ok
                else "NOT SETTLED"
            )
            + (
                f"  drift={r['ias_drift']:+.2f} kt"
                f"  residual={r['power_residual_hp']:+.1f} hp"
                f"  energyVS={r['energy_vs']:.0f} fpm"
            )
        )

        valid = (
            valid
            and case_ok
        )

    print()

    if valid:
        print(
            "CLIMB SPEED SWEEP "
            "MEASUREMENT VALID"
        )
    else:
        print(
            "CLIMB SPEED SWEEP NOT YET "
            "VALID FOR PERFORMANCE JUDGMENT"
        )

        print(
            "Adjust the TEST HARNESS controller; "
            "do not tune aircraft coefficients."
        )


if __name__ == "__main__":
    main()
