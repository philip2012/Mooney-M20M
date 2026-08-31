#!/usr/bin/env python3

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class TrapezoidalPlanform:
    wing_area_sqft: float
    wingspan_ft: float
    taper_ratio: float
    root_chord_ft: float
    tip_chord_ft: float

    @property
    def semi_span_ft(self) -> float:
        return self.wingspan_ft / 2.0


@dataclass(frozen=True)
class BendingSolution:
    y_ft: tuple[float, ...]
    load_lbf_per_ft: tuple[float, ...]
    shear_lbf: tuple[float, ...]
    moment_lbf_ft: tuple[float, ...]
    curvature_per_ft: tuple[float, ...]
    slope_rad: tuple[float, ...]
    deflection_ft: tuple[float, ...]

    @property
    def root_shear_lbf(self) -> float:
        return self.shear_lbf[0]

    @property
    def root_moment_lbf_ft(self) -> float:
        return self.moment_lbf_ft[0]

    @property
    def tip_slope_rad(self) -> float:
        return self.slope_rad[-1]

    @property
    def tip_deflection_ft(self) -> float:
        return self.deflection_ft[-1]


def derive_trapezoidal_planform(
    wing_area_sqft: float,
    wingspan_ft: float,
    taper_ratio: float,
) -> TrapezoidalPlanform:
    """
    Derive root and tip chord for an ideal trapezoidal wing.

    taper_ratio is Ct / Cr.

    Full-wing trapezoidal area:

        S = b * (Cr + Ct) / 2

    therefore:

        Cr = 2S / (b * (1 + lambda))
        Ct = lambda * Cr

    This is a geometric approximation only. It must not replace actual
    Mooney station data if better source geometry becomes available.
    """

    values = (
        wing_area_sqft,
        wingspan_ft,
        taper_ratio,
    )

    if not all(math.isfinite(value) for value in values):
        raise ValueError("Planform inputs must be finite")

    if wing_area_sqft <= 0.0:
        raise ValueError("Wing area must be positive")

    if wingspan_ft <= 0.0:
        raise ValueError("Wingspan must be positive")

    if taper_ratio <= 0.0:
        raise ValueError("Taper ratio must be positive")

    root_chord_ft = (
        2.0
        * wing_area_sqft
        / (
            wingspan_ft
            * (1.0 + taper_ratio)
        )
    )

    tip_chord_ft = (
        taper_ratio
        * root_chord_ft
    )

    return TrapezoidalPlanform(
        wing_area_sqft=wing_area_sqft,
        wingspan_ft=wingspan_ft,
        taper_ratio=taper_ratio,
        root_chord_ft=root_chord_ft,
        tip_chord_ft=tip_chord_ft,
    )


def linear_chord_ft(
    planform: TrapezoidalPlanform,
    y_ft: float,
) -> float:
    """
    Return chord at spanwise location y on one half-wing.

    y = 0              -> wing root
    y = semi-span      -> wing tip
    """

    if not math.isfinite(y_ft):
        raise ValueError("Spanwise position must be finite")

    if not 0.0 <= y_ft <= planform.semi_span_ft:
        raise ValueError(
            "Spanwise position must lie between root and tip"
        )

    fraction = (
        y_ft
        / planform.semi_span_ft
    )

    return (
        planform.root_chord_ft
        + fraction
        * (
            planform.tip_chord_ft
            - planform.root_chord_ft
        )
    )


def make_uniform_span_grid(
    semi_span_ft: float,
    station_count: int,
) -> tuple[float, ...]:
    """
    Build a root-to-tip spanwise grid including both endpoints.
    """

    if not math.isfinite(semi_span_ft):
        raise ValueError("Semi-span must be finite")

    if semi_span_ft <= 0.0:
        raise ValueError("Semi-span must be positive")

    if station_count < 2:
        raise ValueError(
            "At least two span stations are required"
        )

    step = (
        semi_span_ft
        / (station_count - 1)
    )

    return tuple(
        i * step
        for i in range(station_count)
    )


def _validate_bending_inputs(
    y_ft: Sequence[float],
    load_lbf_per_ft: Sequence[float],
    ei_lbf_ft2: Sequence[float],
) -> None:
    count = len(y_ft)

    if count < 2:
        raise ValueError(
            "At least two span stations are required"
        )

    if len(load_lbf_per_ft) != count:
        raise ValueError(
            "Load array length must match span array length"
        )

    if len(ei_lbf_ft2) != count:
        raise ValueError(
            "EI array length must match span array length"
        )

    for name, values in (
        ("span", y_ft),
        ("load", load_lbf_per_ft),
        ("EI", ei_lbf_ft2),
    ):
        if not all(
            math.isfinite(value)
            for value in values
        ):
            raise ValueError(
                f"{name} values must all be finite"
            )

    for i in range(count - 1):
        if y_ft[i + 1] <= y_ft[i]:
            raise ValueError(
                "Span stations must be strictly increasing"
            )

    if any(
        value <= 0.0
        for value in ei_lbf_ft2
    ):
        raise ValueError(
            "EI must be positive at every span station"
        )


def solve_cantilever_bending(
    y_ft: Sequence[float],
    load_lbf_per_ft: Sequence[float],
    ei_lbf_ft2: Sequence[float],
) -> BendingSolution:
    """
    Solve static vertical bending for one cantilever half-wing.

    Coordinate/sign convention
    --------------------------
    y:
        Spanwise distance from wing root, positive outboard.

    load:
        Distributed vertical structural load q(y), positive upward.

    shear:
        Internal shear associated with the outboard load.

    moment:
        Internal bending moment associated with the outboard load.

    deflection:
        Positive upward.

    Boundary conditions
    -------------------
    Wing tip:
        V(L) = 0
        M(L) = 0

    Wing root:
        theta(0) = 0
        w(0) = 0

    Governing equations
    -------------------
        V(y) = integral_y^L q(s) ds

        M(y) = integral_y^L V(s) ds

        curvature(y) = M(y) / EI(y)

        theta(y) = integral_0^y curvature(s) ds

        w(y) = integral_0^y theta(s) ds

    Numerical method
    ----------------
    Trapezoidal integration is used between span stations.

    This function performs only static Euler-Bernoulli bending.
    It deliberately contains no aeroelastic feedback, torsion,
    modal dynamics, damping, or FlightGear/JSBSim property logic.
    """

    _validate_bending_inputs(
        y_ft,
        load_lbf_per_ft,
        ei_lbf_ft2,
    )

    y = tuple(float(value) for value in y_ft)
    load = tuple(
        float(value)
        for value in load_lbf_per_ft
    )
    ei = tuple(
        float(value)
        for value in ei_lbf_ft2
    )

    count = len(y)

    shear = [0.0] * count
    moment = [0.0] * count

    # Integrate load from tip toward root.
    #
    # Tip boundary:
    #   V(L) = 0
    for i in range(
        count - 2,
        -1,
        -1,
    ):
        dx = (
            y[i + 1]
            - y[i]
        )

        strip_load = (
            0.5
            * (
                load[i]
                + load[i + 1]
            )
            * dx
        )

        shear[i] = (
            shear[i + 1]
            + strip_load
        )

    # Integrate shear from tip toward root.
    #
    # Tip boundary:
    #   M(L) = 0
    for i in range(
        count - 2,
        -1,
        -1,
    ):
        dx = (
            y[i + 1]
            - y[i]
        )

        strip_moment = (
            0.5
            * (
                shear[i]
                + shear[i + 1]
            )
            * dx
        )

        moment[i] = (
            moment[i + 1]
            + strip_moment
        )

    curvature = [
        moment[i]
        / ei[i]
        for i in range(count)
    ]

    slope = [0.0] * count
    deflection = [0.0] * count

    # Integrate curvature from root toward tip.
    #
    # Clamped-root boundary:
    #   theta(0) = 0
    for i in range(1, count):
        dx = (
            y[i]
            - y[i - 1]
        )

        slope[i] = (
            slope[i - 1]
            + 0.5
            * (
                curvature[i - 1]
                + curvature[i]
            )
            * dx
        )

    # Integrate slope from root toward tip.
    #
    # Clamped-root boundary:
    #   w(0) = 0
    for i in range(1, count):
        dx = (
            y[i]
            - y[i - 1]
        )

        deflection[i] = (
            deflection[i - 1]
            + 0.5
            * (
                slope[i - 1]
                + slope[i]
            )
            * dx
        )

    return BendingSolution(
        y_ft=y,
        load_lbf_per_ft=load,
        shear_lbf=tuple(shear),
        moment_lbf_ft=tuple(moment),
        curvature_per_ft=tuple(curvature),
        slope_rad=tuple(slope),
        deflection_ft=tuple(deflection),
    )
