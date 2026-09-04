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


@dataclass(frozen=True)
class WingStrip:
    index: int

    y_inner_ft: float
    y_outer_ft: float
    y_centroid_ft: float

    chord_inner_ft: float
    chord_outer_ft: float

    area_sqft: float

    @property
    def width_ft(self) -> float:
        return (
            self.y_outer_ft
            - self.y_inner_ft
        )


def make_trapezoidal_strips(
    planform: TrapezoidalPlanform,
    strip_count: int,
) -> tuple[WingStrip, ...]:
    """
    Divide one half-wing into trapezoidal spanwise strips.

    This currently uses the idealized trapezoidal planform derived
    from the published wing area, span, and taper ratio.

    Replace with actual Mooney station geometry later if better
    source data becomes available.
    """

    if strip_count < 1:
        raise ValueError(
            "At least one wing strip is required"
        )

    edges = make_uniform_span_grid(
        planform.semi_span_ft,
        strip_count + 1,
    )

    strips = []

    for index in range(strip_count):
        y_inner = edges[index]
        y_outer = edges[index + 1]

        chord_inner = linear_chord_ft(
            planform,
            y_inner,
        )

        chord_outer = linear_chord_ft(
            planform,
            y_outer,
        )

        width = (
            y_outer
            - y_inner
        )

        area = (
            0.5
            * (
                chord_inner
                + chord_outer
            )
            * width
        )

        # Exact spanwise centroid for a strip whose chord varies
        # linearly between the two edges.
        centroid_offset = (
            width
            * (
                chord_inner
                + 2.0 * chord_outer
            )
            / (
                3.0
                * (
                    chord_inner
                    + chord_outer
                )
            )
        )

        strips.append(
            WingStrip(
                index=index,
                y_inner_ft=y_inner,
                y_outer_ft=y_outer,
                y_centroid_ft=(
                    y_inner
                    + centroid_offset
                ),
                chord_inner_ft=chord_inner,
                chord_outer_ft=chord_outer,
                area_sqft=area,
            )
        )

    return tuple(strips)


def sectional_lift_lbf_per_ft(
    y_ft: Sequence[float],
    chord_ft: Sequence[float],
    cl: Sequence[float],
    qbar_psf: float,
) -> tuple[float, ...]:
    """
    Calculate local aerodynamic lift per unit span.

        L'(y) = qbar * c(y) * CL(y)

    Units:

        lb/ft = lb/ft^2 * ft

    Positive CL gives positive upward loading.
    """

    count = len(y_ft)

    if count < 2:
        raise ValueError(
            "At least two span stations are required"
        )

    if len(chord_ft) != count:
        raise ValueError(
            "Chord array length must match span array length"
        )

    if len(cl) != count:
        raise ValueError(
            "CL array length must match span array length"
        )

    if not math.isfinite(qbar_psf):
        raise ValueError(
            "Dynamic pressure must be finite"
        )

    if qbar_psf < 0.0:
        raise ValueError(
            "Dynamic pressure cannot be negative"
        )

    for name, values in (
        ("span", y_ft),
        ("chord", chord_ft),
        ("CL", cl),
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
        for value in chord_ft
    ):
        raise ValueError(
            "Chord must be positive"
        )

    return tuple(
        qbar_psf
        * chord_ft[i]
        * cl[i]
        for i in range(count)
    )


def integrate_distributed_load(
    y_ft: Sequence[float],
    load_lbf_per_ft: Sequence[float],
) -> float:
    """
    Integrate a spanwise distributed load using the trapezoidal rule.
    """

    count = len(y_ft)

    if count < 2:
        raise ValueError(
            "At least two span stations are required"
        )

    if len(load_lbf_per_ft) != count:
        raise ValueError(
            "Load array length must match span array length"
        )

    if not all(
        math.isfinite(value)
        for value in y_ft
    ):
        raise ValueError(
            "Span values must all be finite"
        )

    if not all(
        math.isfinite(value)
        for value in load_lbf_per_ft
    ):
        raise ValueError(
            "Load values must all be finite"
        )

    total = 0.0

    for i in range(count - 1):
        dx = (
            y_ft[i + 1]
            - y_ft[i]
        )

        if dx <= 0.0:
            raise ValueError(
                "Span stations must be strictly increasing"
            )

        total += (
            0.5
            * (
                load_lbf_per_ft[i]
                + load_lbf_per_ft[i + 1]
            )
            * dx
        )

    return total


@dataclass(frozen=True)
class LocalSectionFlow:
    signed_y_ft: float

    forward_speed_fps: float
    roll_normal_speed_fps: float
    local_speed_fps: float

    reference_alpha_rad: float
    geometric_twist_rad: float
    aeroelastic_twist_rad: float
    roll_delta_alpha_rad: float

    effective_alpha_rad: float


def local_section_flow(
    *,
    reference_alpha_rad: float,
    forward_speed_fps: float,
    roll_rate_rad_s: float,
    signed_y_ft: float,
    geometric_twist_rad: float = 0.0,
    aeroelastic_twist_rad: float = 0.0,
) -> LocalSectionFlow:
    """
    Calculate local section airflow caused by rigid-body roll.

    Coordinate convention used by this structural model:

        signed_y_ft < 0 : left wing
        signed_y_ft = 0 : aircraft centerline
        signed_y_ft > 0 : right wing

    Positive roll rate is defined as the right wing moving downward.

    Therefore the rigid-body vertical velocity of a span station is:

        w_roll = p * y

    and its corresponding local effective-angle contribution is:

        delta_alpha_roll = atan2(w_roll, V)

    Positive geometric/aeroelastic twist is defined here as increasing
    local aerodynamic incidence and therefore increasing effective AoA.

    This function intentionally models only roll-induced local flow and
    twist bookkeeping. Gusts, induced downwash, sideslip, yaw rate,
    structural dynamics, and aerodynamic feedback are separate later
    additions.
    """

    values = (
        reference_alpha_rad,
        forward_speed_fps,
        roll_rate_rad_s,
        signed_y_ft,
        geometric_twist_rad,
        aeroelastic_twist_rad,
    )

    if not all(
        math.isfinite(value)
        for value in values
    ):
        raise ValueError(
            "Local-flow inputs must all be finite"
        )

    if forward_speed_fps <= 0.0:
        raise ValueError(
            "Forward airspeed must be positive"
        )

    roll_normal_speed_fps = (
        roll_rate_rad_s
        * signed_y_ft
    )

    roll_delta_alpha_rad = math.atan2(
        roll_normal_speed_fps,
        forward_speed_fps,
    )

    local_speed_fps = math.hypot(
        forward_speed_fps,
        roll_normal_speed_fps,
    )

    effective_alpha_rad = (
        reference_alpha_rad
        + geometric_twist_rad
        + aeroelastic_twist_rad
        + roll_delta_alpha_rad
    )

    return LocalSectionFlow(
        signed_y_ft=signed_y_ft,
        forward_speed_fps=forward_speed_fps,
        roll_normal_speed_fps=roll_normal_speed_fps,
        local_speed_fps=local_speed_fps,
        reference_alpha_rad=reference_alpha_rad,
        geometric_twist_rad=geometric_twist_rad,
        aeroelastic_twist_rad=aeroelastic_twist_rad,
        roll_delta_alpha_rad=roll_delta_alpha_rad,
        effective_alpha_rad=effective_alpha_rad,
    )
