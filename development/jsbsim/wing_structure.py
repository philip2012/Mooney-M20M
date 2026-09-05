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


@dataclass(frozen=True)
class LiftingLineSolution:
    theta_rad: tuple[float, ...]
    signed_y_ft: tuple[float, ...]
    chord_ft: tuple[float, ...]

    alpha_geometric_rad: tuple[float, ...]
    alpha_zero_lift_rad: tuple[float, ...]
    induced_alpha_rad: tuple[float, ...]

    section_cl: tuple[float, ...]

    fourier_coefficients: tuple[float, ...]

    wing_cl: float
    induced_cd: float
    span_efficiency: float


def make_lifting_line_collocation(
    wingspan_ft: float,
    station_count: int,
) -> tuple[
    tuple[float, ...],
    tuple[float, ...],
]:
    """
    Create full-wing Prandtl lifting-line collocation stations.

    theta runs from just above 0 to just below pi.

    Span convention:

        y < 0 : left wing
        y = 0 : centerline
        y > 0 : right wing

    using:

        y = -(b / 2) cos(theta)

    Tip endpoints themselves are deliberately excluded because the
    lifting-line equations contain terms divided by sin(theta).
    """

    if not math.isfinite(wingspan_ft):
        raise ValueError(
            "Wingspan must be finite"
        )

    if wingspan_ft <= 0.0:
        raise ValueError(
            "Wingspan must be positive"
        )

    if station_count < 3:
        raise ValueError(
            "At least three lifting-line stations are required"
        )

    theta = tuple(
        (i + 1)
        * math.pi
        / (station_count + 1)
        for i in range(station_count)
    )

    signed_y = tuple(
        -0.5
        * wingspan_ft
        * math.cos(angle)
        for angle in theta
    )

    return theta, signed_y


def _solve_linear_system(
    matrix: Sequence[Sequence[float]],
    rhs: Sequence[float],
) -> tuple[float, ...]:
    """
    Solve Ax = b using Gaussian elimination with partial pivoting.

    Kept local and dependency-free so the wing prototype does not
    require NumPy.
    """

    count = len(rhs)

    if len(matrix) != count:
        raise ValueError(
            "Matrix row count must match RHS length"
        )

    a = [
        [float(value) for value in row]
        for row in matrix
    ]

    b = [
        float(value)
        for value in rhs
    ]

    if any(
        len(row) != count
        for row in a
    ):
        raise ValueError(
            "Linear-system matrix must be square"
        )

    for column in range(count):
        pivot = max(
            range(column, count),
            key=lambda row: abs(
                a[row][column]
            ),
        )

        pivot_value = a[pivot][column]

        if abs(pivot_value) < 1e-14:
            raise ValueError(
                "Lifting-line system is singular"
            )

        if pivot != column:
            a[column], a[pivot] = (
                a[pivot],
                a[column],
            )

            b[column], b[pivot] = (
                b[pivot],
                b[column],
            )

        for row in range(
            column + 1,
            count,
        ):
            factor = (
                a[row][column]
                / a[column][column]
            )

            if factor == 0.0:
                continue

            a[row][column] = 0.0

            for j in range(
                column + 1,
                count,
            ):
                a[row][j] -= (
                    factor
                    * a[column][j]
                )

            b[row] -= (
                factor
                * b[column]
            )

    solution = [0.0] * count

    for row in range(
        count - 1,
        -1,
        -1,
    ):
        residual = b[row]

        for column in range(
            row + 1,
            count,
        ):
            residual -= (
                a[row][column]
                * solution[column]
            )

        solution[row] = (
            residual
            / a[row][row]
        )

    return tuple(solution)


def lifting_line_series(
    coefficients: Sequence[float],
    theta_rad: float,
) -> float:
    """
    Evaluate:

        sum(A_n sin(n theta))
    """

    if not math.isfinite(theta_rad):
        raise ValueError(
            "Theta must be finite"
        )

    return sum(
        coefficient
        * math.sin(
            harmonic
            * theta_rad
        )
        for harmonic, coefficient in enumerate(
            coefficients,
            start=1,
        )
    )


def solve_lifting_line(
    *,
    wingspan_ft: float,
    wing_area_sqft: float,
    theta_rad: Sequence[float],
    chord_ft: Sequence[float],
    alpha_geometric_rad: Sequence[float],
    lift_curve_slope_per_rad: Sequence[float],
    alpha_zero_lift_rad: Sequence[float] | None = None,
) -> LiftingLineSolution:
    """
    Solve the classical Prandtl lifting-line equations.

    The model supports arbitrary full-wing distributions of:

        chord
        geometric/effective incidence
        2-D lift-curve slope
        zero-lift AoA

    so asymmetric roll, ailerons, gusts, and aeroelastic twist can
    later be supplied without changing the solver architecture.

    This is a linear attached-flow lifting-line solver.

    It does NOT yet model:

        section stall
        nonlinear airfoil polars
        viscous profile drag
        compressibility
        dynamic stall
        unsteady aerodynamics
    """

    count = len(theta_rad)

    if count < 3:
        raise ValueError(
            "At least three lifting-line stations are required"
        )

    arrays = (
        chord_ft,
        alpha_geometric_rad,
        lift_curve_slope_per_rad,
    )

    if any(
        len(values) != count
        for values in arrays
    ):
        raise ValueError(
            "All lifting-line arrays must have equal length"
        )

    if alpha_zero_lift_rad is None:
        alpha_zero_lift = tuple(
            0.0
            for _ in range(count)
        )
    else:
        if len(alpha_zero_lift_rad) != count:
            raise ValueError(
                "Zero-lift AoA array length must match stations"
            )

        alpha_zero_lift = tuple(
            float(value)
            for value in alpha_zero_lift_rad
        )

    scalar_values = (
        wingspan_ft,
        wing_area_sqft,
    )

    if not all(
        math.isfinite(value)
        for value in scalar_values
    ):
        raise ValueError(
            "Wing dimensions must be finite"
        )

    if wingspan_ft <= 0.0:
        raise ValueError(
            "Wingspan must be positive"
        )

    if wing_area_sqft <= 0.0:
        raise ValueError(
            "Wing area must be positive"
        )

    all_arrays = (
        theta_rad,
        chord_ft,
        alpha_geometric_rad,
        lift_curve_slope_per_rad,
        alpha_zero_lift,
    )

    if not all(
        math.isfinite(value)
        for values in all_arrays
        for value in values
    ):
        raise ValueError(
            "Lifting-line inputs must all be finite"
        )

    if any(
        value <= 0.0
        for value in chord_ft
    ):
        raise ValueError(
            "Chord must be positive at every station"
        )

    if any(
        value <= 0.0
        for value in lift_curve_slope_per_rad
    ):
        raise ValueError(
            "Lift-curve slope must be positive"
        )

    theta = tuple(
        float(value)
        for value in theta_rad
    )

    for angle in theta:
        if not 0.0 < angle < math.pi:
            raise ValueError(
                "Collocation theta must lie inside (0, pi)"
            )

    for i in range(count - 1):
        if theta[i + 1] <= theta[i]:
            raise ValueError(
                "Theta stations must be strictly increasing"
            )

    chord = tuple(
        float(value)
        for value in chord_ft
    )

    alpha = tuple(
        float(value)
        for value in alpha_geometric_rad
    )

    a0 = tuple(
        float(value)
        for value in lift_curve_slope_per_rad
    )

    matrix = []
    rhs = []

    for i in range(count):
        sin_theta = math.sin(
            theta[i]
        )

        row = []

        for harmonic in range(
            1,
            count + 1,
        ):
            sin_n_theta = math.sin(
                harmonic
                * theta[i]
            )

            row.append(
                sin_n_theta
                * (
                    (
                        4.0
                        * wingspan_ft
                        / (
                            a0[i]
                            * chord[i]
                        )
                    )
                    + (
                        harmonic
                        / sin_theta
                    )
                )
            )

        matrix.append(row)

        rhs.append(
            alpha[i]
            - alpha_zero_lift[i]
        )

    coefficients = _solve_linear_system(
        matrix,
        rhs,
    )

    signed_y = tuple(
        -0.5
        * wingspan_ft
        * math.cos(angle)
        for angle in theta
    )

    circulation_shape = tuple(
        lifting_line_series(
            coefficients,
            angle,
        )
        for angle in theta
    )

    induced_alpha = []

    section_cl = []

    for i, angle in enumerate(theta):
        sin_theta = math.sin(
            angle
        )

        induced = sum(
            harmonic
            * coefficient
            * math.sin(
                harmonic
                * angle
            )
            / sin_theta
            for harmonic, coefficient in enumerate(
                coefficients,
                start=1,
            )
        )

        induced_alpha.append(
            induced
        )

        section_cl.append(
            (
                4.0
                * wingspan_ft
                / chord[i]
            )
            * circulation_shape[i]
        )

    aspect_ratio = (
        wingspan_ft ** 2
        / wing_area_sqft
    )

    wing_cl = (
        math.pi
        * aspect_ratio
        * coefficients[0]
    )

    induced_cd = (
        math.pi
        * aspect_ratio
        * sum(
            harmonic
            * coefficient ** 2
            for harmonic, coefficient in enumerate(
                coefficients,
                start=1,
            )
        )
    )

    if induced_cd > 1e-16:
        span_efficiency = (
            wing_cl ** 2
            / (
                math.pi
                * aspect_ratio
                * induced_cd
            )
        )
    else:
        span_efficiency = 1.0

    return LiftingLineSolution(
        theta_rad=theta,
        signed_y_ft=signed_y,
        chord_ft=chord,
        alpha_geometric_rad=alpha,
        alpha_zero_lift_rad=alpha_zero_lift,
        induced_alpha_rad=tuple(
            induced_alpha
        ),
        section_cl=tuple(
            section_cl
        ),
        fourier_coefficients=coefficients,
        wing_cl=wing_cl,
        induced_cd=induced_cd,
        span_efficiency=span_efficiency,
    )


@dataclass(frozen=True)
class HalfWingAerodynamicLoad:
    side: str

    y_ft: tuple[float, ...]
    chord_ft: tuple[float, ...]
    section_cl: tuple[float, ...]
    lift_lbf_per_ft: tuple[float, ...]

    total_lift_lbf: float


@dataclass(frozen=True)
class OneWayAeroStructuralSolution:
    left_load: HalfWingAerodynamicLoad
    right_load: HalfWingAerodynamicLoad

    left_bending: BendingSolution
    right_bending: BendingSolution


def extract_half_wing_load(
    lifting_line: LiftingLineSolution,
    *,
    side: str,
    qbar_psf: float,
    semi_span_ft: float,
    tip_chord_ft: float,
) -> HalfWingAerodynamicLoad:
    """
    Convert a full-wing lifting-line solution into one root-to-tip
    structural load distribution.

    The lifting-line collocation points exclude the physical tips,
    so an explicit tip station is appended with zero circulation/lift.

    Structural convention:

        y = 0          wing root
        y = semi-span  wing tip

    regardless of whether the selected side is left or right.
    """

    if side not in (
        "left",
        "right",
    ):
        raise ValueError(
            "Wing side must be 'left' or 'right'"
        )

    values = (
        qbar_psf,
        semi_span_ft,
        tip_chord_ft,
    )

    if not all(
        math.isfinite(value)
        for value in values
    ):
        raise ValueError(
            "Half-wing load inputs must be finite"
        )

    if qbar_psf < 0.0:
        raise ValueError(
            "Dynamic pressure cannot be negative"
        )

    if semi_span_ft <= 0.0:
        raise ValueError(
            "Semi-span must be positive"
        )

    if tip_chord_ft <= 0.0:
        raise ValueError(
            "Tip chord must be positive"
        )

    stations = []

    for signed_y, chord, section_cl in zip(
        lifting_line.signed_y_ft,
        lifting_line.chord_ft,
        lifting_line.section_cl,
    ):
        if side == "right":
            if signed_y < -1e-12:
                continue

            structural_y = signed_y

        else:
            if signed_y > 1e-12:
                continue

            structural_y = -signed_y

        stations.append(
            (
                structural_y,
                chord,
                section_cl,
            )
        )

    stations.sort(
        key=lambda row: row[0]
    )

    if len(stations) < 2:
        raise ValueError(
            "Not enough half-wing lifting-line stations"
        )

    if abs(stations[0][0]) > 1e-10:
        raise ValueError(
            "Lifting-line grid must contain a centerline station"
        )

    if stations[-1][0] >= semi_span_ft:
        raise ValueError(
            "Lifting-line collocation station lies at or beyond tip"
        )

    # The physical tip is not a collocation point.
    #
    # Prandtl lifting-line enforces:
    #
    #     Gamma(tip) = 0
    #
    # therefore sectional lift also goes to zero there.
    stations.append(
        (
            semi_span_ft,
            tip_chord_ft,
            0.0,
        )
    )

    y = tuple(
        row[0]
        for row in stations
    )

    chord = tuple(
        row[1]
        for row in stations
    )

    section_cl = tuple(
        row[2]
        for row in stations
    )

    lift = tuple(
        qbar_psf
        * chord_value
        * cl_value
        for chord_value, cl_value in zip(
            chord,
            section_cl,
        )
    )

    total_lift = integrate_distributed_load(
        y,
        lift,
    )

    return HalfWingAerodynamicLoad(
        side=side,
        y_ft=y,
        chord_ft=chord,
        section_cl=section_cl,
        lift_lbf_per_ft=lift,
        total_lift_lbf=total_lift,
    )


def solve_one_way_aero_structural_bending(
    *,
    lifting_line: LiftingLineSolution,
    qbar_psf: float,
    semi_span_ft: float,
    tip_chord_ft: float,
    left_ei_lbf_ft2: Sequence[float],
    right_ei_lbf_ft2: Sequence[float],
) -> OneWayAeroStructuralSolution:
    """
    Couple the finite-wing aerodynamic solution to independent
    left/right static cantilever beam solvers.

    This is intentionally ONE-WAY coupling:

        aerodynamics -> structure

    Wing deformation does not yet feed back into the aerodynamic
    incidence distribution.

    That feedback will be introduced only after the static structural
    model and torsional stiffness model are validated.
    """

    left_load = extract_half_wing_load(
        lifting_line,
        side="left",
        qbar_psf=qbar_psf,
        semi_span_ft=semi_span_ft,
        tip_chord_ft=tip_chord_ft,
    )

    right_load = extract_half_wing_load(
        lifting_line,
        side="right",
        qbar_psf=qbar_psf,
        semi_span_ft=semi_span_ft,
        tip_chord_ft=tip_chord_ft,
    )

    if len(left_ei_lbf_ft2) != len(
        left_load.y_ft
    ):
        raise ValueError(
            "Left EI distribution must match left structural grid"
        )

    if len(right_ei_lbf_ft2) != len(
        right_load.y_ft
    ):
        raise ValueError(
            "Right EI distribution must match right structural grid"
        )

    left_bending = solve_cantilever_bending(
        left_load.y_ft,
        left_load.lift_lbf_per_ft,
        left_ei_lbf_ft2,
    )

    right_bending = solve_cantilever_bending(
        right_load.y_ft,
        right_load.lift_lbf_per_ft,
        right_ei_lbf_ft2,
    )

    return OneWayAeroStructuralSolution(
        left_load=left_load,
        right_load=right_load,
        left_bending=left_bending,
        right_bending=right_bending,
    )


# Mooney M20M wing geometry.
#
# Published/reference geometry:
#
#   root incidence: 2.5 deg
#   tip incidence:  1.0 deg
#   dihedral:       5.5 deg
#
# JSBSim's current metrics already specify 2.5 deg wing incidence,
# so structural/aeroelastic calculations should use RELATIVE twist
# from the wing root rather than adding absolute incidence again.

M20M_ROOT_INCIDENCE_RAD = math.radians(2.5)
M20M_TIP_INCIDENCE_RAD = math.radians(1.0)

M20M_TIP_TWIST_FROM_ROOT_RAD = (
    M20M_TIP_INCIDENCE_RAD
    - M20M_ROOT_INCIDENCE_RAD
)

M20M_DIHEDRAL_RAD = math.radians(5.5)


def mooney_m20m_geometric_twist_rad(
    signed_y_ft: float,
    semi_span_ft: float,
) -> float:
    """
    Return M20M geometric wing twist relative to the wing root.

    Convention:

        root = 0 rad
        tip  = -1.5 deg

    Negative twist therefore represents washout.

    The published endpoint incidences are currently interpolated
    linearly along the span.

    This linear distribution is an explicit modeling assumption,
    not a claim that Mooney manufactured the twist distribution as
    perfectly linear between every wing station.
    """

    values = (
        signed_y_ft,
        semi_span_ft,
    )

    if not all(
        math.isfinite(value)
        for value in values
    ):
        raise ValueError(
            "M20M twist inputs must be finite"
        )

    if semi_span_ft <= 0.0:
        raise ValueError(
            "Semi-span must be positive"
        )

    distance = abs(
        signed_y_ft
    )

    if distance > semi_span_ft + 1e-12:
        raise ValueError(
            "Spanwise position lies outside the wing"
        )

    fraction = min(
        distance / semi_span_ft,
        1.0,
    )

    return (
        M20M_TIP_TWIST_FROM_ROOT_RAD
        * fraction
    )


@dataclass(frozen=True)
class M20MWingFlowDistribution:
    theta_rad: tuple[float, ...]
    signed_y_ft: tuple[float, ...]
    chord_ft: tuple[float, ...]

    geometric_twist_rad: tuple[float, ...]
    aeroelastic_twist_rad: tuple[float, ...]
    roll_delta_alpha_rad: tuple[float, ...]

    effective_alpha_rad: tuple[float, ...]
    local_speed_fps: tuple[float, ...]


def make_m20m_wing_flow_distribution(
    *,
    planform: TrapezoidalPlanform,
    reference_alpha_rad: float,
    forward_speed_fps: float,
    roll_rate_rad_s: float,
    station_count: int,
    aeroelastic_twist_rad: Sequence[float] | None = None,
) -> M20MWingFlowDistribution:
    """
    Build the full-wing local-flow distribution for the M20M.

    The supplied reference alpha is assumed to already include the
    JSBSim wing-incidence convention used by the aircraft FDM.

    Therefore only RELATIVE M20M geometric twist is added here.

    This function combines:

        planform chord
        documented M20M geometric washout
        rigid-body roll-induced local flow
        optional aeroelastic twist

    It deliberately does NOT choose:

        airfoil lift-curve slopes
        zero-lift angles
        stall limits
        flap/aileron effectiveness

    Those are aerodynamic-model inputs and must come from defensible
    data rather than being invented here.
    """

    scalar_values = (
        reference_alpha_rad,
        forward_speed_fps,
        roll_rate_rad_s,
    )

    if not all(
        math.isfinite(value)
        for value in scalar_values
    ):
        raise ValueError(
            "M20M wing-flow inputs must be finite"
        )

    if forward_speed_fps <= 0.0:
        raise ValueError(
            "Forward airspeed must be positive"
        )

    theta, signed_y = make_lifting_line_collocation(
        planform.wingspan_ft,
        station_count,
    )

    if aeroelastic_twist_rad is None:
        elastic_twist = tuple(
            0.0
            for _ in theta
        )
    else:
        if len(aeroelastic_twist_rad) != len(theta):
            raise ValueError(
                "Aeroelastic twist distribution must match lifting-line stations"
            )

        elastic_twist = tuple(
            float(value)
            for value in aeroelastic_twist_rad
        )

        if not all(
            math.isfinite(value)
            for value in elastic_twist
        ):
            raise ValueError(
                "Aeroelastic twist values must be finite"
            )

    chord = tuple(
        linear_chord_ft(
            planform,
            abs(y),
        )
        for y in signed_y
    )

    geometric_twist = tuple(
        mooney_m20m_geometric_twist_rad(
            y,
            planform.semi_span_ft,
        )
        for y in signed_y
    )

    flows = tuple(
        local_section_flow(
            reference_alpha_rad=reference_alpha_rad,
            forward_speed_fps=forward_speed_fps,
            roll_rate_rad_s=roll_rate_rad_s,
            signed_y_ft=signed_y[i],
            geometric_twist_rad=geometric_twist[i],
            aeroelastic_twist_rad=elastic_twist[i],
        )
        for i in range(len(theta))
    )

    return M20MWingFlowDistribution(
        theta_rad=theta,
        signed_y_ft=signed_y,
        chord_ft=chord,
        geometric_twist_rad=geometric_twist,
        aeroelastic_twist_rad=elastic_twist,
        roll_delta_alpha_rad=tuple(
            flow.roll_delta_alpha_rad
            for flow in flows
        ),
        effective_alpha_rad=tuple(
            flow.effective_alpha_rad
            for flow in flows
        ),
        local_speed_fps=tuple(
            flow.local_speed_fps
            for flow in flows
        ),
    )


# M20M clean-section aerodynamic reference data.
#
# Airfoils:
#
#   root: NACA 63(2)-215
#   tip:  NACA 64(1)-412
#
# Reference condition:
#
#   Reynolds number = 9.0e6
#
# These values represent clean-section linear-region aerodynamic
# data. They are NOT yet Reynolds-varying and must not be interpreted
# as stall/nonlinear airfoil models.

M20M_AIRFOIL_REFERENCE_REYNOLDS = 9_000_000.0

M20M_ROOT_LIFT_SLOPE_PER_DEG = 0.120
M20M_TIP_LIFT_SLOPE_PER_DEG = 0.112

M20M_ROOT_LIFT_SLOPE_PER_RAD = (
    M20M_ROOT_LIFT_SLOPE_PER_DEG
    * 180.0
    / math.pi
)

M20M_TIP_LIFT_SLOPE_PER_RAD = (
    M20M_TIP_LIFT_SLOPE_PER_DEG
    * 180.0
    / math.pi
)

M20M_ROOT_ZERO_LIFT_ALPHA_RAD = math.radians(
    -1.2
)

M20M_TIP_ZERO_LIFT_ALPHA_RAD = math.radians(
    -2.8
)


@dataclass(frozen=True)
class M20MSectionLinearAerodynamics:
    signed_y_ft: float
    span_fraction: float

    lift_curve_slope_per_rad: float
    alpha_zero_lift_rad: float


@dataclass(frozen=True)
class M20MAirfoilDistribution:
    signed_y_ft: tuple[float, ...]

    lift_curve_slope_per_rad: tuple[float, ...]
    alpha_zero_lift_rad: tuple[float, ...]


def mooney_m20m_section_linear_aerodynamics(
    signed_y_ft: float,
    semi_span_ft: float,
) -> M20MSectionLinearAerodynamics:
    """
    Return the clean linear aerodynamic parameters for one M20M
    spanwise section.

    The known root and tip airfoils are:

        root: NACA 63(2)-215
        tip:  NACA 64(1)-412

    The reference data used here are for Re = 9e6.

    Since the aircraft documentation describes the airfoil as varying
    from the root section to the tip section, the aerodynamic
    parameters are linearly interpolated spanwise.

    That interpolation is an explicit reduced-order modeling
    assumption. It does not imply that the actual manufactured wing
    uses a mathematically linear family of intermediate airfoils.
    """

    if not math.isfinite(
        signed_y_ft
    ):
        raise ValueError(
            "Spanwise position must be finite"
        )

    if not math.isfinite(
        semi_span_ft
    ):
        raise ValueError(
            "Semi-span must be finite"
        )

    if semi_span_ft <= 0.0:
        raise ValueError(
            "Semi-span must be positive"
        )

    distance = abs(
        signed_y_ft
    )

    if distance > semi_span_ft + 1e-12:
        raise ValueError(
            "Spanwise position lies outside the wing"
        )

    span_fraction = min(
        distance / semi_span_ft,
        1.0,
    )

    lift_curve_slope = (
        M20M_ROOT_LIFT_SLOPE_PER_RAD
        + span_fraction
        * (
            M20M_TIP_LIFT_SLOPE_PER_RAD
            - M20M_ROOT_LIFT_SLOPE_PER_RAD
        )
    )

    alpha_zero_lift = (
        M20M_ROOT_ZERO_LIFT_ALPHA_RAD
        + span_fraction
        * (
            M20M_TIP_ZERO_LIFT_ALPHA_RAD
            - M20M_ROOT_ZERO_LIFT_ALPHA_RAD
        )
    )

    return M20MSectionLinearAerodynamics(
        signed_y_ft=signed_y_ft,
        span_fraction=span_fraction,
        lift_curve_slope_per_rad=lift_curve_slope,
        alpha_zero_lift_rad=alpha_zero_lift,
    )


def make_m20m_airfoil_distribution(
    signed_y_ft: Sequence[float],
    semi_span_ft: float,
) -> M20MAirfoilDistribution:
    """
    Build spanwise M20M linear airfoil parameters suitable for the
    lifting-line solver.
    """

    if len(signed_y_ft) < 3:
        raise ValueError(
            "At least three span stations are required"
        )

    sections = tuple(
        mooney_m20m_section_linear_aerodynamics(
            y,
            semi_span_ft,
        )
        for y in signed_y_ft
    )

    return M20MAirfoilDistribution(
        signed_y_ft=tuple(
            float(y)
            for y in signed_y_ft
        ),
        lift_curve_slope_per_rad=tuple(
            section.lift_curve_slope_per_rad
            for section in sections
        ),
        alpha_zero_lift_rad=tuple(
            section.alpha_zero_lift_rad
            for section in sections
        ),
    )


@dataclass(frozen=True)
class SectionFlowProperties:
    chord_ft: float
    speed_fps: float

    dynamic_pressure_psf: float
    reynolds_number: float


@dataclass(frozen=True)
class M20MLocalAeroStateDistribution:
    signed_y_ft: tuple[float, ...]
    chord_ft: tuple[float, ...]
    local_speed_fps: tuple[float, ...]

    dynamic_pressure_psf: tuple[float, ...]
    reynolds_number: tuple[float, ...]


def section_flow_properties(
    *,
    chord_ft: float,
    speed_fps: float,
    air_density_slug_ft3: float,
    dynamic_viscosity_slug_ft_s: float,
) -> SectionFlowProperties:
    """
    Calculate basic local aerodynamic flow properties.

    Dynamic pressure:

        q = 0.5 * rho * V^2

    Reynolds number:

        Re = rho * V * c / mu

    Units:

        chord_ft                    ft
        speed_fps                   ft/s
        air_density_slug_ft3        slug/ft^3
        dynamic_viscosity_slug_ft_s slug/(ft*s)

    giving:

        dynamic_pressure_psf        lbf/ft^2
        reynolds_number             dimensionless
    """

    values = (
        chord_ft,
        speed_fps,
        air_density_slug_ft3,
        dynamic_viscosity_slug_ft_s,
    )

    if not all(
        math.isfinite(value)
        for value in values
    ):
        raise ValueError(
            "Section flow inputs must all be finite"
        )

    if chord_ft <= 0.0:
        raise ValueError(
            "Chord must be positive"
        )

    if speed_fps < 0.0:
        raise ValueError(
            "Airspeed cannot be negative"
        )

    if air_density_slug_ft3 <= 0.0:
        raise ValueError(
            "Air density must be positive"
        )

    if dynamic_viscosity_slug_ft_s <= 0.0:
        raise ValueError(
            "Dynamic viscosity must be positive"
        )

    dynamic_pressure = (
        0.5
        * air_density_slug_ft3
        * speed_fps ** 2
    )

    reynolds_number = (
        air_density_slug_ft3
        * speed_fps
        * chord_ft
        / dynamic_viscosity_slug_ft_s
    )

    return SectionFlowProperties(
        chord_ft=chord_ft,
        speed_fps=speed_fps,
        dynamic_pressure_psf=dynamic_pressure,
        reynolds_number=reynolds_number,
    )


def make_m20m_local_aero_state_distribution(
    *,
    flow_distribution: M20MWingFlowDistribution,
    air_density_slug_ft3: float,
    dynamic_viscosity_slug_ft_s: float,
) -> M20MLocalAeroStateDistribution:
    """
    Calculate local qbar and Reynolds number for every M20M
    lifting-line station.

    This uses each station's actual local speed from the wing-flow
    model and its local spanwise chord.

    This function deliberately does not change the airfoil
    coefficients as a function of Reynolds number. It only computes
    the physical Reynolds state needed by a later validated
    Reynolds-dependent aerodynamic model.
    """

    count = len(
        flow_distribution.signed_y_ft
    )

    arrays = (
        flow_distribution.chord_ft,
        flow_distribution.local_speed_fps,
    )

    if any(
        len(values) != count
        for values in arrays
    ):
        raise ValueError(
            "M20M wing-flow arrays must have equal length"
        )

    states = tuple(
        section_flow_properties(
            chord_ft=flow_distribution.chord_ft[i],
            speed_fps=flow_distribution.local_speed_fps[i],
            air_density_slug_ft3=air_density_slug_ft3,
            dynamic_viscosity_slug_ft_s=dynamic_viscosity_slug_ft_s,
        )
        for i in range(count)
    )

    return M20MLocalAeroStateDistribution(
        signed_y_ft=flow_distribution.signed_y_ft,
        chord_ft=flow_distribution.chord_ft,
        local_speed_fps=flow_distribution.local_speed_fps,
        dynamic_pressure_psf=tuple(
            state.dynamic_pressure_psf
            for state in states
        ),
        reynolds_number=tuple(
            state.reynolds_number
            for state in states
        ),
    )


def extract_half_wing_local_q_load(
    lifting_line: LiftingLineSolution,
    local_aero_state: M20MLocalAeroStateDistribution,
    *,
    side: str,
    semi_span_ft: float,
    tip_chord_ft: float,
) -> HalfWingAerodynamicLoad:
    """
    Convert a full-wing lifting-line solution into a structural
    half-wing load using stationwise dynamic pressure.

        L'(y) = q(y) * c(y) * Cl(y)

    Unlike extract_half_wing_load(), this function does not assume
    one scalar qbar for the entire wing.

    The physical tip is appended with zero lift because Prandtl
    lifting-line enforces zero circulation at the tip.
    """

    if side not in (
        "left",
        "right",
    ):
        raise ValueError(
            "Wing side must be 'left' or 'right'"
        )

    if not math.isfinite(semi_span_ft):
        raise ValueError(
            "Semi-span must be finite"
        )

    if not math.isfinite(tip_chord_ft):
        raise ValueError(
            "Tip chord must be finite"
        )

    if semi_span_ft <= 0.0:
        raise ValueError(
            "Semi-span must be positive"
        )

    if tip_chord_ft <= 0.0:
        raise ValueError(
            "Tip chord must be positive"
        )

    count = len(
        lifting_line.signed_y_ft
    )

    arrays = (
        lifting_line.chord_ft,
        lifting_line.section_cl,
        local_aero_state.signed_y_ft,
        local_aero_state.chord_ft,
        local_aero_state.dynamic_pressure_psf,
    )

    if any(
        len(values) != count
        for values in arrays
    ):
        raise ValueError(
            "Lifting-line and local aerodynamic arrays must have equal length"
        )

    for i in range(count):
        if not math.isclose(
            lifting_line.signed_y_ft[i],
            local_aero_state.signed_y_ft[i],
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "Lifting-line and aerodynamic-state span grids do not match"
            )

        if not math.isclose(
            lifting_line.chord_ft[i],
            local_aero_state.chord_ft[i],
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "Lifting-line and aerodynamic-state chord distributions do not match"
            )

        qbar = (
            local_aero_state.dynamic_pressure_psf[i]
        )

        if not math.isfinite(qbar):
            raise ValueError(
                "Dynamic pressure values must be finite"
            )

        if qbar < 0.0:
            raise ValueError(
                "Dynamic pressure cannot be negative"
            )

    stations = []

    for i in range(count):
        signed_y = (
            lifting_line.signed_y_ft[i]
        )

        if side == "right":
            if signed_y < -1e-12:
                continue

            structural_y = signed_y

        else:
            if signed_y > 1e-12:
                continue

            structural_y = -signed_y

        chord = (
            lifting_line.chord_ft[i]
        )

        section_cl = (
            lifting_line.section_cl[i]
        )

        qbar = (
            local_aero_state.dynamic_pressure_psf[i]
        )

        lift_lbf_per_ft = (
            qbar
            * chord
            * section_cl
        )

        stations.append(
            (
                structural_y,
                chord,
                section_cl,
                lift_lbf_per_ft,
            )
        )

    stations.sort(
        key=lambda row: row[0]
    )

    if len(stations) < 2:
        raise ValueError(
            "Not enough half-wing aerodynamic stations"
        )

    if abs(stations[0][0]) > 1e-10:
        raise ValueError(
            "Aerodynamic grid must contain a centerline station"
        )

    if stations[-1][0] >= semi_span_ft:
        raise ValueError(
            "Aerodynamic collocation station lies at or beyond tip"
        )

    # Explicit physical tip:
    #
    #     Gamma = 0
    #     L'    = 0
    stations.append(
        (
            semi_span_ft,
            tip_chord_ft,
            0.0,
            0.0,
        )
    )

    y = tuple(
        row[0]
        for row in stations
    )

    chord = tuple(
        row[1]
        for row in stations
    )

    section_cl = tuple(
        row[2]
        for row in stations
    )

    lift = tuple(
        row[3]
        for row in stations
    )

    total_lift = integrate_distributed_load(
        y,
        lift,
    )

    return HalfWingAerodynamicLoad(
        side=side,
        y_ft=y,
        chord_ft=chord,
        section_cl=section_cl,
        lift_lbf_per_ft=lift,
        total_lift_lbf=total_lift,
    )


def solve_one_way_local_q_aero_structural_bending(
    *,
    lifting_line: LiftingLineSolution,
    local_aero_state: M20MLocalAeroStateDistribution,
    semi_span_ft: float,
    tip_chord_ft: float,
    left_ei_lbf_ft2: Sequence[float],
    right_ei_lbf_ft2: Sequence[float],
) -> OneWayAeroStructuralSolution:
    """
    Couple finite-wing aerodynamics to independent left/right beam
    solvers using stationwise dynamic pressure.

        alpha(y)
            -> Cl(y)
            -> q(y)
            -> L'(y)
            -> M(y)
            -> w(y)

    Coupling remains one-way. Structural deformation does not yet
    feed back into aerodynamic twist.
    """

    left_load = extract_half_wing_local_q_load(
        lifting_line,
        local_aero_state,
        side="left",
        semi_span_ft=semi_span_ft,
        tip_chord_ft=tip_chord_ft,
    )

    right_load = extract_half_wing_local_q_load(
        lifting_line,
        local_aero_state,
        side="right",
        semi_span_ft=semi_span_ft,
        tip_chord_ft=tip_chord_ft,
    )

    if len(left_ei_lbf_ft2) != len(
        left_load.y_ft
    ):
        raise ValueError(
            "Left EI distribution must match left structural grid"
        )

    if len(right_ei_lbf_ft2) != len(
        right_load.y_ft
    ):
        raise ValueError(
            "Right EI distribution must match right structural grid"
        )

    left_bending = solve_cantilever_bending(
        left_load.y_ft,
        left_load.lift_lbf_per_ft,
        left_ei_lbf_ft2,
    )

    right_bending = solve_cantilever_bending(
        right_load.y_ft,
        right_load.lift_lbf_per_ft,
        right_ei_lbf_ft2,
    )

    return OneWayAeroStructuralSolution(
        left_load=left_load,
        right_load=right_load,
        left_bending=left_bending,
        right_bending=right_bending,
    )


STANDARD_GRAVITY_FPS2 = 32.1740485564


@dataclass(frozen=True)
class DistributedInertialLoad:
    y_ft: tuple[float, ...]

    aerodynamic_load_lbf_per_ft: tuple[float, ...]

    structural_mass_slugs_per_ft: tuple[float, ...]
    fuel_mass_slugs_per_ft: tuple[float, ...]
    total_mass_slugs_per_ft: tuple[float, ...]

    inertial_load_lbf_per_ft: tuple[float, ...]
    net_load_lbf_per_ft: tuple[float, ...]

    @property
    def total_inertial_force_lbf(self) -> float:
        return integrate_distributed_load(
            self.y_ft,
            self.inertial_load_lbf_per_ft,
        )

    @property
    def total_net_force_lbf(self) -> float:
        return integrate_distributed_load(
            self.y_ft,
            self.net_load_lbf_per_ft,
        )


def pounds_mass_to_slugs(
    mass_lbm: float,
) -> float:
    """
    Convert pounds-mass to slugs.

        1 slug = 32.1740485564 lbm
    """

    if not math.isfinite(mass_lbm):
        raise ValueError(
            "Mass must be finite"
        )

    if mass_lbm < 0.0:
        raise ValueError(
            "Mass cannot be negative"
        )

    return (
        mass_lbm
        / STANDARD_GRAVITY_FPS2
    )


def normalize_mass_distribution(
    y_ft: Sequence[float],
    relative_shape: Sequence[float],
    total_mass_slugs: float,
) -> tuple[float, ...]:
    """
    Scale an arbitrary non-negative spanwise shape so its integral
    equals the requested total mass.

    This lets later aircraft-specific models describe wing structure
    and fuel using defensible shape functions without changing the
    inertial-load solver.

        integral m'(y) dy = total_mass
    """

    count = len(y_ft)

    if count < 2:
        raise ValueError(
            "At least two span stations are required"
        )

    if len(relative_shape) != count:
        raise ValueError(
            "Mass shape length must match span grid"
        )

    if not math.isfinite(total_mass_slugs):
        raise ValueError(
            "Total mass must be finite"
        )

    if total_mass_slugs < 0.0:
        raise ValueError(
            "Total mass cannot be negative"
        )

    if not all(
        math.isfinite(value)
        for value in relative_shape
    ):
        raise ValueError(
            "Mass shape values must be finite"
        )

    if any(
        value < 0.0
        for value in relative_shape
    ):
        raise ValueError(
            "Mass shape cannot be negative"
        )

    # Validate the grid and obtain the unnormalised shape integral.
    shape_integral = integrate_distributed_load(
        y_ft,
        relative_shape,
    )

    if total_mass_slugs == 0.0:
        return tuple(
            0.0
            for _ in y_ft
        )

    if shape_integral <= 0.0:
        raise ValueError(
            "Non-zero mass requires a positive mass-distribution shape"
        )

    scale = (
        total_mass_slugs
        / shape_integral
    )

    return tuple(
        scale * value
        for value in relative_shape
    )


def solve_distributed_inertial_load(
    *,
    y_ft: Sequence[float],
    aerodynamic_load_lbf_per_ft: Sequence[float],
    structural_mass_slugs_per_ft: Sequence[float],
    fuel_mass_slugs_per_ft: Sequence[float],
    normal_acceleration_fps2: float,
) -> DistributedInertialLoad:
    """
    Combine aerodynamic and distributed inertial loads.

    Sign convention:

        aerodynamic load > 0 : upward
        acceleration     > 0 : aircraft accelerating upward
        inertial load    < 0 : opposes that acceleration

    Therefore:

        p_inertial(y) = -m'(y) * a_z

        p_net(y) =
            p_aero(y)
            + p_inertial(y)

    This is the load that should be passed to the beam solver.

    No FlightGear/JSBSim acceleration property is bound here yet.
    The correct runtime property and sign convention must be verified
    separately before production integration.
    """

    count = len(y_ft)

    arrays = (
        aerodynamic_load_lbf_per_ft,
        structural_mass_slugs_per_ft,
        fuel_mass_slugs_per_ft,
    )

    if any(
        len(values) != count
        for values in arrays
    ):
        raise ValueError(
            "All distributed-load arrays must match the span grid"
        )

    if not math.isfinite(
        normal_acceleration_fps2
    ):
        raise ValueError(
            "Normal acceleration must be finite"
        )

    # This also validates increasing span stations.
    integrate_distributed_load(
        y_ft,
        tuple(
            0.0
            for _ in y_ft
        ),
    )

    if not all(
        math.isfinite(value)
        for values in arrays
        for value in values
    ):
        raise ValueError(
            "Distributed load and mass values must be finite"
        )

    if any(
        value < 0.0
        for value in structural_mass_slugs_per_ft
    ):
        raise ValueError(
            "Structural mass distribution cannot be negative"
        )

    if any(
        value < 0.0
        for value in fuel_mass_slugs_per_ft
    ):
        raise ValueError(
            "Fuel mass distribution cannot be negative"
        )

    total_mass = tuple(
        structural_mass_slugs_per_ft[i]
        + fuel_mass_slugs_per_ft[i]
        for i in range(count)
    )

    inertial_load = tuple(
        -total_mass[i]
        * normal_acceleration_fps2
        for i in range(count)
    )

    net_load = tuple(
        aerodynamic_load_lbf_per_ft[i]
        + inertial_load[i]
        for i in range(count)
    )

    return DistributedInertialLoad(
        y_ft=tuple(
            float(value)
            for value in y_ft
        ),
        aerodynamic_load_lbf_per_ft=tuple(
            float(value)
            for value in aerodynamic_load_lbf_per_ft
        ),
        structural_mass_slugs_per_ft=tuple(
            float(value)
            for value in structural_mass_slugs_per_ft
        ),
        fuel_mass_slugs_per_ft=tuple(
            float(value)
            for value in fuel_mass_slugs_per_ft
        ),
        total_mass_slugs_per_ft=total_mass,
        inertial_load_lbf_per_ft=inertial_load,
        net_load_lbf_per_ft=net_load,
    )


# Current production M20M fuel model:
#
#   44.5 US gal usable per wing
#   6.0 lb/US gal
#   267 lb per wing
#
# The SPANWISE SHAPE of the tank is deliberately not defined here yet.
# That requires defensible tank/rib geometry.

M20M_USABLE_FUEL_PER_WING_GAL = 44.5
M20M_FUEL_DENSITY_LB_PER_GAL = 6.0

M20M_FUEL_CAPACITY_PER_WING_LBM = (
    M20M_USABLE_FUEL_PER_WING_GAL
    * M20M_FUEL_DENSITY_LB_PER_GAL
)


@dataclass(frozen=True)
class M20MWingFuelDistribution:
    y_ft: tuple[float, ...]

    left_fuel_lbm: float
    right_fuel_lbm: float

    left_fill_fraction: float
    right_fill_fraction: float

    left_mass_slugs_per_ft: tuple[float, ...]
    right_mass_slugs_per_ft: tuple[float, ...]

    @property
    def total_fuel_lbm(self) -> float:
        return (
            self.left_fuel_lbm
            + self.right_fuel_lbm
        )


def make_m20m_wing_fuel_distribution(
    *,
    y_ft: Sequence[float],
    relative_tank_shape: Sequence[float],
    left_fuel_lbm: float,
    right_fuel_lbm: float,
) -> M20MWingFuelDistribution:
    """
    Build independent left/right M20M distributed fuel masses.

    `relative_tank_shape` describes only the SHAPE of the fuel
    distribution on one root-to-tip half-wing grid.

    It is deliberately supplied by the caller because exact M20M
    tank span boundaries have not yet been established strongly
    enough to hard-code them.

    The shape is normalized independently for each wing so that:

        integral m'_fuel,L dy = left fuel mass

        integral m'_fuel,R dy = right fuel mass

    This allows FlightGear/JSBSim tank quantities to eventually feed
    the structural model directly without changing the structural
    distribution architecture.
    """

    count = len(y_ft)

    if count < 2:
        raise ValueError(
            "At least two fuel-distribution stations are required"
        )

    if len(relative_tank_shape) != count:
        raise ValueError(
            "Fuel shape length must match span grid"
        )

    quantities = (
        left_fuel_lbm,
        right_fuel_lbm,
    )

    if not all(
        math.isfinite(value)
        for value in quantities
    ):
        raise ValueError(
            "Fuel quantities must be finite"
        )

    if any(
        value < 0.0
        for value in quantities
    ):
        raise ValueError(
            "Fuel quantity cannot be negative"
        )

    if any(
        value > M20M_FUEL_CAPACITY_PER_WING_LBM + 1e-12
        for value in quantities
    ):
        raise ValueError(
            "Fuel quantity exceeds M20M per-wing capacity"
        )

    # Validate span grid and shape before normalization.
    integrate_distributed_load(
        y_ft,
        tuple(
            0.0
            for _ in y_ft
        ),
    )

    if not all(
        math.isfinite(value)
        for value in relative_tank_shape
    ):
        raise ValueError(
            "Fuel-distribution shape must be finite"
        )

    if any(
        value < 0.0
        for value in relative_tank_shape
    ):
        raise ValueError(
            "Fuel-distribution shape cannot be negative"
        )

    left_mass_slugs = pounds_mass_to_slugs(
        left_fuel_lbm
    )

    right_mass_slugs = pounds_mass_to_slugs(
        right_fuel_lbm
    )

    left_distribution = normalize_mass_distribution(
        y_ft,
        relative_tank_shape,
        left_mass_slugs,
    )

    right_distribution = normalize_mass_distribution(
        y_ft,
        relative_tank_shape,
        right_mass_slugs,
    )

    return M20MWingFuelDistribution(
        y_ft=tuple(
            float(value)
            for value in y_ft
        ),
        left_fuel_lbm=float(
            left_fuel_lbm
        ),
        right_fuel_lbm=float(
            right_fuel_lbm
        ),
        left_fill_fraction=(
            left_fuel_lbm
            / M20M_FUEL_CAPACITY_PER_WING_LBM
        ),
        right_fill_fraction=(
            right_fuel_lbm
            / M20M_FUEL_CAPACITY_PER_WING_LBM
        ),
        left_mass_slugs_per_ft=left_distribution,
        right_mass_slugs_per_ft=right_distribution,
    )


@dataclass(frozen=True)
class FuelCoupledAeroStructuralSolution:
    left_load: DistributedInertialLoad
    right_load: DistributedInertialLoad

    left_bending: BendingSolution
    right_bending: BendingSolution


def solve_fuel_coupled_aero_structural_bending(
    *,
    aerodynamic_solution: OneWayAeroStructuralSolution,
    left_structural_mass_slugs_per_ft: Sequence[float],
    right_structural_mass_slugs_per_ft: Sequence[float],
    fuel_distribution: M20MWingFuelDistribution,
    normal_acceleration_fps2: float,
    left_ei_lbf_ft2: Sequence[float],
    right_ei_lbf_ft2: Sequence[float],
) -> FuelCoupledAeroStructuralSolution:
    """
    Apply structural and fuel inertia to left/right aerodynamic loads.

    Pipeline:

        aerodynamic lift
            +
        distributed structural inertia
            +
        distributed fuel inertia
            ->
        net beam load
            ->
        bending response

    This is still static, one-way structural coupling.
    """

    left_aero = aerodynamic_solution.left_load
    right_aero = aerodynamic_solution.right_load

    if len(fuel_distribution.y_ft) != len(left_aero.y_ft):
        raise ValueError(
            "Fuel grid must match structural half-wing grid"
        )

    if not all(
        math.isclose(
            fuel_y,
            structural_y,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        for fuel_y, structural_y in zip(
            fuel_distribution.y_ft,
            left_aero.y_ft,
        )
    ):
        raise ValueError(
            "Fuel grid must match structural half-wing grid"
        )

    if len(right_aero.y_ft) != len(left_aero.y_ft):
        raise ValueError(
            "Left and right structural grids must match"
        )

    if not all(
        math.isclose(
            left_y,
            right_y,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        for left_y, right_y in zip(
            left_aero.y_ft,
            right_aero.y_ft,
        )
    ):
        raise ValueError(
            "Left and right structural grids must match"
        )

    count = len(
        left_aero.y_ft
    )

    arrays = (
        left_structural_mass_slugs_per_ft,
        right_structural_mass_slugs_per_ft,
        left_ei_lbf_ft2,
        right_ei_lbf_ft2,
    )

    if any(
        len(values) != count
        for values in arrays
    ):
        raise ValueError(
            "Structural mass and EI arrays must match half-wing grid"
        )

    left_net = solve_distributed_inertial_load(
        y_ft=left_aero.y_ft,
        aerodynamic_load_lbf_per_ft=left_aero.lift_lbf_per_ft,
        structural_mass_slugs_per_ft=left_structural_mass_slugs_per_ft,
        fuel_mass_slugs_per_ft=fuel_distribution.left_mass_slugs_per_ft,
        normal_acceleration_fps2=normal_acceleration_fps2,
    )

    right_net = solve_distributed_inertial_load(
        y_ft=right_aero.y_ft,
        aerodynamic_load_lbf_per_ft=right_aero.lift_lbf_per_ft,
        structural_mass_slugs_per_ft=right_structural_mass_slugs_per_ft,
        fuel_mass_slugs_per_ft=fuel_distribution.right_mass_slugs_per_ft,
        normal_acceleration_fps2=normal_acceleration_fps2,
    )

    left_bending = solve_cantilever_bending(
        left_net.y_ft,
        left_net.net_load_lbf_per_ft,
        left_ei_lbf_ft2,
    )

    right_bending = solve_cantilever_bending(
        right_net.y_ft,
        right_net.net_load_lbf_per_ft,
        right_ei_lbf_ft2,
    )

    return FuelCoupledAeroStructuralSolution(
        left_load=left_net,
        right_load=right_net,
        left_bending=left_bending,
        right_bending=right_bending,
    )


# M20M integral fuel-cell span limits.
#
# M20M Service & Maintenance Manual, Chapter 57:
#
#   fuel cells start at WS 24.50
#   fuel cells continue through WS 88.75
#
# Wing stations are measured in inches from aircraft centerline.

M20M_FUEL_TANK_INBOARD_WS_IN = 24.50
M20M_FUEL_TANK_OUTBOARD_WS_IN = 88.75

M20M_FUEL_TANK_INBOARD_Y_FT = (
    M20M_FUEL_TANK_INBOARD_WS_IN
    / 12.0
)

M20M_FUEL_TANK_OUTBOARD_Y_FT = (
    M20M_FUEL_TANK_OUTBOARD_WS_IN
    / 12.0
)


# Root and tip thickness ratios follow from the documented
# M20M root/tip airfoil sections:
#
#   root section: 15 percent thick
#   tip section:  12 percent thick

M20M_ROOT_THICKNESS_RATIO = 0.15
M20M_TIP_THICKNESS_RATIO = 0.12


def mooney_m20m_thickness_ratio(
    y_ft: float,
    semi_span_ft: float,
) -> float:
    """
    Return reduced-order M20M airfoil thickness ratio at one
    root-to-tip half-wing station.

    Root:
        t/c = 0.15

    Tip:
        t/c = 0.12

    Linear interpolation is an explicit reduced-order assumption.
    """

    if not math.isfinite(y_ft):
        raise ValueError(
            "Spanwise position must be finite"
        )

    if not math.isfinite(semi_span_ft):
        raise ValueError(
            "Semi-span must be finite"
        )

    if semi_span_ft <= 0.0:
        raise ValueError(
            "Semi-span must be positive"
        )

    if (
        y_ft < -1e-12
        or y_ft > semi_span_ft + 1e-12
    ):
        raise ValueError(
            "Spanwise position lies outside half-wing"
        )

    y = min(
        max(y_ft, 0.0),
        semi_span_ft,
    )

    fraction = (
        y
        / semi_span_ft
    )

    return (
        M20M_ROOT_THICKNESS_RATIO
        + fraction
        * (
            M20M_TIP_THICKNESS_RATIO
            - M20M_ROOT_THICKNESS_RATIO
        )
    )


def mooney_m20m_fuel_tank_shape_value(
    *,
    y_ft: float,
    planform: TrapezoidalPlanform,
) -> float:
    """
    Return relative M20M fuel volume per unit span.

    Actual M20M tank boundaries are used:

        WS 24.50 through WS 88.75

    Within the tank, the reduced-order volume proxy is:

        shape(y) = c(y)^2 * (t/c)(y)

    Airfoil cross-sectional area scales approximately with chord
    squared and thickness ratio. Since the wet-wing tank occupies
    a forward portion of the local wing section, this provides a
    physically motivated distribution shape.

    The result is RELATIVE only. make_m20m_wing_fuel_distribution()
    subsequently normalizes it to the exact requested fuel mass.

    This is not a claim that actual tank volume follows this formula
    exactly.
    """

    if not math.isfinite(y_ft):
        raise ValueError(
            "Spanwise position must be finite"
        )

    if (
        y_ft < -1e-12
        or y_ft > planform.semi_span_ft + 1e-12
    ):
        raise ValueError(
            "Spanwise position lies outside half-wing"
        )

    y = min(
        max(y_ft, 0.0),
        planform.semi_span_ft,
    )

    if (
        y < M20M_FUEL_TANK_INBOARD_Y_FT
        or y > M20M_FUEL_TANK_OUTBOARD_Y_FT
    ):
        return 0.0

    chord = linear_chord_ft(
        planform,
        y,
    )

    thickness_ratio = mooney_m20m_thickness_ratio(
        y,
        planform.semi_span_ft,
    )

    return (
        chord ** 2
        * thickness_ratio
    )


def make_m20m_fuel_tank_shape(
    *,
    y_ft: Sequence[float],
    planform: TrapezoidalPlanform,
) -> tuple[float, ...]:
    """
    Build the actual M20M span-limited relative fuel-cell shape.
    """

    if len(y_ft) < 2:
        raise ValueError(
            "At least two span stations are required"
        )

    # Validate structural grid.
    integrate_distributed_load(
        y_ft,
        tuple(
            0.0
            for _ in y_ft
        ),
    )

    shape = tuple(
        mooney_m20m_fuel_tank_shape_value(
            y_ft=y,
            planform=planform,
        )
        for y in y_ft
    )

    if not any(
        value > 0.0
        for value in shape
    ):
        raise ValueError(
            "Span grid does not resolve the M20M fuel-cell region"
        )

    return shape


def make_m20m_geometry_fuel_distribution(
    *,
    y_ft: Sequence[float],
    planform: TrapezoidalPlanform,
    left_fuel_lbm: float,
    right_fuel_lbm: float,
) -> M20MWingFuelDistribution:
    """
    Build left/right M20M fuel distributions using the documented
    fuel-cell span boundaries and reduced-order local volume shape.
    """

    shape = make_m20m_fuel_tank_shape(
        y_ft=y_ft,
        planform=planform,
    )

    return make_m20m_wing_fuel_distribution(
        y_ft=y_ft,
        relative_tank_shape=shape,
        left_fuel_lbm=left_fuel_lbm,
        right_fuel_lbm=right_fuel_lbm,
    )


def distributed_centroid_ft(
    y_ft: Sequence[float],
    distribution_per_ft: Sequence[float],
) -> float:
    """
    Return spanwise centroid of a non-negative distributed quantity.

        y_bar = integral(y p(y) dy) / integral(p(y) dy)
    """

    if len(y_ft) != len(
        distribution_per_ft
    ):
        raise ValueError(
            "Distribution length must match span grid"
        )

    total = integrate_distributed_load(
        y_ft,
        distribution_per_ft,
    )

    if total <= 0.0:
        raise ValueError(
            "Centroid requires a positive integrated distribution"
        )

    first_moment = integrate_distributed_load(
        y_ft,
        tuple(
            y * value
            for y, value in zip(
                y_ft,
                distribution_per_ft,
            )
        ),
    )

    return (
        first_moment
        / total
    )


@dataclass(frozen=True)
class DistributedMassComponent:
    """
    One independently normalized structural mass component.

    Examples later may include:

        main spar
        skins
        ribs
        stringers
        rear/stub spar
        landing-gear support structure
        aileron/flap structure

    No M20M component weights are assumed here.
    """

    name: str
    y_ft: tuple[float, ...]
    total_mass_lbm: float
    mass_slugs_per_ft: tuple[float, ...]


@dataclass(frozen=True)
class WingStructuralMassDistribution:
    """
    Combined structural mass distribution for one half-wing.
    """

    y_ft: tuple[float, ...]
    components: tuple[DistributedMassComponent, ...]
    mass_slugs_per_ft: tuple[float, ...]

    @property
    def total_mass_slugs(self) -> float:
        return integrate_distributed_load(
            self.y_ft,
            self.mass_slugs_per_ft,
        )

    @property
    def total_mass_lbm(self) -> float:
        return (
            self.total_mass_slugs
            * STANDARD_GRAVITY_FPS2
        )


def make_distributed_mass_component(
    *,
    name: str,
    y_ft: Sequence[float],
    relative_shape: Sequence[float],
    total_mass_lbm: float,
) -> DistributedMassComponent:
    """
    Normalize one relative structural shape to an exact component mass.

        integral m'(y) dy = component mass

    `relative_shape` is dimensionless and defines only the spanwise
    distribution shape.
    """

    if not isinstance(name, str):
        raise ValueError(
            "Component name must be a string"
        )

    if not name.strip():
        raise ValueError(
            "Component name cannot be empty"
        )

    if not math.isfinite(total_mass_lbm):
        raise ValueError(
            "Component mass must be finite"
        )

    if total_mass_lbm < 0.0:
        raise ValueError(
            "Component mass cannot be negative"
        )

    mass_slugs = pounds_mass_to_slugs(
        total_mass_lbm
    )

    distribution = normalize_mass_distribution(
        y_ft,
        relative_shape,
        mass_slugs,
    )

    return DistributedMassComponent(
        name=name,
        y_ft=tuple(
            float(value)
            for value in y_ft
        ),
        total_mass_lbm=float(
            total_mass_lbm
        ),
        mass_slugs_per_ft=distribution,
    )


def combine_distributed_mass_components(
    *,
    y_ft: Sequence[float],
    components: Sequence[DistributedMassComponent],
) -> WingStructuralMassDistribution:
    """
    Sum independent structural mass components onto one common
    root-to-tip half-wing grid.
    """

    if len(y_ft) < 2:
        raise ValueError(
            "At least two structural stations are required"
        )

    reference_grid = tuple(
        float(value)
        for value in y_ft
    )

    # Validate monotonic grid.
    integrate_distributed_load(
        reference_grid,
        tuple(
            0.0
            for _ in reference_grid
        ),
    )

    total = [
        0.0
        for _ in reference_grid
    ]

    for component in components:
        if len(component.y_ft) != len(
            reference_grid
        ):
            raise ValueError(
                "Structural component grid length mismatch"
            )

        if not all(
            math.isclose(
                component_y,
                reference_y,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
            for component_y, reference_y in zip(
                component.y_ft,
                reference_grid,
            )
        ):
            raise ValueError(
                "Structural component grid mismatch"
            )

        if len(
            component.mass_slugs_per_ft
        ) != len(reference_grid):
            raise ValueError(
                "Structural component distribution length mismatch"
            )

        for index, value in enumerate(
            component.mass_slugs_per_ft
        ):
            if not math.isfinite(value):
                raise ValueError(
                    "Structural mass distribution must be finite"
                )

            if value < 0.0:
                raise ValueError(
                    "Structural mass distribution cannot be negative"
                )

            total[index] += value

    return WingStructuralMassDistribution(
        y_ft=reference_grid,
        components=tuple(
            components
        ),
        mass_slugs_per_ft=tuple(
            total
        ),
    )


def make_chord_proportional_mass_shape(
    *,
    y_ft: Sequence[float],
    planform: TrapezoidalPlanform,
) -> tuple[float, ...]:
    """
    Reduced-order area-per-span proxy:

        shape(y) proportional to chord(y)

    This helper does NOT claim the complete M20M wing mass follows
    chord. It is intended for components whose material area per unit
    span is reasonably approximated by local chord, such as an early
    skin-area proxy.

    Component-specific M20M shapes should replace this helper whenever
    better geometry is available.
    """

    values = []

    for y in y_ft:
        if not math.isfinite(y):
            raise ValueError(
                "Spanwise position must be finite"
            )

        if (
            y < -1e-12
            or y > planform.semi_span_ft + 1e-12
        ):
            raise ValueError(
                "Spanwise position lies outside half-wing"
            )

        values.append(
            linear_chord_ft(
                planform,
                min(
                    max(y, 0.0),
                    planform.semi_span_ft,
                ),
            )
        )

    return tuple(
        values
    )


def solve_structural_mass_coupled_aero_bending(
    *,
    aerodynamic_solution: OneWayAeroStructuralSolution,
    left_structural_mass: WingStructuralMassDistribution,
    right_structural_mass: WingStructuralMassDistribution,
    fuel_distribution: M20MWingFuelDistribution,
    normal_acceleration_fps2: float,
    left_ei_lbf_ft2: Sequence[float],
    right_ei_lbf_ft2: Sequence[float],
) -> FuelCoupledAeroStructuralSolution:
    """
    Couple complete left/right structural mass distributions into
    the existing aerodynamic + fuel + inertia beam solution.

    This function is deliberately a thin integration boundary.

    It does not duplicate inertial-load or beam equations. Instead:

        structural components
            ->
        combined structural mass distribution
            ->
        solve_fuel_coupled_aero_structural_bending()

    This keeps component bookkeeping separate from structural physics.
    """

    left_aero = aerodynamic_solution.left_load
    right_aero = aerodynamic_solution.right_load

    if len(
        left_structural_mass.y_ft
    ) != len(left_aero.y_ft):
        raise ValueError(
            "Left structural mass grid must match aerodynamic grid"
        )

    if len(
        right_structural_mass.y_ft
    ) != len(right_aero.y_ft):
        raise ValueError(
            "Right structural mass grid must match aerodynamic grid"
        )

    if not all(
        math.isclose(
            structural_y,
            aerodynamic_y,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        for structural_y, aerodynamic_y in zip(
            left_structural_mass.y_ft,
            left_aero.y_ft,
        )
    ):
        raise ValueError(
            "Left structural mass grid must match aerodynamic grid"
        )

    if not all(
        math.isclose(
            structural_y,
            aerodynamic_y,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        for structural_y, aerodynamic_y in zip(
            right_structural_mass.y_ft,
            right_aero.y_ft,
        )
    ):
        raise ValueError(
            "Right structural mass grid must match aerodynamic grid"
        )

    return solve_fuel_coupled_aero_structural_bending(
        aerodynamic_solution=aerodynamic_solution,
        left_structural_mass_slugs_per_ft=(
            left_structural_mass.mass_slugs_per_ft
        ),
        right_structural_mass_slugs_per_ft=(
            right_structural_mass.mass_slugs_per_ft
        ),
        fuel_distribution=fuel_distribution,
        normal_acceleration_fps2=normal_acceleration_fps2,
        left_ei_lbf_ft2=left_ei_lbf_ft2,
        right_ei_lbf_ft2=right_ei_lbf_ft2,
    )


@dataclass(frozen=True)
class SpanwiseBendingStiffness:
    """
    Spanwise vertical bending stiffness for one half-wing.

    Section properties are retained separately so later M20M-specific
    geometry can be audited instead of hiding everything inside EI.
    """

    y_ft: tuple[float, ...]
    elastic_modulus_psi: tuple[float, ...]
    second_moment_in4: tuple[float, ...]
    ei_lbf_ft2: tuple[float, ...]

    @property
    def root_ei_lbf_ft2(self) -> float:
        return self.ei_lbf_ft2[0]

    @property
    def tip_ei_lbf_ft2(self) -> float:
        return self.ei_lbf_ft2[-1]


def bending_stiffness_lbf_ft2(
    *,
    elastic_modulus_psi: float,
    second_moment_in4: float,
) -> float:
    """
    Convert material modulus and section second moment into beam
    bending stiffness.

        EI = E * I

    Input units:

        E : lbf / in^2
        I : in^4

    E * I therefore has units lbf * in^2.

    Convert square inches to square feet:

        1 ft^2 = 144 in^2

    therefore:

        EI[lbf ft^2] = E[psi] * I[in^4] / 144
    """

    if not math.isfinite(
        elastic_modulus_psi
    ):
        raise ValueError(
            "Elastic modulus must be finite"
        )

    if not math.isfinite(
        second_moment_in4
    ):
        raise ValueError(
            "Second moment of area must be finite"
        )

    if elastic_modulus_psi <= 0.0:
        raise ValueError(
            "Elastic modulus must be positive"
        )

    if second_moment_in4 <= 0.0:
        raise ValueError(
            "Second moment of area must be positive"
        )

    return (
        elastic_modulus_psi
        * second_moment_in4
        / 144.0
    )


def make_spanwise_bending_stiffness(
    *,
    y_ft: Sequence[float],
    elastic_modulus_psi: Sequence[float],
    second_moment_in4: Sequence[float],
) -> SpanwiseBendingStiffness:
    """
    Calculate EI independently at every half-wing station.

    No assumption is made that material modulus or cross-sectional
    inertia is constant along the span.
    """

    count = len(y_ft)

    if count < 2:
        raise ValueError(
            "At least two stiffness stations are required"
        )

    if len(elastic_modulus_psi) != count:
        raise ValueError(
            "Elastic-modulus distribution must match span grid"
        )

    if len(second_moment_in4) != count:
        raise ValueError(
            "Second-moment distribution must match span grid"
        )

    # Validate the structural span grid.
    integrate_distributed_load(
        y_ft,
        tuple(
            0.0
            for _ in y_ft
        ),
    )

    ei = tuple(
        bending_stiffness_lbf_ft2(
            elastic_modulus_psi=e,
            second_moment_in4=i,
        )
        for e, i in zip(
            elastic_modulus_psi,
            second_moment_in4,
        )
    )

    return SpanwiseBendingStiffness(
        y_ft=tuple(
            float(value)
            for value in y_ft
        ),
        elastic_modulus_psi=tuple(
            float(value)
            for value in elastic_modulus_psi
        ),
        second_moment_in4=tuple(
            float(value)
            for value in second_moment_in4
        ),
        ei_lbf_ft2=ei,
    )


def make_piecewise_linear_ei_distribution(
    *,
    y_ft: Sequence[float],
    anchor_y_ft: Sequence[float],
    anchor_ei_lbf_ft2: Sequence[float],
) -> tuple[float, ...]:
    """
    Interpolate a spanwise EI distribution from structural anchors.

    This is intended for later M20M structural breakpoints such as
    changes in spar construction.

    The anchors are direct structural inputs. This function does not
    estimate or invent their values.
    """

    if len(y_ft) < 2:
        raise ValueError(
            "At least two output stations are required"
        )

    if len(anchor_y_ft) < 2:
        raise ValueError(
            "At least two EI anchors are required"
        )

    if len(anchor_y_ft) != len(
        anchor_ei_lbf_ft2
    ):
        raise ValueError(
            "EI anchor arrays must have equal length"
        )

    # Validate output span grid.
    integrate_distributed_load(
        y_ft,
        tuple(
            0.0
            for _ in y_ft
        ),
    )

    anchors_y = tuple(
        float(value)
        for value in anchor_y_ft
    )

    anchors_ei = tuple(
        float(value)
        for value in anchor_ei_lbf_ft2
    )

    if not all(
        math.isfinite(value)
        for value in anchors_y
    ):
        raise ValueError(
            "EI anchor positions must be finite"
        )

    if not all(
        math.isfinite(value)
        for value in anchors_ei
    ):
        raise ValueError(
            "EI anchor values must be finite"
        )

    if any(
        value <= 0.0
        for value in anchors_ei
    ):
        raise ValueError(
            "EI anchor values must be positive"
        )

    for index in range(
        len(anchors_y) - 1
    ):
        if (
            anchors_y[index + 1]
            <= anchors_y[index]
        ):
            raise ValueError(
                "EI anchor positions must be strictly increasing"
            )

    first_anchor = anchors_y[0]
    last_anchor = anchors_y[-1]

    result = []

    for raw_y in y_ft:
        y = float(raw_y)

        if not math.isfinite(y):
            raise ValueError(
                "Spanwise position must be finite"
            )

        if (
            y < first_anchor - 1e-12
            or y > last_anchor + 1e-12
        ):
            raise ValueError(
                "Output span grid lies outside EI anchor domain"
            )

        if math.isclose(
            y,
            first_anchor,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            result.append(
                anchors_ei[0]
            )
            continue

        if math.isclose(
            y,
            last_anchor,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            result.append(
                anchors_ei[-1]
            )
            continue

        found_interval = False

        for index in range(
            len(anchors_y) - 1
        ):
            y0 = anchors_y[index]
            y1 = anchors_y[index + 1]

            if (
                y0 <= y <= y1
            ):
                fraction = (
                    (y - y0)
                    / (y1 - y0)
                )

                ei = (
                    anchors_ei[index]
                    + fraction
                    * (
                        anchors_ei[index + 1]
                        - anchors_ei[index]
                    )
                )

                result.append(
                    ei
                )

                found_interval = True
                break

        if not found_interval:
            raise ValueError(
                "Could not interpolate EI at span station"
            )

    return tuple(
        result
    )
