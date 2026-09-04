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
