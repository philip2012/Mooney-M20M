#!/usr/bin/env python3

import math
import unittest

from wing_structure import (
    derive_trapezoidal_planform,
    integrate_distributed_load,
    lifting_line_series,
    linear_chord_ft,
    local_section_flow,
    make_lifting_line_collocation,
    make_trapezoidal_strips,
    make_uniform_span_grid,
    sectional_lift_lbf_per_ft,
    solve_cantilever_bending,
    solve_lifting_line,
    extract_half_wing_load,
    solve_one_way_aero_structural_bending,
    M20M_DIHEDRAL_RAD,
    M20M_ROOT_INCIDENCE_RAD,
    M20M_TIP_INCIDENCE_RAD,
    M20M_TIP_TWIST_FROM_ROOT_RAD,
    mooney_m20m_geometric_twist_rad,
    make_m20m_wing_flow_distribution,
    M20M_ROOT_LIFT_SLOPE_PER_RAD,
    M20M_ROOT_ZERO_LIFT_ALPHA_RAD,
    M20M_TIP_LIFT_SLOPE_PER_RAD,
    M20M_TIP_ZERO_LIFT_ALPHA_RAD,
    M20MLocalAeroStateDistribution,
    make_m20m_airfoil_distribution,
    mooney_m20m_section_linear_aerodynamics,
    make_m20m_local_aero_state_distribution,
    section_flow_properties,
    extract_half_wing_local_q_load,
    solve_one_way_local_q_aero_structural_bending,
    STANDARD_GRAVITY_FPS2,
    normalize_mass_distribution,
    pounds_mass_to_slugs,
    solve_distributed_inertial_load,
    M20M_FUEL_CAPACITY_PER_WING_LBM,
    M20M_FUEL_DENSITY_LB_PER_GAL,
    M20M_USABLE_FUEL_PER_WING_GAL,
    make_m20m_wing_fuel_distribution,
)

MOONEY_WING_AREA_SQFT = 174.786
MOONEY_WINGSPAN_FT = 36.0833

# Current FDM metrics notes:
#   Cr / Ct = 2.271
#
# Therefore:
#   Ct / Cr ~= 1 / 2.271
MOONEY_TAPER_RATIO = 1.0 / 2.271


class TestTrapezoidalPlanform(unittest.TestCase):
    def test_mooney_planform_reconstructs_area(self):
        planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        reconstructed_area = (
            planform.wingspan_ft
            * (
                planform.root_chord_ft
                + planform.tip_chord_ft
            )
            / 2.0
        )

        self.assertTrue(
            math.isclose(
                reconstructed_area,
                MOONEY_WING_AREA_SQFT,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_linear_chord_hits_root_and_tip(self):
        planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        self.assertTrue(
            math.isclose(
                linear_chord_ft(
                    planform,
                    0.0,
                ),
                planform.root_chord_ft,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

        self.assertTrue(
            math.isclose(
                linear_chord_ft(
                    planform,
                    planform.semi_span_ft,
                ),
                planform.tip_chord_ft,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )


class TestCantileverBending(unittest.TestCase):
    def setUp(self):
        self.length_ft = 10.0
        self.q_lbf_per_ft = 40.0
        self.ei_lbf_ft2 = 2_000_000.0

        # Dense grid so the numerical integration can be checked
        # tightly against the closed-form beam equations.
        self.y = make_uniform_span_grid(
            self.length_ft,
            2001,
        )

        self.load = tuple(
            self.q_lbf_per_ft
            for _ in self.y
        )

        self.ei = tuple(
            self.ei_lbf_ft2
            for _ in self.y
        )

    def test_zero_load_produces_zero_response(self):
        zero_load = tuple(
            0.0
            for _ in self.y
        )

        result = solve_cantilever_bending(
            self.y,
            zero_load,
            self.ei,
        )

        for values in (
            result.shear_lbf,
            result.moment_lbf_ft,
            result.curvature_per_ft,
            result.slope_rad,
            result.deflection_ft,
        ):
            self.assertTrue(
                all(
                    abs(value) < 1e-12
                    for value in values
                )
            )

    def test_uniform_load_root_shear(self):
        result = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        expected = (
            self.q_lbf_per_ft
            * self.length_ft
        )

        self.assertTrue(
            math.isclose(
                result.root_shear_lbf,
                expected,
                rel_tol=1e-9,
                abs_tol=1e-9,
            )
        )

    def test_uniform_load_root_moment(self):
        result = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        # Uniformly distributed load on a cantilever:
        #
        #   M_root = q L^2 / 2
        expected = (
            self.q_lbf_per_ft
            * self.length_ft ** 2
            / 2.0
        )

        self.assertTrue(
            math.isclose(
                result.root_moment_lbf_ft,
                expected,
                rel_tol=1e-8,
                abs_tol=1e-8,
            )
        )

    def test_uniform_load_tip_slope(self):
        result = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        # Euler-Bernoulli cantilever with uniform load:
        #
        #   theta_tip = q L^3 / (6 EI)
        expected = (
            self.q_lbf_per_ft
            * self.length_ft ** 3
            / (
                6.0
                * self.ei_lbf_ft2
            )
        )

        self.assertTrue(
            math.isclose(
                result.tip_slope_rad,
                expected,
                rel_tol=1e-6,
                abs_tol=1e-12,
            )
        )

    def test_uniform_load_tip_deflection(self):
        result = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        # Euler-Bernoulli cantilever with uniform load:
        #
        #   w_tip = q L^4 / (8 EI)
        expected = (
            self.q_lbf_per_ft
            * self.length_ft ** 4
            / (
                8.0
                * self.ei_lbf_ft2
            )
        )

        self.assertTrue(
            math.isclose(
                result.tip_deflection_ft,
                expected,
                rel_tol=1e-6,
                abs_tol=1e-12,
            )
        )

    def test_double_load_doubles_deflection(self):
        baseline = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        doubled_load = tuple(
            value * 2.0
            for value in self.load
        )

        doubled = solve_cantilever_bending(
            self.y,
            doubled_load,
            self.ei,
        )

        self.assertTrue(
            math.isclose(
                doubled.tip_deflection_ft,
                baseline.tip_deflection_ft * 2.0,
                rel_tol=1e-10,
                abs_tol=1e-12,
            )
        )

    def test_double_stiffness_halves_deflection(self):
        baseline = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        doubled_ei = tuple(
            value * 2.0
            for value in self.ei
        )

        stiffer = solve_cantilever_bending(
            self.y,
            self.load,
            doubled_ei,
        )

        self.assertTrue(
            math.isclose(
                stiffer.tip_deflection_ft,
                baseline.tip_deflection_ft / 2.0,
                rel_tol=1e-10,
                abs_tol=1e-12,
            )
        )

    def test_tip_boundary_conditions(self):
        result = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        self.assertAlmostEqual(
            result.shear_lbf[-1],
            0.0,
            places=12,
        )

        self.assertAlmostEqual(
            result.moment_lbf_ft[-1],
            0.0,
            places=12,
        )

    def test_root_boundary_conditions(self):
        result = solve_cantilever_bending(
            self.y,
            self.load,
            self.ei,
        )

        self.assertAlmostEqual(
            result.slope_rad[0],
            0.0,
            places=12,
        )

        self.assertAlmostEqual(
            result.deflection_ft[0],
            0.0,
            places=12,
        )

    def test_rejects_non_increasing_span(self):
        with self.assertRaises(ValueError):
            solve_cantilever_bending(
                (0.0, 1.0, 1.0),
                (1.0, 1.0, 1.0),
                (1000.0, 1000.0, 1000.0),
            )

    def test_rejects_non_positive_ei(self):
        with self.assertRaises(ValueError):
            solve_cantilever_bending(
                (0.0, 1.0, 2.0),
                (1.0, 1.0, 1.0),
                (1000.0, 0.0, 1000.0),
            )


class TestSpanwiseGeometry(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

    def test_strip_areas_reconstruct_half_wing(self):
        strips = make_trapezoidal_strips(
            self.planform,
            strip_count=16,
        )

        calculated = sum(
            strip.area_sqft
            for strip in strips
        )

        expected = (
            MOONEY_WING_AREA_SQFT
            / 2.0
        )

        self.assertTrue(
            math.isclose(
                calculated,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_strips_are_contiguous(self):
        strips = make_trapezoidal_strips(
            self.planform,
            strip_count=16,
        )

        self.assertAlmostEqual(
            strips[0].y_inner_ft,
            0.0,
            places=12,
        )

        self.assertAlmostEqual(
            strips[-1].y_outer_ft,
            self.planform.semi_span_ft,
            places=12,
        )

        for inner, outer in zip(
            strips[:-1],
            strips[1:],
        ):
            self.assertAlmostEqual(
                inner.y_outer_ft,
                outer.y_inner_ft,
                places=12,
            )

    def test_strip_centroid_lies_inside_strip(self):
        strips = make_trapezoidal_strips(
            self.planform,
            strip_count=16,
        )

        for strip in strips:
            self.assertGreater(
                strip.y_centroid_ft,
                strip.y_inner_ft,
            )

            self.assertLess(
                strip.y_centroid_ft,
                strip.y_outer_ft,
            )


class TestSectionalAerodynamics(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        self.y = make_uniform_span_grid(
            self.planform.semi_span_ft,
            101,
        )

        self.chord = tuple(
            linear_chord_ft(
                self.planform,
                y,
            )
            for y in self.y
        )

    def test_constant_cl_reconstructs_half_wing_lift(self):
        qbar = 50.0
        cl_value = 0.8

        cl = tuple(
            cl_value
            for _ in self.y
        )

        lift_distribution = sectional_lift_lbf_per_ft(
            self.y,
            self.chord,
            cl,
            qbar,
        )

        calculated = integrate_distributed_load(
            self.y,
            lift_distribution,
        )

        expected = (
            qbar
            * (
                MOONEY_WING_AREA_SQFT
                / 2.0
            )
            * cl_value
        )

        self.assertTrue(
            math.isclose(
                calculated,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

    def test_zero_cl_produces_zero_lift(self):
        cl = tuple(
            0.0
            for _ in self.y
        )

        lift_distribution = sectional_lift_lbf_per_ft(
            self.y,
            self.chord,
            cl,
            100.0,
        )

        self.assertTrue(
            all(
                value == 0.0
                for value in lift_distribution
            )
        )

    def test_negative_cl_produces_downward_load(self):
        cl = tuple(
            -0.5
            for _ in self.y
        )

        lift_distribution = sectional_lift_lbf_per_ft(
            self.y,
            self.chord,
            cl,
            100.0,
        )

        self.assertTrue(
            all(
                value < 0.0
                for value in lift_distribution
            )
        )

    def test_dynamic_pressure_scales_linearly(self):
        cl = tuple(
            0.7
            for _ in self.y
        )

        low = sectional_lift_lbf_per_ft(
            self.y,
            self.chord,
            cl,
            40.0,
        )

        high = sectional_lift_lbf_per_ft(
            self.y,
            self.chord,
            cl,
            80.0,
        )

        for low_value, high_value in zip(
            low,
            high,
        ):
            self.assertTrue(
                math.isclose(
                    high_value,
                    2.0 * low_value,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )


class TestLocalSectionFlow(unittest.TestCase):
    def test_zero_roll_preserves_reference_alpha(self):
        result = local_section_flow(
            reference_alpha_rad=0.10,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            signed_y_ft=10.0,
        )

        self.assertAlmostEqual(
            result.roll_delta_alpha_rad,
            0.0,
            places=12,
        )

        self.assertAlmostEqual(
            result.effective_alpha_rad,
            0.10,
            places=12,
        )

    def test_centerline_has_no_roll_induced_alpha(self):
        result = local_section_flow(
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=1.0,
            signed_y_ft=0.0,
        )

        self.assertAlmostEqual(
            result.roll_delta_alpha_rad,
            0.0,
            places=12,
        )

    def test_roll_produces_opposite_left_right_alpha(self):
        left = local_section_flow(
            reference_alpha_rad=0.0,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.5,
            signed_y_ft=-12.0,
        )

        right = local_section_flow(
            reference_alpha_rad=0.0,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.5,
            signed_y_ft=12.0,
        )

        self.assertLess(
            left.roll_delta_alpha_rad,
            0.0,
        )

        self.assertGreater(
            right.roll_delta_alpha_rad,
            0.0,
        )

        self.assertTrue(
            math.isclose(
                left.roll_delta_alpha_rad,
                -right.roll_delta_alpha_rad,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_roll_delta_matches_exact_geometry(self):
        result = local_section_flow(
            reference_alpha_rad=0.0,
            forward_speed_fps=200.0,
            roll_rate_rad_s=0.4,
            signed_y_ft=10.0,
        )

        expected = math.atan2(
            0.4 * 10.0,
            200.0,
        )

        self.assertTrue(
            math.isclose(
                result.roll_delta_alpha_rad,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_local_speed_includes_roll_velocity(self):
        result = local_section_flow(
            reference_alpha_rad=0.0,
            forward_speed_fps=200.0,
            roll_rate_rad_s=0.5,
            signed_y_ft=10.0,
        )

        expected = math.hypot(
            200.0,
            5.0,
        )

        self.assertTrue(
            math.isclose(
                result.local_speed_fps,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_twist_terms_add_to_effective_alpha(self):
        result = local_section_flow(
            reference_alpha_rad=0.10,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            signed_y_ft=10.0,
            geometric_twist_rad=-0.02,
            aeroelastic_twist_rad=0.01,
        )

        self.assertTrue(
            math.isclose(
                result.effective_alpha_rad,
                0.09,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_rejects_zero_forward_speed(self):
        with self.assertRaises(ValueError):
            local_section_flow(
                reference_alpha_rad=0.0,
                forward_speed_fps=0.0,
                roll_rate_rad_s=0.5,
                signed_y_ft=10.0,
            )


class TestLiftingLine(unittest.TestCase):
    def setUp(self):
        self.wingspan_ft = 36.0
        self.aspect_ratio = 8.0

        self.wing_area_sqft = (
            self.wingspan_ft ** 2
            / self.aspect_ratio
        )

        self.station_count = 21

        (
            self.theta,
            self.signed_y,
        ) = make_lifting_line_collocation(
            self.wingspan_ft,
            self.station_count,
        )

        # Exact elliptical planform:
        #
        #   c(theta) = c_root sin(theta)
        #
        # with:
        #
        #   AR = 4b / (pi c_root)
        self.root_chord_ft = (
            4.0
            * self.wingspan_ft
            / (
                math.pi
                * self.aspect_ratio
            )
        )

        self.chord = tuple(
            self.root_chord_ft
            * math.sin(angle)
            for angle in self.theta
        )

        self.a0 = 2.0 * math.pi

    def solve_constant_alpha(
        self,
        alpha_rad,
    ):
        alpha = tuple(
            alpha_rad
            for _ in self.theta
        )

        a0 = tuple(
            self.a0
            for _ in self.theta
        )

        return solve_lifting_line(
            wingspan_ft=self.wingspan_ft,
            wing_area_sqft=self.wing_area_sqft,
            theta_rad=self.theta,
            chord_ft=self.chord,
            alpha_geometric_rad=alpha,
            lift_curve_slope_per_rad=a0,
        )

    def test_zero_alpha_produces_zero_lift(self):
        result = self.solve_constant_alpha(
            0.0
        )

        self.assertAlmostEqual(
            result.wing_cl,
            0.0,
            places=12,
        )

        self.assertTrue(
            all(
                abs(value) < 1e-12
                for value in result.section_cl
            )
        )

    def test_elliptical_wing_matches_finite_wing_lift_slope(self):
        alpha = 0.08

        result = self.solve_constant_alpha(
            alpha
        )

        expected_slope = (
            self.a0
            / (
                1.0
                + (
                    self.a0
                    / (
                        math.pi
                        * self.aspect_ratio
                    )
                )
            )
        )

        expected_cl = (
            expected_slope
            * alpha
        )

        self.assertTrue(
            math.isclose(
                result.wing_cl,
                expected_cl,
                rel_tol=1e-9,
                abs_tol=1e-12,
            )
        )

    def test_elliptical_wing_uses_only_first_mode(self):
        result = self.solve_constant_alpha(
            0.08
        )

        for coefficient in (
            result.fourier_coefficients[1:]
        ):
            self.assertLess(
                abs(coefficient),
                1e-10,
            )

    def test_elliptical_loading_is_symmetric(self):
        result = self.solve_constant_alpha(
            0.08
        )

        for left, right in zip(
            result.section_cl,
            reversed(
                result.section_cl
            ),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-10,
                    abs_tol=1e-12,
                )
            )

    def test_positive_lift_has_positive_induced_alpha(self):
        result = self.solve_constant_alpha(
            0.08
        )

        self.assertTrue(
            all(
                value > 0.0
                for value in result.induced_alpha_rad
            )
        )

    def test_elliptical_wing_has_unit_span_efficiency(self):
        result = self.solve_constant_alpha(
            0.08
        )

        self.assertTrue(
            math.isclose(
                result.span_efficiency,
                1.0,
                rel_tol=1e-9,
                abs_tol=1e-12,
            )
        )

    def test_double_alpha_doubles_wing_lift(self):
        low = self.solve_constant_alpha(
            0.04
        )

        high = self.solve_constant_alpha(
            0.08
        )

        self.assertTrue(
            math.isclose(
                high.wing_cl,
                2.0 * low.wing_cl,
                rel_tol=1e-10,
                abs_tol=1e-12,
            )
        )

    def test_circulation_goes_to_zero_at_tips(self):
        result = self.solve_constant_alpha(
            0.08
        )

        left_tip = lifting_line_series(
            result.fourier_coefficients,
            0.0,
        )

        right_tip = lifting_line_series(
            result.fourier_coefficients,
            math.pi,
        )

        self.assertAlmostEqual(
            left_tip,
            0.0,
            places=12,
        )

        self.assertAlmostEqual(
            right_tip,
            0.0,
            places=12,
        )

    def test_higher_aspect_ratio_approaches_2d_slope(self):
        alpha = 0.05

        low_ar = 4.0
        high_ar = 20.0

        def solve_for_ar(ar):
            area = (
                self.wingspan_ft ** 2
                / ar
            )

            root_chord = (
                4.0
                * self.wingspan_ft
                / (
                    math.pi
                    * ar
                )
            )

            chord = tuple(
                root_chord
                * math.sin(angle)
                for angle in self.theta
            )

            alpha_distribution = tuple(
                alpha
                for _ in self.theta
            )

            a0_distribution = tuple(
                self.a0
                for _ in self.theta
            )

            return solve_lifting_line(
                wingspan_ft=self.wingspan_ft,
                wing_area_sqft=area,
                theta_rad=self.theta,
                chord_ft=chord,
                alpha_geometric_rad=alpha_distribution,
                lift_curve_slope_per_rad=a0_distribution,
            )

        low = solve_for_ar(
            low_ar
        )

        high = solve_for_ar(
            high_ar
        )

        low_slope = (
            low.wing_cl
            / alpha
        )

        high_slope = (
            high.wing_cl
            / alpha
        )

        self.assertGreater(
            high_slope,
            low_slope,
        )

        self.assertLess(
            abs(
                self.a0
                - high_slope
            ),
            abs(
                self.a0
                - low_slope
            ),
        )

    def test_asymmetric_alpha_breaks_left_right_symmetry(self):
        a0 = tuple(
            self.a0
            for _ in self.theta
        )

        alpha = tuple(
            0.06
            + 0.001
            * y
            for y in self.signed_y
        )

        result = solve_lifting_line(
            wingspan_ft=self.wingspan_ft,
            wing_area_sqft=self.wing_area_sqft,
            theta_rad=self.theta,
            chord_ft=self.chord,
            alpha_geometric_rad=alpha,
            lift_curve_slope_per_rad=a0,
        )

        half = (
            self.station_count
            // 2
        )

        left = result.section_cl[
            half - 3
        ]

        right = result.section_cl[
            half + 3
        ]

        self.assertGreater(
            right,
            left,
        )


class TestAeroStructuralCoupling(unittest.TestCase):

    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        (
            self.theta,
            self.signed_y,
        ) = make_lifting_line_collocation(
            MOONEY_WINGSPAN_FT,
            41,
        )

        self.chord = tuple(
            linear_chord_ft(
                self.planform,
                abs(y),
            )
            for y in self.signed_y
        )

        self.a0 = tuple(
            2.0 * math.pi
            for _ in self.theta
        )

        self.qbar = 70.0

        # Deliberately artificial stiffness.
        #
        # This value exists ONLY to validate aero -> beam coupling.
        # It is not an M20M structural estimate.
        self.test_ei = 2_000_000.0

    def solve_aero(
        self,
        alpha_distribution,
    ):
        return solve_lifting_line(
            wingspan_ft=MOONEY_WINGSPAN_FT,
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            theta_rad=self.theta,
            chord_ft=self.chord,
            alpha_geometric_rad=alpha_distribution,
            lift_curve_slope_per_rad=self.a0,
        )

    def make_ei(
        self,
        aerodynamic_solution,
    ):
        left = extract_half_wing_load(
            aerodynamic_solution,
            side="left",
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        right = extract_half_wing_load(
            aerodynamic_solution,
            side="right",
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        return (
            tuple(
                self.test_ei
                for _ in left.y_ft
            ),
            tuple(
                self.test_ei
                for _ in right.y_ft
            ),
        )

    def test_symmetric_aero_produces_symmetric_bending(self):
        alpha = tuple(
            0.07
            for _ in self.theta
        )

        aero = self.solve_aero(
            alpha
        )

        left_ei, right_ei = self.make_ei(
            aero
        )

        result = solve_one_way_aero_structural_bending(
            lifting_line=aero,
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
            left_ei_lbf_ft2=left_ei,
            right_ei_lbf_ft2=right_ei,
        )

        self.assertTrue(
            math.isclose(
                result.left_load.total_lift_lbf,
                result.right_load.total_lift_lbf,
                rel_tol=1e-10,
                abs_tol=1e-9,
            )
        )

        self.assertTrue(
            math.isclose(
                result.left_bending.root_moment_lbf_ft,
                result.right_bending.root_moment_lbf_ft,
                rel_tol=1e-10,
                abs_tol=1e-9,
            )
        )

        self.assertTrue(
            math.isclose(
                result.left_bending.tip_deflection_ft,
                result.right_bending.tip_deflection_ft,
                rel_tol=1e-10,
                abs_tol=1e-12,
            )
        )

    def test_half_wing_loads_reconstruct_total_lift(self):
        alpha = tuple(
            0.07
            for _ in self.theta
        )

        aero = self.solve_aero(
            alpha
        )

        left = extract_half_wing_load(
            aero,
            side="left",
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        right = extract_half_wing_load(
            aero,
            side="right",
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        integrated_lift = (
            left.total_lift_lbf
            + right.total_lift_lbf
        )

        coefficient_lift = (
            self.qbar
            * MOONEY_WING_AREA_SQFT
            * aero.wing_cl
        )

        # Numerical integration is being performed over the discrete
        # lifting-line structural grid rather than analytically.
        self.assertTrue(
            math.isclose(
                integrated_lift,
                coefficient_lift,
                rel_tol=2e-3,
                abs_tol=1e-6,
            )
        )

    def test_double_alpha_doubles_structural_deflection(self):
        low_alpha = tuple(
            0.04
            for _ in self.theta
        )

        high_alpha = tuple(
            0.08
            for _ in self.theta
        )

        low_aero = self.solve_aero(
            low_alpha
        )

        high_aero = self.solve_aero(
            high_alpha
        )

        low_left_ei, low_right_ei = self.make_ei(
            low_aero
        )

        high_left_ei, high_right_ei = self.make_ei(
            high_aero
        )

        low = solve_one_way_aero_structural_bending(
            lifting_line=low_aero,
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
            left_ei_lbf_ft2=low_left_ei,
            right_ei_lbf_ft2=low_right_ei,
        )

        high = solve_one_way_aero_structural_bending(
            lifting_line=high_aero,
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
            left_ei_lbf_ft2=high_left_ei,
            right_ei_lbf_ft2=high_right_ei,
        )

        self.assertTrue(
            math.isclose(
                high.right_bending.tip_deflection_ft,
                2.0
                * low.right_bending.tip_deflection_ft,
                rel_tol=1e-9,
                abs_tol=1e-12,
            )
        )

    def test_asymmetric_aero_produces_asymmetric_bending(self):
        alpha = tuple(
            0.06
            + 0.001 * y
            for y in self.signed_y
        )

        aero = self.solve_aero(
            alpha
        )

        left_ei, right_ei = self.make_ei(
            aero
        )

        result = solve_one_way_aero_structural_bending(
            lifting_line=aero,
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
            left_ei_lbf_ft2=left_ei,
            right_ei_lbf_ft2=right_ei,
        )

        self.assertGreater(
            result.right_load.total_lift_lbf,
            result.left_load.total_lift_lbf,
        )

        self.assertGreater(
            result.right_bending.root_moment_lbf_ft,
            result.left_bending.root_moment_lbf_ft,
        )

        self.assertGreater(
            result.right_bending.tip_deflection_ft,
            result.left_bending.tip_deflection_ft,
        )

    def test_tip_load_is_zero(self):
        alpha = tuple(
            0.07
            for _ in self.theta
        )

        aero = self.solve_aero(
            alpha
        )

        right = extract_half_wing_load(
            aero,
            side="right",
            qbar_psf=self.qbar,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        self.assertAlmostEqual(
            right.y_ft[-1],
            self.planform.semi_span_ft,
            places=12,
        )

        self.assertAlmostEqual(
            right.lift_lbf_per_ft[-1],
            0.0,
            places=12,
        )


class TestMooneyWingGeometry(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        self.semi_span = (
            self.planform.semi_span_ft
        )

    def test_root_relative_twist_is_zero(self):
        twist = mooney_m20m_geometric_twist_rad(
            0.0,
            self.semi_span,
        )

        self.assertAlmostEqual(
            twist,
            0.0,
            places=12,
        )

    def test_tip_relative_twist_is_minus_one_point_five_deg(self):
        twist = mooney_m20m_geometric_twist_rad(
            self.semi_span,
            self.semi_span,
        )

        expected = math.radians(
            -1.5
        )

        self.assertTrue(
            math.isclose(
                twist,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_left_and_right_twist_are_symmetric(self):
        left = mooney_m20m_geometric_twist_rad(
            -10.0,
            self.semi_span,
        )

        right = mooney_m20m_geometric_twist_rad(
            10.0,
            self.semi_span,
        )

        self.assertTrue(
            math.isclose(
                left,
                right,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_root_incidence_plus_tip_twist_reconstructs_tip_incidence(self):
        reconstructed = (
            M20M_ROOT_INCIDENCE_RAD
            + M20M_TIP_TWIST_FROM_ROOT_RAD
        )

        self.assertTrue(
            math.isclose(
                reconstructed,
                M20M_TIP_INCIDENCE_RAD,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_twist_increases_in_magnitude_outboard(self):
        inner = abs(
            mooney_m20m_geometric_twist_rad(
                5.0,
                self.semi_span,
            )
        )

        outer = abs(
            mooney_m20m_geometric_twist_rad(
                15.0,
                self.semi_span,
            )
        )

        self.assertGreater(
            outer,
            inner,
        )


class TestMooneyWingFlowDistribution(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

    def test_zero_roll_distribution_is_symmetric(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=41,
        )

        for left, right in zip(
            flow.effective_alpha_rad,
            reversed(flow.effective_alpha_rad),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

    def test_centerline_preserves_reference_alpha(self):
        reference_alpha = 0.08

        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=reference_alpha,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=41,
        )

        center = len(flow.theta_rad) // 2

        self.assertTrue(
            math.isclose(
                flow.effective_alpha_rad[center],
                reference_alpha,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_outboard_sections_have_lower_geometric_incidence(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=41,
        )

        center = len(flow.theta_rad) // 2

        self.assertLess(
            flow.effective_alpha_rad[-1],
            flow.effective_alpha_rad[center],
        )

        self.assertLess(
            flow.effective_alpha_rad[0],
            flow.effective_alpha_rad[center],
        )

    def test_positive_roll_breaks_left_right_alpha_symmetry(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.5,
            station_count=41,
        )

        center = len(flow.theta_rad) // 2

        left = flow.effective_alpha_rad[
            center - 8
        ]

        right = flow.effective_alpha_rad[
            center + 8
        ]

        self.assertGreater(
            right,
            left,
        )

    def test_aeroelastic_twist_is_added_stationwise(self):
        station_count = 41

        baseline = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=station_count,
        )

        elastic_twist = tuple(
            0.01
            for _ in baseline.theta_rad
        )

        modified = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=station_count,
            aeroelastic_twist_rad=elastic_twist,
        )

        for original, changed in zip(
            baseline.effective_alpha_rad,
            modified.effective_alpha_rad,
        ):
            self.assertTrue(
                math.isclose(
                    changed,
                    original + 0.01,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

    def test_chord_distribution_is_symmetric(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=41,
        )

        for left, right in zip(
            flow.chord_ft,
            reversed(flow.chord_ft),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )


class TestMooneyAirfoilAerodynamics(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        self.semi_span = (
            self.planform.semi_span_ft
        )

    def test_root_matches_root_airfoil_data(self):
        section = mooney_m20m_section_linear_aerodynamics(
            0.0,
            self.semi_span,
        )

        self.assertTrue(
            math.isclose(
                section.lift_curve_slope_per_rad,
                M20M_ROOT_LIFT_SLOPE_PER_RAD,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

        self.assertTrue(
            math.isclose(
                section.alpha_zero_lift_rad,
                M20M_ROOT_ZERO_LIFT_ALPHA_RAD,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_tip_matches_tip_airfoil_data(self):
        section = mooney_m20m_section_linear_aerodynamics(
            self.semi_span,
            self.semi_span,
        )

        self.assertTrue(
            math.isclose(
                section.lift_curve_slope_per_rad,
                M20M_TIP_LIFT_SLOPE_PER_RAD,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

        self.assertTrue(
            math.isclose(
                section.alpha_zero_lift_rad,
                M20M_TIP_ZERO_LIFT_ALPHA_RAD,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_left_and_right_airfoil_parameters_are_symmetric(self):
        left = mooney_m20m_section_linear_aerodynamics(
            -10.0,
            self.semi_span,
        )

        right = mooney_m20m_section_linear_aerodynamics(
            10.0,
            self.semi_span,
        )

        self.assertTrue(
            math.isclose(
                left.lift_curve_slope_per_rad,
                right.lift_curve_slope_per_rad,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

        self.assertTrue(
            math.isclose(
                left.alpha_zero_lift_rad,
                right.alpha_zero_lift_rad,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_lift_curve_slope_transitions_root_to_tip(self):
        root = mooney_m20m_section_linear_aerodynamics(
            0.0,
            self.semi_span,
        )

        middle = mooney_m20m_section_linear_aerodynamics(
            0.5 * self.semi_span,
            self.semi_span,
        )

        tip = mooney_m20m_section_linear_aerodynamics(
            self.semi_span,
            self.semi_span,
        )

        self.assertGreater(
            root.lift_curve_slope_per_rad,
            middle.lift_curve_slope_per_rad,
        )

        self.assertGreater(
            middle.lift_curve_slope_per_rad,
            tip.lift_curve_slope_per_rad,
        )

    def test_zero_lift_angle_becomes_more_negative_outboard(self):
        root = mooney_m20m_section_linear_aerodynamics(
            0.0,
            self.semi_span,
        )

        middle = mooney_m20m_section_linear_aerodynamics(
            0.5 * self.semi_span,
            self.semi_span,
        )

        tip = mooney_m20m_section_linear_aerodynamics(
            self.semi_span,
            self.semi_span,
        )

        self.assertGreater(
            root.alpha_zero_lift_rad,
            middle.alpha_zero_lift_rad,
        )

        self.assertGreater(
            middle.alpha_zero_lift_rad,
            tip.alpha_zero_lift_rad,
        )

    def test_distribution_matches_station_count(self):
        _, signed_y = make_lifting_line_collocation(
            MOONEY_WINGSPAN_FT,
            41,
        )

        distribution = make_m20m_airfoil_distribution(
            signed_y,
            self.semi_span,
        )

        self.assertEqual(
            len(
                distribution.lift_curve_slope_per_rad
            ),
            41,
        )

        self.assertEqual(
            len(
                distribution.alpha_zero_lift_rad
            ),
            41,
        )

    def test_distribution_is_symmetric(self):
        _, signed_y = make_lifting_line_collocation(
            MOONEY_WINGSPAN_FT,
            41,
        )

        distribution = make_m20m_airfoil_distribution(
            signed_y,
            self.semi_span,
        )

        for left, right in zip(
            distribution.lift_curve_slope_per_rad,
            reversed(
                distribution.lift_curve_slope_per_rad
            ),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

        for left, right in zip(
            distribution.alpha_zero_lift_rad,
            reversed(
                distribution.alpha_zero_lift_rad
            ),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )


class TestLocalAerodynamicState(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        self.rho = 0.002
        self.mu = 4.0e-7

    def test_dynamic_pressure_matches_definition(self):
        state = section_flow_properties(
            chord_ft=5.0,
            speed_fps=200.0,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        expected = (
            0.5
            * self.rho
            * 200.0 ** 2
        )

        self.assertTrue(
            math.isclose(
                state.dynamic_pressure_psf,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_reynolds_number_matches_definition(self):
        state = section_flow_properties(
            chord_ft=5.0,
            speed_fps=200.0,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        expected = (
            self.rho
            * 200.0
            * 5.0
            / self.mu
        )

        self.assertTrue(
            math.isclose(
                state.reynolds_number,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_zero_speed_produces_zero_qbar_and_reynolds(self):
        state = section_flow_properties(
            chord_ft=5.0,
            speed_fps=0.0,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        self.assertEqual(
            state.dynamic_pressure_psf,
            0.0,
        )

        self.assertEqual(
            state.reynolds_number,
            0.0,
        )

    def test_rejects_non_positive_density(self):
        with self.assertRaises(ValueError):
            section_flow_properties(
                chord_ft=5.0,
                speed_fps=200.0,
                air_density_slug_ft3=0.0,
                dynamic_viscosity_slug_ft_s=self.mu,
            )

    def test_rejects_non_positive_viscosity(self):
        with self.assertRaises(ValueError):
            section_flow_properties(
                chord_ft=5.0,
                speed_fps=200.0,
                air_density_slug_ft3=self.rho,
                dynamic_viscosity_slug_ft_s=0.0,
            )

    def test_zero_roll_aero_state_is_symmetric(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=41,
        )

        state = make_m20m_local_aero_state_distribution(
            flow_distribution=flow,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        for left, right in zip(
            state.dynamic_pressure_psf,
            reversed(
                state.dynamic_pressure_psf
            ),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

        for left, right in zip(
            state.reynolds_number,
            reversed(
                state.reynolds_number
            ),
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

    def test_root_reynolds_exceeds_outboard_reynolds_at_zero_roll(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.0,
            station_count=41,
        )

        state = make_m20m_local_aero_state_distribution(
            flow_distribution=flow,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        center = (
            len(
                state.reynolds_number
            )
            // 2
        )

        self.assertGreater(
            state.reynolds_number[center],
            state.reynolds_number[-1],
        )

        self.assertGreater(
            state.reynolds_number[center],
            state.reynolds_number[0],
        )

    def test_roll_local_speed_changes_qbar_outboard(self):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=250.0,
            roll_rate_rad_s=0.7,
            station_count=41,
        )

        state = make_m20m_local_aero_state_distribution(
            flow_distribution=flow,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        center = (
            len(
                state.dynamic_pressure_psf
            )
            // 2
        )

        self.assertGreater(
            state.dynamic_pressure_psf[-1],
            state.dynamic_pressure_psf[center],
        )

        self.assertGreater(
            state.dynamic_pressure_psf[0],
            state.dynamic_pressure_psf[center],
        )


class TestLocalDynamicPressureCoupling(unittest.TestCase):
    def setUp(self):
        self.planform = derive_trapezoidal_planform(
            wing_area_sqft=MOONEY_WING_AREA_SQFT,
            wingspan_ft=MOONEY_WINGSPAN_FT,
            taper_ratio=MOONEY_TAPER_RATIO,
        )

        self.rho = 0.002
        self.mu = 4.0e-7
        self.speed = 250.0

        # Artificial stiffness used ONLY to validate coupling.
        self.test_ei = 2_000_000.0

    def solve_m20m(
        self,
        roll_rate_rad_s,
    ):
        flow = make_m20m_wing_flow_distribution(
            planform=self.planform,
            reference_alpha_rad=0.08,
            forward_speed_fps=self.speed,
            roll_rate_rad_s=roll_rate_rad_s,
            station_count=41,
        )

        airfoils = make_m20m_airfoil_distribution(
            flow.signed_y_ft,
            self.planform.semi_span_ft,
        )

        aero = solve_lifting_line(
            wingspan_ft=self.planform.wingspan_ft,
            wing_area_sqft=self.planform.wing_area_sqft,
            theta_rad=flow.theta_rad,
            chord_ft=flow.chord_ft,
            alpha_geometric_rad=flow.effective_alpha_rad,
            lift_curve_slope_per_rad=airfoils.lift_curve_slope_per_rad,
            alpha_zero_lift_rad=airfoils.alpha_zero_lift_rad,
        )

        state = make_m20m_local_aero_state_distribution(
            flow_distribution=flow,
            air_density_slug_ft3=self.rho,
            dynamic_viscosity_slug_ft_s=self.mu,
        )

        return flow, aero, state

    def test_zero_roll_local_q_matches_scalar_q_load(self):
        _, aero, state = self.solve_m20m(
            0.0
        )

        center = (
            len(state.dynamic_pressure_psf)
            // 2
        )

        scalar_q = (
            state.dynamic_pressure_psf[center]
        )

        scalar = extract_half_wing_load(
            aero,
            side="right",
            qbar_psf=scalar_q,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        local = extract_half_wing_local_q_load(
            aero,
            state,
            side="right",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        for scalar_load, local_load in zip(
            scalar.lift_lbf_per_ft,
            local.lift_lbf_per_ft,
        ):
            self.assertTrue(
                math.isclose(
                    scalar_load,
                    local_load,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

        self.assertTrue(
            math.isclose(
                scalar.total_lift_lbf,
                local.total_lift_lbf,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_zero_roll_local_q_bending_is_symmetric(self):
        _, aero, state = self.solve_m20m(
            0.0
        )

        left = extract_half_wing_local_q_load(
            aero,
            state,
            side="left",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        right = extract_half_wing_local_q_load(
            aero,
            state,
            side="right",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        left_ei = tuple(
            self.test_ei
            for _ in left.y_ft
        )

        right_ei = tuple(
            self.test_ei
            for _ in right.y_ft
        )

        result = solve_one_way_local_q_aero_structural_bending(
            lifting_line=aero,
            local_aero_state=state,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
            left_ei_lbf_ft2=left_ei,
            right_ei_lbf_ft2=right_ei,
        )

        self.assertTrue(
            math.isclose(
                result.left_bending.root_moment_lbf_ft,
                result.right_bending.root_moment_lbf_ft,
                rel_tol=1e-10,
                abs_tol=1e-9,
            )
        )

        self.assertTrue(
            math.isclose(
                result.left_bending.tip_deflection_ft,
                result.right_bending.tip_deflection_ft,
                rel_tol=1e-10,
                abs_tol=1e-12,
            )
        )

    def test_roll_local_q_changes_distributed_load(self):
        _, aero, state = self.solve_m20m(
            0.7
        )

        center = (
            len(state.dynamic_pressure_psf)
            // 2
        )

        scalar_q = (
            state.dynamic_pressure_psf[center]
        )

        scalar = extract_half_wing_load(
            aero,
            side="right",
            qbar_psf=scalar_q,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        local = extract_half_wing_local_q_load(
            aero,
            state,
            side="right",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        self.assertGreater(
            local.total_lift_lbf,
            scalar.total_lift_lbf,
        )

    def test_roll_produces_asymmetric_structural_response(self):
        _, aero, state = self.solve_m20m(
            0.7
        )

        left = extract_half_wing_local_q_load(
            aero,
            state,
            side="left",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        right = extract_half_wing_local_q_load(
            aero,
            state,
            side="right",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        left_ei = tuple(
            self.test_ei
            for _ in left.y_ft
        )

        right_ei = tuple(
            self.test_ei
            for _ in right.y_ft
        )

        result = solve_one_way_local_q_aero_structural_bending(
            lifting_line=aero,
            local_aero_state=state,
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
            left_ei_lbf_ft2=left_ei,
            right_ei_lbf_ft2=right_ei,
        )

        self.assertGreater(
            result.right_load.total_lift_lbf,
            result.left_load.total_lift_lbf,
        )

        self.assertGreater(
            result.right_bending.root_moment_lbf_ft,
            result.left_bending.root_moment_lbf_ft,
        )

        self.assertGreater(
            result.right_bending.tip_deflection_ft,
            result.left_bending.tip_deflection_ft,
        )

    def test_local_q_tip_load_remains_zero(self):
        _, aero, state = self.solve_m20m(
            0.7
        )

        load = extract_half_wing_local_q_load(
            aero,
            state,
            side="right",
            semi_span_ft=self.planform.semi_span_ft,
            tip_chord_ft=self.planform.tip_chord_ft,
        )

        self.assertAlmostEqual(
            load.y_ft[-1],
            self.planform.semi_span_ft,
            places=12,
        )

        self.assertAlmostEqual(
            load.lift_lbf_per_ft[-1],
            0.0,
            places=12,
        )

    def test_local_q_rejects_mismatched_span_grid(self):
        _, aero, state = self.solve_m20m(
            0.0
        )

        bad_y = list(
            state.signed_y_ft
        )

        bad_y[5] += 0.01

        bad_state = M20MLocalAeroStateDistribution(
            signed_y_ft=tuple(bad_y),
            chord_ft=state.chord_ft,
            local_speed_fps=state.local_speed_fps,
            dynamic_pressure_psf=state.dynamic_pressure_psf,
            reynolds_number=state.reynolds_number,
        )

        with self.assertRaises(ValueError):
            extract_half_wing_local_q_load(
                aero,
                bad_state,
                side="right",
                semi_span_ft=self.planform.semi_span_ft,
                tip_chord_ft=self.planform.tip_chord_ft,
            )


class TestDistributedInertialLoading(unittest.TestCase):
    def setUp(self):
        self.y = make_uniform_span_grid(
            10.0,
            101,
        )

    def test_pounds_mass_to_slugs(self):
        mass = pounds_mass_to_slugs(
            STANDARD_GRAVITY_FPS2
        )

        self.assertTrue(
            math.isclose(
                mass,
                1.0,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_normalized_mass_reconstructs_total_mass(self):
        shape = tuple(
            1.0
            for _ in self.y
        )

        expected_mass = 8.0

        distribution = normalize_mass_distribution(
            self.y,
            shape,
            expected_mass,
        )

        calculated = integrate_distributed_load(
            self.y,
            distribution,
        )

        self.assertTrue(
            math.isclose(
                calculated,
                expected_mass,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_zero_mass_produces_zero_distribution(self):
        shape = tuple(
            1.0
            for _ in self.y
        )

        distribution = normalize_mass_distribution(
            self.y,
            shape,
            0.0,
        )

        self.assertTrue(
            all(
                value == 0.0
                for value in distribution
            )
        )

    def test_negative_mass_shape_is_rejected(self):
        shape = tuple(
            -1.0
            if i == 20
            else 1.0
            for i in range(len(self.y))
        )

        with self.assertRaises(ValueError):
            normalize_mass_distribution(
                self.y,
                shape,
                5.0,
            )

    def test_one_g_fuel_inertia_equals_fuel_weight(self):
        # Current M20M FDM tank capacity per side.
        fuel_mass_lbm = 267.0

        fuel_mass_slugs = pounds_mass_to_slugs(
            fuel_mass_lbm
        )

        fuel_shape = tuple(
            1.0
            for _ in self.y
        )

        fuel_distribution = normalize_mass_distribution(
            self.y,
            fuel_shape,
            fuel_mass_slugs,
        )

        zero_structure = tuple(
            0.0
            for _ in self.y
        )

        zero_aero = tuple(
            0.0
            for _ in self.y
        )

        result = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=zero_aero,
            structural_mass_slugs_per_ft=zero_structure,
            fuel_mass_slugs_per_ft=fuel_distribution,
            normal_acceleration_fps2=STANDARD_GRAVITY_FPS2,
        )

        self.assertTrue(
            math.isclose(
                result.total_inertial_force_lbf,
                -267.0,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

    def test_two_g_doubles_inertial_force(self):
        mass = pounds_mass_to_slugs(
            100.0
        )

        shape = tuple(
            1.0
            for _ in self.y
        )

        distribution = normalize_mass_distribution(
            self.y,
            shape,
            mass,
        )

        zero = tuple(
            0.0
            for _ in self.y
        )

        one_g = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=zero,
            structural_mass_slugs_per_ft=distribution,
            fuel_mass_slugs_per_ft=zero,
            normal_acceleration_fps2=STANDARD_GRAVITY_FPS2,
        )

        two_g = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=zero,
            structural_mass_slugs_per_ft=distribution,
            fuel_mass_slugs_per_ft=zero,
            normal_acceleration_fps2=(
                2.0
                * STANDARD_GRAVITY_FPS2
            ),
        )

        self.assertTrue(
            math.isclose(
                two_g.total_inertial_force_lbf,
                2.0
                * one_g.total_inertial_force_lbf,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

    def test_upward_acceleration_produces_downward_inertial_load(self):
        mass = normalize_mass_distribution(
            self.y,
            tuple(
                1.0
                for _ in self.y
            ),
            5.0,
        )

        zero = tuple(
            0.0
            for _ in self.y
        )

        result = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=zero,
            structural_mass_slugs_per_ft=mass,
            fuel_mass_slugs_per_ft=zero,
            normal_acceleration_fps2=STANDARD_GRAVITY_FPS2,
        )

        self.assertTrue(
            all(
                value <= 0.0
                for value in result.inertial_load_lbf_per_ft
            )
        )

        self.assertLess(
            result.total_inertial_force_lbf,
            0.0,
        )

    def test_net_load_is_aero_plus_inertia(self):
        aero = tuple(
            100.0
            for _ in self.y
        )

        structural_mass = normalize_mass_distribution(
            self.y,
            tuple(
                1.0
                for _ in self.y
            ),
            pounds_mass_to_slugs(
                50.0
            ),
        )

        fuel_mass = normalize_mass_distribution(
            self.y,
            tuple(
                1.0
                for _ in self.y
            ),
            pounds_mass_to_slugs(
                25.0
            ),
        )

        result = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=aero,
            structural_mass_slugs_per_ft=structural_mass,
            fuel_mass_slugs_per_ft=fuel_mass,
            normal_acceleration_fps2=STANDARD_GRAVITY_FPS2,
        )

        for aero_load, inertia, net in zip(
            result.aerodynamic_load_lbf_per_ft,
            result.inertial_load_lbf_per_ft,
            result.net_load_lbf_per_ft,
        ):
            self.assertTrue(
                math.isclose(
                    net,
                    aero_load + inertia,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

    def test_more_fuel_reduces_net_upward_load_at_positive_g(self):
        aero = tuple(
            100.0
            for _ in self.y
        )

        zero = tuple(
            0.0
            for _ in self.y
        )

        light_fuel = normalize_mass_distribution(
            self.y,
            tuple(
                1.0
                for _ in self.y
            ),
            pounds_mass_to_slugs(
                50.0
            ),
        )

        heavy_fuel = normalize_mass_distribution(
            self.y,
            tuple(
                1.0
                for _ in self.y
            ),
            pounds_mass_to_slugs(
                200.0
            ),
        )

        light = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=aero,
            structural_mass_slugs_per_ft=zero,
            fuel_mass_slugs_per_ft=light_fuel,
            normal_acceleration_fps2=STANDARD_GRAVITY_FPS2,
        )

        heavy = solve_distributed_inertial_load(
            y_ft=self.y,
            aerodynamic_load_lbf_per_ft=aero,
            structural_mass_slugs_per_ft=zero,
            fuel_mass_slugs_per_ft=heavy_fuel,
            normal_acceleration_fps2=STANDARD_GRAVITY_FPS2,
        )

        self.assertLess(
            heavy.total_net_force_lbf,
            light.total_net_force_lbf,
        )


class TestMooneyWingFuelDistribution(unittest.TestCase):
    def setUp(self):
        self.y = make_uniform_span_grid(
            10.0,
            101,
        )

        # Synthetic tank shape used ONLY for validating distribution
        # bookkeeping. These are NOT claimed M20M tank boundaries.
        self.shape = tuple(
            1.0
            if 2.0 <= y <= 7.0
            else 0.0
            for y in self.y
        )

    def integrated_lbm(
        self,
        distribution,
    ):
        mass_slugs = integrate_distributed_load(
            self.y,
            distribution,
        )

        return (
            mass_slugs
            * STANDARD_GRAVITY_FPS2
        )

    def test_capacity_matches_current_m20m_fdm(self):
        expected = (
            M20M_USABLE_FUEL_PER_WING_GAL
            * M20M_FUEL_DENSITY_LB_PER_GAL
        )

        self.assertTrue(
            math.isclose(
                M20M_FUEL_CAPACITY_PER_WING_LBM,
                expected,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

        self.assertTrue(
            math.isclose(
                M20M_FUEL_CAPACITY_PER_WING_LBM,
                267.0,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_full_left_tank_reconstructs_267_lb(self):
        fuel = make_m20m_wing_fuel_distribution(
            y_ft=self.y,
            relative_tank_shape=self.shape,
            left_fuel_lbm=267.0,
            right_fuel_lbm=0.0,
        )

        self.assertTrue(
            math.isclose(
                self.integrated_lbm(
                    fuel.left_mass_slugs_per_ft
                ),
                267.0,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

    def test_half_right_tank_reconstructs_mass(self):
        fuel = make_m20m_wing_fuel_distribution(
            y_ft=self.y,
            relative_tank_shape=self.shape,
            left_fuel_lbm=0.0,
            right_fuel_lbm=133.5,
        )

        self.assertTrue(
            math.isclose(
                self.integrated_lbm(
                    fuel.right_mass_slugs_per_ft
                ),
                133.5,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

    def test_empty_tank_produces_zero_distribution(self):
        fuel = make_m20m_wing_fuel_distribution(
            y_ft=self.y,
            relative_tank_shape=self.shape,
            left_fuel_lbm=0.0,
            right_fuel_lbm=100.0,
        )

        self.assertTrue(
            all(
                value == 0.0
                for value in fuel.left_mass_slugs_per_ft
            )
        )

    def test_left_and_right_quantities_are_independent(self):
        fuel = make_m20m_wing_fuel_distribution(
            y_ft=self.y,
            relative_tank_shape=self.shape,
            left_fuel_lbm=200.0,
            right_fuel_lbm=100.0,
        )

        left = self.integrated_lbm(
            fuel.left_mass_slugs_per_ft
        )

        right = self.integrated_lbm(
            fuel.right_mass_slugs_per_ft
        )

        self.assertTrue(
            math.isclose(
                left,
                200.0,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

        self.assertTrue(
            math.isclose(
                right,
                100.0,
                rel_tol=1e-12,
                abs_tol=1e-10,
            )
        )

        self.assertGreater(
            left,
            right,
        )

    def test_equal_quantities_produce_equal_distributions(self):
        fuel = make_m20m_wing_fuel_distribution(
            y_ft=self.y,
            relative_tank_shape=self.shape,
            left_fuel_lbm=180.0,
            right_fuel_lbm=180.0,
        )

        for left, right in zip(
            fuel.left_mass_slugs_per_ft,
            fuel.right_mass_slugs_per_ft,
        ):
            self.assertTrue(
                math.isclose(
                    left,
                    right,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            )

    def test_fill_fraction_is_calculated_independently(self):
        fuel = make_m20m_wing_fuel_distribution(
            y_ft=self.y,
            relative_tank_shape=self.shape,
            left_fuel_lbm=267.0,
            right_fuel_lbm=133.5,
        )

        self.assertTrue(
            math.isclose(
                fuel.left_fill_fraction,
                1.0,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

        self.assertTrue(
            math.isclose(
                fuel.right_fill_fraction,
                0.5,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        )

    def test_rejects_over_capacity_fuel(self):
        with self.assertRaises(ValueError):
            make_m20m_wing_fuel_distribution(
                y_ft=self.y,
                relative_tank_shape=self.shape,
                left_fuel_lbm=268.0,
                right_fuel_lbm=0.0,
            )

    def test_rejects_negative_fuel(self):
        with self.assertRaises(ValueError):
            make_m20m_wing_fuel_distribution(
                y_ft=self.y,
                relative_tank_shape=self.shape,
                left_fuel_lbm=-1.0,
                right_fuel_lbm=0.0,
            )


if __name__ == "__main__":
    unittest.main(
        verbosity=2,
    )
