#!/usr/bin/env python3

import math
import unittest

from wing_structure import (
    derive_trapezoidal_planform,
    linear_chord_ft,
    make_uniform_span_grid,
    solve_cantilever_bending,
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


if __name__ == "__main__":
    unittest.main(
        verbosity=2,
    )
