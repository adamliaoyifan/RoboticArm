#!/usr/bin/env python3
import unittest

from luggage_planning.vacuum_retention import retention_metrics


class TestVacuumRetention(unittest.TestCase):
    def _metrics(self, mass, tilt=5.0):
        return retention_metrics(
            mass_kg=mass,
            tilt_deg=tilt,
            pressure_kpa=70.0,
            effective_area_m2=0.012,
            seal_efficiency=0.80,
            friction_coefficient=0.60,
            linear_accel_mps2=2.0,
            angular_accel_radps2=1.0,
            payload_radius_m=0.48,
        )

    def test_catalog_mass_matrix_has_two_x_margin(self):
        for mass in (8.0, 15.0, 23.0):
            self.assertGreaterEqual(self._metrics(mass)["margin"], 2.0)

    def test_margin_decreases_with_mass_and_tilt(self):
        self.assertGreater(
            self._metrics(8.0)["margin"],
            self._metrics(23.0)["margin"])
        self.assertGreater(
            self._metrics(23.0, tilt=0.0)["shear_margin"],
            self._metrics(23.0, tilt=20.0)["shear_margin"])

    def test_low_pressure_fails_heavy_payload(self):
        metrics = retention_metrics(
            mass_kg=23.0,
            tilt_deg=5.0,
            pressure_kpa=35.0,
            effective_area_m2=0.012,
            seal_efficiency=0.80,
            friction_coefficient=0.60,
            linear_accel_mps2=2.0,
            angular_accel_radps2=1.0,
            payload_radius_m=0.48,
        )
        self.assertLess(metrics["margin"], 2.0)


if __name__ == "__main__":
    unittest.main()
