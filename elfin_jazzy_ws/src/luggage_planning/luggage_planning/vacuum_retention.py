#!/usr/bin/env python3
"""Vacuum payload-retention calculations, independent of ROS/Gazebo."""
import math


def retention_metrics(
        mass_kg, tilt_deg, pressure_kpa, effective_area_m2,
        seal_efficiency, friction_coefficient, linear_accel_mps2,
        angular_accel_radps2, payload_radius_m):
    mass = max(0.0, float(mass_kg))
    tilt = math.radians(abs(float(tilt_deg)))
    normal_capacity = (
        max(0.0, float(pressure_kpa)) * 1000.0
        * max(0.0, float(effective_area_m2))
        * max(0.0, min(1.0, float(seal_efficiency))))
    shear_capacity = (
        normal_capacity * max(0.0, float(friction_coefficient)))
    gravity = 9.80665
    normal_required = mass * (
        gravity * math.cos(tilt) + max(0.0, float(linear_accel_mps2)))
    shear_required = mass * (
        gravity * math.sin(tilt)
        + max(0.0, float(linear_accel_mps2))
        + max(0.0, float(angular_accel_radps2))
        * max(0.0, float(payload_radius_m)))
    normal_margin = (
        normal_capacity / normal_required
        if normal_required > 0.0 else float("inf"))
    shear_margin = (
        shear_capacity / shear_required
        if shear_required > 0.0 else float("inf"))
    return {
        "mass_kg": mass,
        "tilt_deg": abs(float(tilt_deg)),
        "normal_capacity_n": normal_capacity,
        "shear_capacity_n": shear_capacity,
        "normal_required_n": normal_required,
        "shear_required_n": shear_required,
        "normal_margin": normal_margin,
        "shear_margin": shear_margin,
        "margin": min(normal_margin, shear_margin),
    }
