import math
import unittest

import numpy as np

from suspension_geometry_lab._pacejka.model import api
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.tire_preview import wheel_preview_pose
from suspension_geometry_lab.tire_response import (DEFAULT_TIR, TIR_DIR, calculate_front_axle_response,
                                                   calculate_tire_response, load_tire_parameters)


class TireResponseTests(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(default_project())
        self.params = load_tire_parameters(TIR_DIR / DEFAULT_TIR)

    def response(self, corner, steer, alpha, **kwargs):
        rows, warnings = calculate_tire_response(self.engine, self.params, corner=corner, steer_deg=steer,
                                                 slip_angles_deg=[alpha], slip_ratio=.08, **kwargs)
        return rows[0], warnings

    def test_combined_mz_and_force_moment_are_projected_once(self):
        row, warnings = self.response("FL", 14, 3.5, fz_n=850, pressure_kpa=80, speed_mps=15,
                                      camber_offset_deg=.5)
        self.assertEqual(warnings, [])
        pose = self.engine.wheel_end_pose(14, "FL")
        ia = math.degrees(math.asin(pose["normal"][2])) + .5
        condition = api.CalculationInput(FZ=850, SA=3.5, SL=.08, IA=ia, pressure=80000, V=15,
                                         tir_params=self.params)
        fx, fy, mz = (api.MF_Universal_solver(key, condition) for key in ("FX", "FY", "MZ"))
        self.assertAlmostEqual(row["mz_nm"], mz)
        self.assertAlmostEqual(row["fx_n"], fx)
        self.assertAlmostEqual(row["fy_n"], fy)
        self.assertNotAlmostEqual(mz, api.MF_Universal_solver("MZ0", condition))

        nx, ny, _ = pose["normal"]
        heading = math.atan2(-nx, ny)
        c, s = math.cos(heading), math.sin(heading)
        force = [fx*c - fy*s, fx*s + fy*c, 850.]
        arm = pose["contact"] - pose["origin"]
        moment = np.cross(arm, force)
        trail_moment = np.cross(arm, [-fy*s, fy*c, 0.])[2]
        self.assertNotAlmostEqual(trail_moment, 0.)
        self.assertAlmostEqual(row["trail_moment_nm"], trail_moment)
        self.assertAlmostEqual(row["aligning_total_nm"], mz + trail_moment)
        self.assertAlmostEqual(row["torque_contact_nm"], pose["axis"] @ moment)
        self.assertAlmostEqual(row["torque_total_nm"], pose["axis"] @ (moment + [0., 0., mz]))

    def test_mirrored_wheels_and_lift(self):
        left, _ = self.response("FL", -10, -3, camber_offset_deg=1)
        right, _ = self.response("FR", 10, 3, camber_offset_deg=-1)
        for key in ("fx_n", "fz_n"):
            self.assertAlmostEqual(left[key], right[key])
        for key in ("fy_n", "mz_nm", "camber_deg", "trail_moment_nm", "aligning_total_nm", "torque_total_nm"):
            self.assertAlmostEqual(left[key], -right[key])
        self.assertAlmostEqual(right["fz_n"], self.engine.car.w * self.engine.car.cg_rate[0] / 2)

        lifted, warnings = self.response("RR", 4, 2, fz_n=0)
        self.assertTrue(warnings)
        for key in ("fx_n", "fy_n", "mz_nm", "trail_moment_nm", "aligning_total_nm", "torque_total_nm"):
            self.assertEqual(lifted[key], 0.)

    def test_preview_uses_vehicle_side_and_camber_before_tir_mirroring(self):
        for corner in ("FL", "FR", "RL", "RR"):
            side = 1 if corner.endswith("L") else -1
            for camber in (-2., 0., 2.):
                with self.subTest(corner=corner, camber=camber):
                    steer, alpha = 10., 3.
                    wheel = wheel_preview_pose(self.engine, corner, steer, alpha, camber)
                    result, _ = self.response(corner, steer, alpha, wheel_camber_deg=camber)
                    self.assertEqual(wheel.camber, camber)
                    self.assertEqual(result["wheel_camber_deg"], camber)
                    self.assertEqual(result["camber_deg"], -side * camber)
                    self.assertEqual(result["model_camber_deg"], -camber)  # Left TIR
                    self.assertAlmostEqual(math.degrees(math.asin(wheel.axle[2])), -side * camber)
                    wheel_top = np.cross(wheel.forward, wheel.axle)
                    self.assertEqual(np.sign(wheel_top[1] * side), np.sign(camber))
                    self.assertAlmostEqual(math.degrees(math.asin(np.cross(wheel.forward, wheel.travel)[2])), alpha)

    def test_camber_can_follow_geometry_or_override_it_at_steer(self):
        for corner, side in (("FL", 1), ("FR", -1)):
            geometry, _ = self.response(corner, 10., 0.)
            self.assertNotEqual(geometry["wheel_camber_deg"], 0.)
            self.assertAlmostEqual(geometry["wheel_camber_deg"], -side * geometry["camber_deg"])
            blank = wheel_preview_pose(self.engine, corner, 10., 0.)
            self.assertAlmostEqual(blank.camber, geometry["wheel_camber_deg"])
            vertical, _ = self.response(corner, 10., 0., wheel_camber_deg=0.)
            self.assertEqual(vertical["camber_deg"], 0.)
            for invalid in (-90., 90., math.nan):
                with self.assertRaisesRegex(ValueError, "Camber"):
                    self.response(corner, 10., 0., wheel_camber_deg=invalid)

    def test_out_of_range_conditions_are_visible_without_clamping(self):
        row, warnings = self.response("FR", 10, 3, fz_n=25, camber_offset_deg=12)
        self.assertEqual(row["fz_n"], 25)
        self.assertIn("輪荷", warnings[0])
        self.assertIn("外傾角", warnings[0])
        with self.assertRaises(ValueError):
            self.response("FR", 10, 3, speed_mps=0)

    def test_front_axle_sums_signed_moments_with_independent_conditions(self):
        left = dict(steer_deg=-10, slip_angle_deg=-3, slip_ratio=.08, camber_offset_deg=1)
        right = dict(steer_deg=10, slip_angle_deg=3, slip_ratio=.08, camber_offset_deg=-1)
        rows, _ = calculate_front_axle_response(self.engine, self.params, left=left, right=right, slip_offsets_deg=[0])
        mirrored = rows[0]
        self.assertNotAlmostEqual(mirrored["fl_mz_nm"], 0.)
        self.assertAlmostEqual(mirrored["front_mz_nm"], 0.)
        self.assertAlmostEqual(mirrored["front_aligning_total_nm"], 0.)
        self.assertAlmostEqual(mirrored["front_torque_total_nm"], 0.)

        right.update(steer_deg=-6, slip_angle_deg=-2, slip_ratio=.04, fz_n=950, pressure_kpa=75, speed_mps=12)
        rows, _ = calculate_front_axle_response(self.engine, self.params, left=left, right=right,
                                                slip_offsets_deg=[0., 1.])
        for offset, row in zip((0., 1.), rows):
            self.assertEqual(row["fl_slip_angle_deg"], -3. + offset)
            self.assertEqual(row["fr_slip_angle_deg"], -2. + offset)
            self.assertEqual(row["fr_model_slip_angle_deg"], 2. - offset)
            self.assertAlmostEqual(row["fr_model_camber_deg"], -row["fr_camber_deg"])
            condition = api.CalculationInput(FZ=950, SA=2. - offset, SL=.04, IA=-row["fr_camber_deg"],
                                             pressure=75000, V=12, tir_params=self.params)
            self.assertAlmostEqual(row["fr_mz_nm"], -api.MF_Universal_solver("MZ", condition))
            self.assertAlmostEqual(row["front_mz_nm"], row["fl_mz_nm"] + row["fr_mz_nm"])
            self.assertAlmostEqual(row["front_trail_moment_nm"], row["fl_trail_moment_nm"] + row["fr_trail_moment_nm"])
            self.assertAlmostEqual(row["front_aligning_total_nm"], row["front_mz_nm"] + row["front_trail_moment_nm"])
            self.assertAlmostEqual(row["front_torque_total_nm"], row["fl_torque_total_nm"] + row["fr_torque_total_nm"])
        self.assertNotAlmostEqual(rows[0]["front_mz_nm"], 0.)


if __name__ == "__main__":
    unittest.main()
