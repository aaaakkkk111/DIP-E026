"""Regression tests for the STM32 firmware-PID MuJoCo reproduction."""

from __future__ import annotations

import math
import unittest

import numpy as np

import firmware_pid_sim as sim


class FirmwarePidSimulationTest(unittest.TestCase):
    def test_timing_and_encoder_constants_match_firmware(self) -> None:
        car = sim.FirmwarePidSimulation(initial_pitch_deg=0.0)
        self.assertAlmostEqual(car.model.opt.timestep, 0.001)
        self.assertEqual(sim.CONTROL_STEPS, 5)
        self.assertAlmostEqual(sim.CONTROL_PERIOD_S, 0.005)
        self.assertEqual(sim.ENCODER_COUNTS_PER_WHEEL_REV, 1320)
        for _ in range(25):
            car.step()
        self.assertEqual(car.controller.control_tick, 5)

    def test_documented_geometry_and_adjustable_contact_match_model(self) -> None:
        car = sim.FirmwarePidSimulation(initial_pitch_deg=0.0)
        self.assertAlmostEqual(sim.WHEEL_DIAMETER_M, 0.067)
        self.assertAlmostEqual(sim.WHEEL_TRACK_M, 0.126)
        left_tire, right_tire = car.tire_geom_ids
        self.assertAlmostEqual(car.model.geom_size[left_tire, 0] * 2.0, 0.067)
        self.assertAlmostEqual(car.model.geom_size[right_tire, 0] * 2.0, 0.067)
        left_body = car.model.geom_bodyid[left_tire]
        right_body = car.model.geom_bodyid[right_tire]
        self.assertAlmostEqual(
            abs(car.model.body_pos[left_body, 1] - car.model.body_pos[right_body, 1]),
            sim.WHEEL_TRACK_M,
        )

        car.set_ground_friction(0.85)
        self.assertAlmostEqual(car.ground_friction, 0.85)
        self.assertAlmostEqual(car.model.geom_friction[car.ground_geom_id, 0], 0.85)
        self.assertAlmostEqual(car.model.geom_friction[left_tire, 0], 0.85)
        self.assertAlmostEqual(car.model.geom_friction[right_tire, 0], 0.85)
        self.assertEqual(car.model.geom_bodyid[car.ground_geom_id], 0)
        np.testing.assert_allclose(
            car.model.geom_solref[car.ground_geom_id], (0.002, 1.0)
        )
        np.testing.assert_allclose(
            car.model.geom_solimp[car.ground_geom_id, :3],
            (0.995, 0.9999, 0.0001),
        )
        self.assertGreaterEqual(car.total_mass_kg, 1.0)
        self.assertLessEqual(car.total_mass_kg, 1.2)

    def test_direction_commands_and_four_way_pushes(self) -> None:
        car = sim.FirmwarePidSimulation(initial_pitch_deg=0.0)
        expected = {
            "forward": (sim.MOTION_SPEED_M_S, 0.0),
            "backward": (-sim.MOTION_SPEED_M_S, 0.0),
            "left": (0.0, sim.MOTION_YAW_RATE_RAD_S),
            "right": (0.0, -sim.MOTION_YAW_RATE_RAD_S),
            "stop": (0.0, 0.0),
        }
        for direction, targets in expected.items():
            with self.subTest(direction=direction):
                car.command_direction(direction)
                self.assertEqual(
                    (
                        car.controller.target_speed_m_s,
                        car.controller.target_yaw_rate_rad_s,
                    ),
                    targets,
                )

        chassis = car.chassis_body_id
        for direction, expected_force in {
            "forward": (2.0, 0.0),
            "backward": (-2.0, 0.0),
            "left": (0.0, 2.0),
            "right": (0.0, -2.0),
        }.items():
            with self.subTest(push=direction):
                car.push_direction(direction, 2.0)
                car._apply_external_force()
                self.assertEqual(
                    tuple(car.data.xfrc_applied[chassis, 0:2]), expected_force
                )

        car.push(30.0, 40.0)
        car._apply_external_force()
        self.assertAlmostEqual(
            float(math.hypot(*car.data.xfrc_applied[chassis, 0:2])), 40.0
        )

    def test_vendor_mode_gains_are_exact(self) -> None:
        normal = sim.GAINS[sim.ControlMode.NORMAL]
        weight = sim.GAINS[sim.ControlMode.WEIGHT]
        self.assertEqual(
            (normal.balance_kp, normal.balance_kd, normal.velocity_kp, normal.velocity_ki),
            (9600.0, 48.0, 6200.0, 31.0),
        )
        self.assertEqual((normal.turn_kp, normal.turn_kd), (1700.0, 20.0))
        self.assertEqual(
            (normal.balance_scale, normal.velocity_scale, normal.turn_scale),
            (1.0, 1.0, 1.0),
        )
        self.assertEqual(
            (weight.balance_kp, weight.balance_kd, weight.velocity_kp, weight.velocity_ki),
            (9600.0, 75.0, 7000.0, 35.0),
        )
        self.assertEqual((weight.turn_kp, weight.turn_kd), (1400.0, 20.0))
        self.assertEqual(
            (weight.balance_scale, weight.velocity_scale, weight.turn_scale),
            (2.0, 1.35, 1.0),
        )

    def test_dead_zone_and_final_pwm_limit_match_firmware(self) -> None:
        self.assertEqual(sim._firmware_dead_zone_and_limit(0), 0)
        self.assertEqual(sim._firmware_dead_zone_and_limit(1), 1301)
        self.assertEqual(sim._firmware_dead_zone_and_limit(-1), -1301)
        self.assertEqual(sim._firmware_dead_zone_and_limit(5000), 2600)
        self.assertEqual(sim._firmware_dead_zone_and_limit(-5000), -2600)

    def test_payload_changes_mass_and_inertia_without_resetting_time(self) -> None:
        car = sim.FirmwarePidSimulation(payload_kg=0.0, initial_pitch_deg=0.0)
        baseline = car.total_mass_kg
        for _ in range(10):
            car.step()
        before_time = car.data.time
        car.set_payload_mass(2.5)
        self.assertAlmostEqual(car.payload_kg, 2.5)
        self.assertAlmostEqual(car.total_mass_kg - baseline, 2.5, places=4)
        self.assertAlmostEqual(car.data.time, before_time)
        self.assertTrue((car.model.body_inertia[car.payload_body_id] > 0.0).all())

    def test_weight_mode_changes_same_state_output(self) -> None:
        observation = sim.RobotObservation(
            time_s=0.0,
            pitch_rad=math.radians(1.0),
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
            forward_speed_m_s=0.0,
            left_wheel_angle_rad=0.0,
            right_wheel_angle_rad=0.0,
        )
        normal = sim.FirmwarePIDController(sim.ControlMode.NORMAL)
        weight = sim.FirmwarePIDController(sim.ControlMode.WEIGHT)
        normal.reset(observation)
        weight.reset(observation)
        normal.update(observation)
        weight.update(observation)
        self.assertEqual(normal.last.balance_pwm, 96)
        self.assertEqual(weight.last.balance_pwm, 192)
        self.assertGreater(weight.last.pwm_left, normal.last.pwm_left)

    def test_fall_and_low_battery_cut_off_pwm(self) -> None:
        controller = sim.FirmwarePIDController()
        fallen = sim.RobotObservation(
            time_s=0.0,
            pitch_rad=math.radians(41.0),
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
            forward_speed_m_s=0.0,
            left_wheel_angle_rad=0.0,
            right_wheel_angle_rad=0.0,
        )
        controller.reset(fallen)
        self.assertEqual(controller.update(fallen), (0, 0))
        self.assertTrue(controller.last.stopped)

        upright = sim.RobotObservation(
            time_s=0.0,
            pitch_rad=math.radians(1.0),
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
            forward_speed_m_s=0.0,
            left_wheel_angle_rad=0.0,
            right_wheel_angle_rad=0.0,
        )
        controller.reset(upright)
        controller.battery_voltage = 9.5
        self.assertEqual(controller.update(upright), (0, 0))
        self.assertTrue(controller.last.stopped)

    def test_normal_and_weight_modes_hold_their_intended_loads(self) -> None:
        scenarios = (
            (sim.ControlMode.NORMAL, 0.0, 3.0),
            (sim.ControlMode.WEIGHT, 4.0, 0.5),
        )
        for mode, payload, initial_pitch in scenarios:
            with self.subTest(mode=mode, payload=payload):
                car = sim.FirmwarePidSimulation(
                    mode=mode,
                    payload_kg=payload,
                    initial_pitch_deg=initial_pitch,
                )
                maximum_pitch = 0.0
                for _ in range(round(3.0 / car.model.opt.timestep)):
                    state = car.step()
                    maximum_pitch = max(maximum_pitch, abs(math.degrees(state.pitch_rad)))
                self.assertLess(maximum_pitch, 5.0)
                self.assertFalse(car.controller.last.stopped)

    def test_camera_follow_smoothly_tracks_chassis(self) -> None:
        current = np.array([0.0, 0.0, 0.25])
        chassis = np.array([1.0, -2.0, 0.04])
        updated = sim.camera_follow_lookat(current, chassis)
        expected = np.array([0.12, -0.24, 0.232])
        np.testing.assert_allclose(updated, expected)

    def test_max_ui_push_recovers_in_all_four_directions(self) -> None:
        for direction in sim.PUSH_DIRECTIONS:
            with self.subTest(direction=direction):
                car = sim.FirmwarePidSimulation(initial_pitch_deg=0.0)
                maximum_pitch_deg = 0.0
                ever_stopped = False
                pushed = False
                for _ in range(round(8.0 / car.model.opt.timestep)):
                    if not pushed and car.data.time >= 1.0:
                        car.push_direction(
                            direction, max(sim.PUSH_FORCE_LEVELS_N.values())
                        )
                        pushed = True
                    state = car.step()
                    maximum_pitch_deg = max(
                        maximum_pitch_deg, abs(math.degrees(state.pitch_rad))
                    )
                    ever_stopped = ever_stopped or car.controller.last.stopped

                self.assertTrue(pushed)
                self.assertFalse(ever_stopped)
                self.assertLess(maximum_pitch_deg, 25.0)
                self.assertLess(abs(math.degrees(state.pitch_rad)), 3.0)
                self.assertLess(abs(state.forward_speed_m_s), 0.05)


if __name__ == "__main__":
    unittest.main()
