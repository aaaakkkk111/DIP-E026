"""Headless regression tests for the balancing-demo control stack."""

from __future__ import annotations

import math
import unittest

import glfw
import mujoco

import balance_bot as bot


class BalanceBotTest(unittest.TestCase):
    def setUp(self) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(bot.MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.controller = bot.BalanceController(self.model)
        self.harness = bot.Harness(self.model, self.data, self.controller)
        self.harness.reset()

    def step_for(self, seconds: float) -> bot.RobotState:
        for _ in range(round(seconds / self.model.opt.timestep)):
            self.controller.step(self.data)
            mujoco.mj_step(self.model, self.data)
        return self.controller.state(self.data)

    def test_status_ticker_resumes_immediately_after_simulation_time_reset(self) -> None:
        ticker = bot.ResetAwareTicker(0.20)
        self.assertTrue(ticker.is_due(0.00))
        self.assertFalse(ticker.is_due(0.10))
        self.assertTrue(ticker.is_due(0.20))
        self.assertTrue(ticker.is_due(0.70))

        # A relearning episode resets MuJoCo time to zero.  Its first frame
        # must refresh instead of waiting for the previous episode's deadline.
        self.assertTrue(ticker.is_due(0.002))
        self.assertAlmostEqual(ticker.elapsed_time, 0.702)
        self.assertFalse(ticker.is_due(0.10))
        self.assertTrue(ticker.is_due(0.202))
        self.assertAlmostEqual(ticker.elapsed_time, 0.902)
        self.assertIn(
            "t   12.34s",
            self.harness.format_state(
                self.controller.state(self.data), display_time=12.34
            ),
        )

    def test_default_controller_remains_balanced(self) -> None:
        state = self.step_for(5.0)
        self.assertLess(abs(math.degrees(state.pitch)), 5.0)
        self.assertLess(abs(state.forward_speed), 0.10)

    def test_learned_policy_is_controllable_and_closed_loop_stable(self) -> None:
        report = self.controller.policy.report
        self.assertEqual(report.samples, 800)
        self.assertEqual(report.controllability_rank, 4)
        self.assertLess(report.spectral_radius, 1.0)

    def test_visible_online_learning_reaches_a_verified_policy(self) -> None:
        self.controller.start_visible_learning(self.data)
        for _ in range(15_000):
            self.controller.step(self.data)
            mujoco.mj_step(self.model, self.data)
            if self.controller.learning_session is None:
                break
        self.assertIsNone(self.controller.learning_session)
        report = self.controller.policy.report
        self.assertGreaterEqual(report.samples, 90)
        self.assertLess(report.spectral_radius, 0.995)
        self.assertIn("Online learning complete", self.controller.learning_summary())

    def test_text_instruction_is_clamped_and_stoppable(self) -> None:
        response = self.harness.execute_instruction("前进 4.0 并左转 4.0")
        command = self.controller.command
        limits = self.controller.supervisor.limits
        self.assertEqual(limits.max_speed, 1.20)
        self.assertEqual(limits.max_yaw_rate, 1.50)
        self.assertEqual(command.forward_speed, limits.max_speed)
        self.assertEqual(command.yaw_rate, limits.max_yaw_rate)
        self.assertIn("applied target", response)
        self.harness.execute_instruction("停止")
        self.assertEqual(command.forward_speed, 0.0)
        self.assertEqual(command.yaw_rate, 0.0)

    def test_command_numbers_allow_connector_words_and_units(self) -> None:
        response = self.harness.execute_instruction(
            "forward at 0.42 m/s and turn left at 0.35 rad/s"
        )
        self.assertAlmostEqual(self.controller.command.forward_speed, 0.42)
        self.assertAlmostEqual(self.controller.command.yaw_rate, 0.35)
        self.assertIn("Requested speed +0.42", response)
        self.assertIn("Requested yaw rate +0.35", response)

        self.harness.execute_instruction("前进速度设置为 0.27 m/s")
        self.assertAlmostEqual(self.controller.command.forward_speed, 0.27)

    def test_arrow_keys_change_only_robot_targets(self) -> None:
        self.harness.handle_key(glfw.KEY_UP)
        self.harness.handle_key(glfw.KEY_LEFT)
        self.assertEqual(self.controller.command.forward_speed, 0.15)
        self.assertEqual(self.controller.command.yaw_rate, 0.25)
        self.harness.handle_key(glfw.KEY_DOWN)
        self.harness.handle_key(glfw.KEY_RIGHT)
        self.assertEqual(self.controller.command.forward_speed, 0.0)
        self.assertEqual(self.controller.command.yaw_rate, 0.0)

    def test_language_switch_localizes_output_without_changing_command_grammar(self) -> None:
        self.assertEqual(self.harness.language, "en")
        self.harness.set_language("en")
        response = self.harness.execute_instruction("前进 0.3")
        self.assertIn("applied target is +0.30", response)
        self.assertEqual(self.controller.command.forward_speed, 0.3)
        self.assertIn("Motion:", self.harness.help_text())
        self.assertIn("Offline LQR", self.controller.learning_summary("en"))

        self.harness.set_language("zh")
        response = self.harness.execute_instruction("stop")
        self.assertIn("已停止", response)
        self.assertEqual(self.controller.command.forward_speed, 0.0)
        with self.assertRaises(ValueError):
            self.harness.set_language("fr")

    def test_large_pitch_freezes_motion_without_automatic_reset(self) -> None:
        angle = math.radians(30.0)
        self.data.qpos[3:7] = (math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0)
        self.controller.command.forward_speed = 0.4
        self.controller.command.yaw_rate = 0.3
        mujoco.mj_forward(self.model, self.data)
        state = self.controller.step(self.data)
        self.assertAlmostEqual(math.degrees(state.pitch), 30.0, places=5)
        self.assertAlmostEqual(self.data.qpos[0], 0.0)
        self.assertAlmostEqual(self.data.qpos[5], math.sin(angle / 2.0))
        self.assertEqual(self.controller.command.forward_speed, 0.0)
        self.assertEqual(self.controller.command.yaw_rate, 0.0)
        self.assertEqual(self.controller.supervisor.mode, "RECOVERY")

    def test_scenario_parameters_and_push_are_bounded(self) -> None:
        scenario = self.controller.scenario
        scenario.set_friction(9.0)
        scenario.set_slope(4.0)
        scenario.set_imu_delay(4.0)
        scenario.set_motor_delay(4.0)
        scenario.schedule_push(self.data.time, 200.0)
        scenario.prepare_step(self.data)
        self.assertEqual(scenario.config.friction, 2.5)
        self.assertEqual(scenario.config.slope_deg, 4.0)
        self.assertEqual(scenario.config.imu_delay_ms, 4.0)
        self.assertEqual(scenario.config.motor_delay_ms, 4.0)
        self.assertEqual(self.data.xfrc_applied[scenario.chassis_id, 0], 80.0)
        self.assertEqual(scenario.delay_motors((1.0, 2.0)), (0.0, 0.0))
        self.assertEqual(scenario.delay_motors((3.0, 4.0)), (0.0, 0.0))
        self.assertEqual(scenario.delay_motors((5.0, 6.0)), (1.0, 2.0))


if __name__ == "__main__":
    unittest.main()
