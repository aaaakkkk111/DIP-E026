"""Tests for the hybrid PID and neural residual controller."""

from __future__ import annotations

import math
import unittest

import numpy as np

import firmware_pid_sim as fw
import neural_residual_sim as neural


class NeuralResidualSimulationTest(unittest.TestCase):
    def test_mode_supervisor_uses_10_degree_threshold_and_hysteresis(self) -> None:
        supervisor = neural.HybridModeSupervisor()
        self.assertIs(supervisor.select(math.radians(9.9)), fw.ControlMode.NORMAL)
        self.assertIs(supervisor.select(math.radians(10.0)), fw.ControlMode.WEIGHT)
        self.assertIs(supervisor.select(math.radians(8.0)), fw.ControlMode.WEIGHT)
        self.assertIs(supervisor.select(math.radians(7.0)), fw.ControlMode.NORMAL)
        self.assertIs(supervisor.select(math.radians(-11.0)), fw.ControlMode.WEIGHT)

    def test_policy_observation_is_fixed_normalized_and_finite(self) -> None:
        simulation = neural.NeuralResidualSimulation(neural.ZeroResidualPolicy())
        observation = simulation.prepare_control()
        self.assertEqual(observation.shape, (len(neural.POLICY_OBSERVATION_NAMES),))
        self.assertEqual(observation.dtype, np.float32)
        self.assertTrue(np.isfinite(observation).all())
        self.assertTrue((np.abs(observation) <= 5.0).all())

    def test_residual_is_bounded_and_added_at_motor_pwm_interface(self) -> None:
        simulation = neural.NeuralResidualSimulation(
            neural.ZeroResidualPolicy(), initial_pitch_deg=0.0
        )
        simulation.prepare_control()
        base = np.asarray(simulation.last_base_pwm, dtype=int)
        combined = simulation.apply_residual_action(np.asarray((2.0, -2.0)))
        expected = np.clip(
            base + np.asarray((neural.RESIDUAL_PWM_LIMIT, -neural.RESIDUAL_PWM_LIMIT)),
            -fw.PWM_COMMAND_LIMIT,
            fw.PWM_COMMAND_LIMIT,
        )
        self.assertEqual(
            simulation.last_residual_pwm,
            (neural.RESIDUAL_PWM_LIMIT, -neural.RESIDUAL_PWM_LIMIT),
        )
        self.assertEqual(combined, tuple(int(value) for value in expected))
        self.assertEqual(simulation.motor.command, combined)

    def test_environment_has_valid_gymnasium_transition(self) -> None:
        environment = neural.ResidualBalanceEnv(episode_seconds=0.05)
        observation, info = environment.reset(seed=5)
        self.assertTrue(environment.observation_space.contains(observation))
        self.assertEqual(info["mode"], environment.simulation.controller.mode.value)
        next_observation, reward, terminated, truncated, info = environment.step(
            np.zeros(2, dtype=np.float32)
        )
        self.assertTrue(environment.observation_space.contains(next_observation))
        self.assertTrue(math.isfinite(reward))
        self.assertIsInstance(terminated, bool)
        self.assertIsInstance(truncated, bool)
        self.assertIn("residual_pwm", info)
        environment.close()

    def test_zero_policy_produces_exact_hybrid_pid_baseline(self) -> None:
        simulation = neural.NeuralResidualSimulation(
            neural.ZeroResidualPolicy(), initial_pitch_deg=21.0
        )
        simulation.step()
        self.assertIs(simulation.controller.mode, fw.ControlMode.WEIGHT)
        self.assertEqual(simulation.last_residual_pwm, (0, 0))
        self.assertEqual(simulation.last_combined_pwm, simulation.last_base_pwm)


if __name__ == "__main__":
    unittest.main()
