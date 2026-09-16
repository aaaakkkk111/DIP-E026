from __future__ import annotations

import unittest
import math
from unittest.mock import patch

import mujoco

from route_a.experiment_runner import ExperimentRunner, ProtocolClient, ProtocolTimeout
from route_a.firmware_profile import DEFAULT_PID, EnvironmentConfig
from route_a.protocol import FrameStreamParser, build_pid_query_frame
from route_a.transport import ImpairmentConfig, MuJoCoTransport, SerialTransport, Transport


class TransportPhysicsTests(unittest.TestCase):
    def test_physics_is_1ms_control_is_5ms(self):
        transport=MuJoCoTransport(EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0))
        transport.advance(.010)
        self.assertEqual(transport.plant.physics_steps,10); self.assertEqual(transport.stm32.tick_ms,10)

    def test_protocol_client_only_uses_transport_bytes(self):
        transport=MuJoCoTransport(EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0))
        pid,raw=ProtocolClient(transport).query_pid(); self.assertEqual(pid,DEFAULT_PID); self.assertGreaterEqual(len(raw),2)
        self.assertIsInstance(transport,Transport)

    def test_latency_fragmentation_loss_and_corruption(self):
        delayed=MuJoCoTransport(impairment=ImpairmentConfig(latency_ms=100,fragment_max_bytes=2))
        delayed.write(build_pid_query_frame()); delayed.advance(.05); self.assertEqual(delayed.read(),b"")
        delayed.advance(.5); parser=FrameStreamParser(); frames=[]
        while chunk:=delayed.read(): frames.extend(parser.feed(chunk))
        self.assertGreaterEqual(len(frames),2)
        dropped=MuJoCoTransport(impairment=ImpairmentConfig(byte_drop_probability=1)); dropped.write(build_pid_query_frame()); dropped.advance(.5)
        self.assertEqual(dropped.read(),b""); self.assertGreater(dropped.stats["dropped_bytes"],0)
        corrupt=MuJoCoTransport(impairment=ImpairmentConfig(byte_corruption_probability=1)); corrupt.write(build_pid_query_frame()); corrupt.advance(.5)
        self.assertGreater(corrupt.stats["corrupted_bytes"],0)

    def _trace(self,config):
        transport=MuJoCoTransport(config); transport.reset_scenario(config,seed=123,initial_pitch_deg=4); transport.apply_push("forward",2)
        oracle=transport.advance(.35); return oracle[-1].pitch_deg,oracle[-1].speed_m_s,oracle[-1].left_torque_nm

    def test_slope_friction_payload_and_motor_dynamics_change_response(self):
        base=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=123)
        variants=[EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=123,slope_deg=5),
                  EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=123,friction=.4),
                  EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=123,payload_mass_kg=.5,payload_height_m=.3),
                  EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=123,motor_time_constant_s=.08)]
        baseline=self._trace(base)
        for variant in variants:
            with self.subTest(variant=variant): self.assertNotEqual(self._trace(variant),baseline)

    def test_simulation_speed_does_not_change_physics(self):
        a=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=77,simulation_speed=.5)
        b=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=77,simulation_speed=10)
        self.assertEqual(self._trace(a),self._trace(b))

    def test_fractional_encoder_motion_is_not_lost(self):
        transport=MuJoCoTransport(EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=5))
        plant=transport.plant; increment=0.2*2*math.pi/1320; observed=[]
        for index in range(1,21):
            plant.data.qpos[plant.left_qpos]=plant.encoder_origin_angles[0]+index*increment
            plant.data.qpos[plant.right_qpos]=plant.encoder_origin_angles[1]+index*increment
            mujoco.mj_forward(plant.model,plant.data)
            sensor=plant.sensor_inputs(); observed.append((sensor.encoder_left,sensor.encoder_right))
        self.assertEqual(sum(item[0] for item in observed),4)
        self.assertEqual(sum(item[1] for item in observed),4)

    def test_balance_recovery_uses_longitudinal_motion_without_large_pitch(self):
        config=EnvironmentConfig(imu_noise_deg=0,seed=321)
        result=ExperimentRunner(MuJoCoTransport(config)).run_trial(
            name="behavior",stage="balance",command="stop",duration_s=6,
            environment=config,seed=321)
        self.assertGreater(result.metrics.longitudinal_excursion_m,0.10)
        self.assertLess(result.metrics.longitudinal_excursion_m,0.30)
        self.assertLess(abs(result.metrics.longitudinal_displacement_m),0.05)
        self.assertLess(result.metrics.pitch_peak_deg,10.0)
        self.assertEqual(result.metrics.pwm_saturation_ratio,0.0)

    def test_transport_is_replaceable(self):
        self.assertTrue(issubclass(MuJoCoTransport,Transport)); self.assertTrue(issubclass(SerialTransport,Transport))

    def test_stage_trials_isolate_external_disturbance(self):
        config=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=88)
        transport=MuJoCoTransport(config); runner=ExperimentRunner(transport)
        with patch.object(transport,"apply_push",wraps=transport.apply_push) as push:
            balance=runner.run_trial(name="baseline",stage="balance",command="stop",duration_s=1.1,environment=config,seed=88)
            self.assertEqual(push.call_count,1)
        balance_summary=runner.build_llm_summary(balance,stage="balance",history=[],harness_limits={})
        self.assertTrue(balance_summary["trial_scenario"]["scheduled_disturbance"]["applied"])
        with patch.object(transport,"apply_push",wraps=transport.apply_push) as push:
            turn=runner.run_trial(name="baseline",stage="turn",command="right",duration_s=1.1,environment=config,seed=88)
            self.assertEqual(push.call_count,0)
        self.assertIn(4,{sample.command for sample in turn.telemetry})
        self.assertIn(1,{sample.command for sample in turn.telemetry})
        turn_summary=runner.build_llm_summary(turn,stage="turn",history=[],harness_limits={})
        self.assertFalse(turn_summary["trial_scenario"]["scheduled_disturbance"]["applied"])
        self.assertEqual([phase["command"] for phase in turn_summary["trial_scenario"]["phases"]],["right","forward"])


if __name__=="__main__":unittest.main()
