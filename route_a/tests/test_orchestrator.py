from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from route_a.deepseek_client import LLMError, MockLLM
from route_a.firmware_profile import DEFAULT_PID, EnvironmentConfig, PIDValues
from route_a.orchestrator import CycleState, RouteAOrchestrator, RunMode
from route_a.safety_harness import HarnessConfig, PID_BOUNDS, SafetyHarness
from route_a.transport import MuJoCoTransport


class FailingLLM:
    def propose(self,summary): raise LLMError("simulated 503")


class OrchestratorTests(unittest.TestCase):
    def _system(self,temp,*,minimum=1.0,llm=None):
        config=HarnessConfig(minimum_score_improvement=minimum,maximum_saturation_ratio=1.0)
        harness=SafetyHarness(config,Path(temp)/"state.json")
        orchestrator=RouteAOrchestrator(MuJoCoTransport(EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0)),llm or MockLLM(),harness)
        orchestrator.runner.runs_dir=Path(temp)/"runs"; return orchestrator

    def test_offline_mock_manual_candidate_is_not_applied_before_approval(self):
        with tempfile.TemporaryDirectory() as temp:
            system=self._system(temp,minimum=-1000); env=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=4)
            pending=system.begin_cycle(env,1.1); self.assertEqual(system.state,CycleState.AWAITING_REVIEW)
            before,_=system.runner.protocol.query_pid(); self.assertEqual(before,DEFAULT_PID)
            result=system.approve(1.1); self.assertEqual(result.decision,"accept")
            self.assertIsNotNone(system.last_trial); self.assertEqual(system.last_trial.name,"candidate")
            self.assertIs(system.last_evaluation,result)
            after,_=system.runner.protocol.query_pid(); self.assertEqual(after,system.harness.champion)
            self.assertEqual(system.harness.manual_successes,1)

    def test_reject_and_rollback_keep_champion(self):
        with tempfile.TemporaryDirectory() as temp:
            system=self._system(temp,minimum=1000); env=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=5)
            system.begin_cycle(env,1.1); result=system.approve(1.1); self.assertEqual(result.decision,"rollback")
            actual,_=system.runner.protocol.query_pid(); self.assertEqual(actual,DEFAULT_PID)
            system.begin_cycle(env,1.1); system.reject(); actual,_=system.runner.protocol.query_pid(); self.assertEqual(actual,DEFAULT_PID)

    def test_api_failure_does_not_change_champion(self):
        with tempfile.TemporaryDirectory() as temp:
            system=self._system(temp,llm=FailingLLM()); env=EnvironmentConfig(imu_noise_deg=0,imu_delay_ms=0,seed=6)
            with self.assertRaises(LLMError): system.begin_cycle(env,1.1)
            actual,_=system.runner.protocol.query_pid(); self.assertEqual(actual,DEFAULT_PID); self.assertEqual(system.harness.champion,DEFAULT_PID)

    def test_automatic_mode_is_gated(self):
        with tempfile.TemporaryDirectory() as temp:
            system=self._system(temp)
            with self.assertRaises(RuntimeError): system.set_mode(RunMode.AUTOMATIC)

    def test_reset_training_generates_broad_random_pid_and_clears_state(self):
        with tempfile.TemporaryDirectory() as temp:
            system=self._system(temp); environment=EnvironmentConfig(imu_noise_deg=0,seed=7)
            system.harness.manual_successes=4; system.harness.iterations=8
            system.harness.api_calls=6; system.harness.no_improvement=3
            system.history.append({"old":"round"})
            initial=system.reset_training(environment,random_seed=123)
            self.assertNotEqual(initial,DEFAULT_PID)
            for key,(low,high) in PID_BOUNDS.items():
                self.assertGreaterEqual(getattr(initial,key),low); self.assertLessEqual(getattr(initial,key),high)
            actual,_=system.runner.protocol.query_pid(); self.assertEqual(actual,initial)
            self.assertEqual(system.harness.champion,initial)
            self.assertEqual((system.harness.manual_successes,system.harness.iterations,
                              system.harness.api_calls,system.harness.no_improvement),(0,0,0,0))
            self.assertEqual(system.history,[]); self.assertEqual(system.state,CycleState.IDLE)
            self.assertEqual(system.mode,RunMode.MANUAL); self.assertEqual(system.stage,"balance")

    def test_environment_reset_preserves_pid_and_training_state(self):
        with tempfile.TemporaryDirectory() as temp:
            system=self._system(temp); chosen=PIDValues(88,44,55,22,11,9)
            system.runner.protocol.write_pid(chosen); system.harness.set_champion(chosen,manual_success=True)
            system.harness.iterations=7; system.harness.api_calls=4
            system.history.append({"kept":True}); system.stage="turn"; system.state=CycleState.COMPLETE
            system.transport.apply_push("forward",2); system.transport.advance(.2)
            system.reset_environment()
            oracle=system.transport.plant.observe_oracle()
            self.assertAlmostEqual(oracle.time_s,0.0); self.assertAlmostEqual(oracle.longitudinal_position_m,0.0)
            self.assertLess(abs(oracle.pitch_deg),2.0)
            self.assertTrue(all(abs(value)<1e-12 for value in system.transport.plant.data.qvel))
            actual,_=system.runner.protocol.query_pid(); self.assertEqual(actual,chosen)
            self.assertEqual(system.harness.champion,chosen); self.assertEqual(system.harness.manual_successes,1)
            self.assertEqual((system.harness.iterations,system.harness.api_calls),(7,4))
            self.assertEqual(system.history,[{"kept":True}]); self.assertEqual(system.stage,"turn")
            self.assertEqual(system.state,CycleState.COMPLETE)


if __name__=="__main__":unittest.main()
