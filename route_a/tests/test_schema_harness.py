from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from route_a.firmware_profile import DEFAULT_PID, PIDValues
from route_a.metrics import PerformanceMetrics
from route_a.safety_harness import HarnessConfig, SafetyHarness
from route_a.schemas import SchemaValidationError, parse_proposal_json


def proposal(candidate=None,stage="balance",decision="propose",extra=None):
    data={"schema_version":1,"decision":decision,"stage":stage,"candidate":(candidate or DEFAULT_PID).as_dict(),
          "expected_effect":"test","requested_test":"balance_recovery" if stage=="balance" else ("velocity_step" if stage=="velocity" else "turn_step"),"confidence":.7}
    if extra:data.update(extra)
    return parse_proposal_json(json.dumps(data))


def metrics(score_case="base"):
    values=dict(pitch_rms_deg=2,pitch_peak_deg=5,pitch_rate_rms_deg_s=10,settling_time_s=1,
        steady_state_pitch_error_deg=.5,speed_tracking_rmse_m_s=.1,speed_overshoot_m_s=.05,
        yaw_tracking_rmse_deg_s=5,wheel_mismatch_m_s=.01,pwm_saturation_ratio=.1,
        control_effort_rms_pwm=500,fall_count=0,protection_trigger_count=0,
        telemetry_loss_ratio=0,telemetry_staleness_s=.1,sample_count=20)
    if score_case=="better": values.update(pitch_rms_deg=.5,pitch_peak_deg=2,control_effort_rms_pwm=300)
    if score_case=="worse": values.update(pitch_rms_deg=20,pitch_peak_deg=39)
    if score_case=="fall": values.update(fall_count=1)
    if score_case=="saturation": values.update(pwm_saturation_ratio=.9)
    if score_case=="timeout": values.update(telemetry_staleness_s=1.0,telemetry_loss_ratio=.5)
    return PerformanceMetrics(**values)


class SchemaHarnessTests(unittest.TestCase):
    def test_schema_missing_extra_type_and_non_finite(self):
        base=json.loads(json.dumps(proposal().as_dict()))
        for mutate in (
            lambda d:d.pop("confidence"), lambda d:d.update(extra=1),
            lambda d:d.update(confidence="high"), lambda d:d["candidate"].update(AP="96")):
            data=json.loads(json.dumps(base)); mutate(data)
            with self.assertRaises(SchemaValidationError): parse_proposal_json(json.dumps(data))
        raw=json.dumps(base).replace('"confidence": 0.7','"confidence": NaN')
        with self.assertRaises(SchemaValidationError): parse_proposal_json(raw)

    def test_bounds_stage_lock_and_deterministic_shrink(self):
        harness=SafetyHarness(); self.assertFalse(harness.validate_candidate(proposal(PIDValues(500,48,62,31,14,20)),"balance").accepted)
        self.assertFalse(harness.validate_candidate(proposal(PIDValues(96,48,63,31,14,20)),"balance").accepted)
        result=harness.validate_candidate(proposal(PIDValues(120,60,62,31,14,20)),"balance")
        self.assertTrue(result.accepted); self.assertTrue(result.shrunk)
        self.assertLessEqual(abs(result.candidate.AP-96),5)

    def test_candidate_is_quantized_to_protocol_precision(self):
        result=SafetyHarness().validate_candidate(
            proposal(PIDValues(96.044,48.006,62,31,14,20)),"balance")
        self.assertTrue(result.accepted)
        self.assertEqual(result.candidate.AP,96.04)
        self.assertEqual(result.candidate.AD,48.01)

    def test_bootstrap_allows_large_balance_rescue_but_keeps_stage_lock(self):
        candidate=proposal(PIDValues(130,70,62,31,14,20))
        normal=SafetyHarness().validate_candidate(candidate,"balance")
        bootstrap=SafetyHarness().validate_candidate(candidate,"balance",bootstrap=True)
        self.assertEqual(normal.candidate.AP,101.0)
        self.assertEqual(bootstrap.candidate.AP,130.0)
        self.assertEqual(bootstrap.candidate.AD,70.0)
        invalid=proposal(PIDValues(130,70,80,31,14,20))
        self.assertFalse(SafetyHarness().validate_candidate(invalid,"balance",bootstrap=True).accepted)

    def test_safety_gates_and_scoring_decisions(self):
        harness=SafetyHarness()
        self.assertEqual(harness.evaluate(metrics(),metrics("better"),"balance").decision,"accept")
        self.assertEqual(harness.evaluate(metrics(),metrics("worse"),"balance").decision,"rollback")
        for case in ("fall","saturation","timeout"):
            self.assertEqual(harness.evaluate(metrics(),metrics(case),"balance").decision,"rollback")

    def test_champion_is_atomically_persisted(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"champion.json"; harness=SafetyHarness(state_path=path)
            chosen=PIDValues(97,49,62,31,14,20); harness.set_champion(chosen,manual_success=True)
            restored=SafetyHarness(state_path=path); self.assertEqual(restored.champion,chosen); self.assertEqual(restored.manual_successes,1)

    def test_automatic_gate_budget_iteration_and_no_improvement(self):
        config=HarnessConfig(automatic_unlock_manual_successes=3,maximum_iterations=1,api_budget=1,no_improvement_limit=1)
        harness=SafetyHarness(config); self.assertFalse(harness.automatic_unlocked)
        for _ in range(3): harness.set_champion(DEFAULT_PID,manual_success=True)
        self.assertTrue(harness.automatic_unlocked)
        harness.iterations=1; self.assertFalse(harness.can_continue()[0])


if __name__=="__main__":unittest.main()
