"""Strict local schema validation for the LLM boundary."""

from __future__ import annotations

import json
from dataclasses import dataclass
from math import isfinite
from typing import Any

from .firmware_profile import PIDValues


DECISIONS = {"propose", "hold", "rollback"}
STAGES = {"balance", "velocity", "turn"}
REQUESTED_TESTS = {"balance_recovery", "velocity_step", "turn_step", "quiet_balance"}
PROPOSAL_FIELDS = {"schema_version", "decision", "stage", "candidate", "expected_effect", "requested_test", "confidence"}


class SchemaValidationError(ValueError):
    pass


@dataclass(frozen=True)
class LLMProposal:
    schema_version: int
    decision: str
    stage: str
    candidate: PIDValues
    expected_effect: str
    requested_test: str
    confidence: float

    def as_dict(self) -> dict[str, Any]:
        return {"schema_version": self.schema_version, "decision": self.decision,
                "stage": self.stage, "candidate": self.candidate.as_dict(),
                "expected_effect": self.expected_effect, "requested_test": self.requested_test,
                "confidence": self.confidence}


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise SchemaValidationError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def parse_proposal_json(raw: str) -> LLMProposal:
    if not isinstance(raw, str) or not raw.strip():
        raise SchemaValidationError("empty LLM response")
    try:
        data = json.loads(raw, object_pairs_hook=_no_duplicates,
                          parse_constant=lambda x: (_ for _ in ()).throw(SchemaValidationError(f"non-finite JSON number: {x}")))
    except SchemaValidationError:
        raise
    except (json.JSONDecodeError, TypeError) as exc:
        raise SchemaValidationError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise SchemaValidationError("proposal must be a JSON object")
    if set(data) != PROPOSAL_FIELDS:
        raise SchemaValidationError(f"proposal fields mismatch; missing={sorted(PROPOSAL_FIELDS-set(data))}, extra={sorted(set(data)-PROPOSAL_FIELDS)}")
    if type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise SchemaValidationError("schema_version must be integer 1")
    if data["decision"] not in DECISIONS or data["stage"] not in STAGES:
        raise SchemaValidationError("invalid decision or stage")
    if data["requested_test"] not in REQUESTED_TESTS:
        raise SchemaValidationError("invalid requested_test")
    if not isinstance(data["expected_effect"], str) or not (1 <= len(data["expected_effect"]) <= 500):
        raise SchemaValidationError("expected_effect must be a non-empty string up to 500 chars")
    confidence = data["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not isfinite(float(confidence)):
        raise SchemaValidationError("confidence must be finite")
    if not 0.0 <= float(confidence) <= 1.0:
        raise SchemaValidationError("confidence must be in [0, 1]")
    if not isinstance(data["candidate"], dict):
        raise SchemaValidationError("candidate must be an object")
    try:
        candidate = PIDValues.from_mapping(data["candidate"], strict=True)
    except (TypeError, ValueError, KeyError) as exc:
        raise SchemaValidationError(str(exc)) from exc
    return LLMProposal(1, data["decision"], data["stage"], candidate,
                       data["expected_effect"], data["requested_test"], float(confidence))
