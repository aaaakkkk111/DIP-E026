from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from route_a.deepseek_client import DeepSeekClient


class _Response:
    def __init__(self, payload): self.payload=payload
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def read(self): return json.dumps(self.payload).encode("utf-8")


class DeepSeekClientTests(unittest.TestCase):
    def test_empty_json_content_is_retried_without_changing_schema(self):
        proposal={"schema_version":1,"decision":"propose","stage":"balance",
            "candidate":{"AP":99.0,"AD":49.0,"VP":62.0,"VI":31.0,"TP":14.0,"TD":20.0},
            "expected_effect":"small balance adjustment","requested_test":"balance_recovery","confidence":0.7}
        empty={"model":"deepseek-v4-flash","choices":[{"message":{"content":""},"finish_reason":"stop"}]}
        valid={"model":"deepseek-v4-flash","choices":[{"message":{"content":json.dumps(proposal)},"finish_reason":"stop"}],"usage":{"total_tokens":12}}
        with tempfile.TemporaryDirectory() as temp:
            env=Path(temp)/"route_a.env"
            env.write_text("DEEPSEEK_API_KEY=test-only\nDEEPSEEK_MODEL=deepseek-v4-flash\n",encoding="utf-8")
            client=DeepSeekClient(env_path=env,max_retries=2)
            with patch("route_a.deepseek_client.urllib.request.urlopen",side_effect=[_Response(empty),_Response(valid)]) as call, patch("route_a.deepseek_client.time.sleep"):
                result=client.propose({"current_pid":proposal["candidate"],"stage":"balance"})
        self.assertEqual(call.call_count,2)
        self.assertEqual(result.proposal.candidate.AP,99.0)
        self.assertEqual(result.usage["total_tokens"],12)

    def test_invalid_schema_is_explained_and_retried(self):
        invalid={"schema_version":1,"decision":"propose","stage":"balance",
            "candidate":{"AP":99.0,"AD":49.0,"VP":62.0,"VI":31.0,"TP":14.0,"TD":20.0},
            "expected_effect":"","requested_test":"balance_recovery","confidence":0.7}
        valid=dict(invalid,expected_effect="reduce pitch oscillation")
        responses=[_Response({"choices":[{"message":{"content":json.dumps(invalid)},"finish_reason":"stop"}]}),
                   _Response({"choices":[{"message":{"content":json.dumps(valid)},"finish_reason":"stop"}]})]
        with tempfile.TemporaryDirectory() as temp:
            env=Path(temp)/"route_a.env"; env.write_text("DEEPSEEK_API_KEY=test-only\n",encoding="utf-8")
            client=DeepSeekClient(env_path=env,max_retries=2)
            captured=[]
            def respond(request,timeout):
                captured.append(json.loads(request.data.decode("utf-8"))); return responses[len(captured)-1]
            with patch("route_a.deepseek_client.urllib.request.urlopen",side_effect=respond), patch("route_a.deepseek_client.time.sleep"):
                result=client.propose({"stage":"balance"})
        self.assertEqual(result.proposal.expected_effect,"reduce pitch oscillation")
        self.assertIn("expected_effect must be a non-empty string",captured[1]["messages"][1]["content"])

    def test_bootstrap_hold_is_reprompted_as_rescue_proposal(self):
        base={"schema_version":1,"stage":"balance",
            "candidate":{"AP":40.0,"AD":20.0,"VP":62.0,"VI":31.0,"TP":14.0,"TD":20.0},
            "requested_test":"balance_recovery","confidence":0.6}
        hold=dict(base,decision="hold",expected_effect="baseline fell")
        rescue=dict(base,decision="propose",expected_effect="large AP/AD recovery step")
        responses=[_Response({"choices":[{"message":{"content":json.dumps(hold)},"finish_reason":"stop"}]}),
                   _Response({"choices":[{"message":{"content":json.dumps(rescue)},"finish_reason":"stop"}]})]
        with tempfile.TemporaryDirectory() as temp:
            env=Path(temp)/"route_a.env"; env.write_text("DEEPSEEK_API_KEY=test-only\n",encoding="utf-8")
            client=DeepSeekClient(env_path=env,max_retries=2); captured=[]
            def respond(request,timeout):
                captured.append(json.loads(request.data.decode("utf-8"))); return responses[len(captured)-1]
            with patch("route_a.deepseek_client.urllib.request.urlopen",side_effect=respond), patch("route_a.deepseek_client.time.sleep"):
                result=client.propose({"stage":"balance","bootstrap_recovery":{"required":True}})
        self.assertEqual(result.proposal.decision,"propose"); self.assertEqual(len(captured),2)
        self.assertIn("bootstrap recovery is required",captured[1]["messages"][1]["content"])


if __name__=="__main__": unittest.main()
