from __future__ import annotations
import json, urllib.request, urllib.error
class OllamaAdvisor:
    """Local advisory client. Its outputs are parsed as data and validated by callers."""
    def __init__(self, model="llama3.2", url="http://127.0.0.1:11434/api/generate"):self.model=model;self.url=url
    def propose(self,prompt):
        payload=json.dumps({"model":self.model,"prompt":prompt+" Return ONLY a JSON object with every requested key.","format":"json","stream":False}).encode()
        try:
            with urllib.request.urlopen(urllib.request.Request(self.url,payload,{"Content-Type":"application/json"}),timeout=30) as r:
                response=json.loads(r.read())["response"]
                return json.loads(response)
        except urllib.error.HTTPError as exc:
            if exc.code==404:return {"error":f"HTTP 404 at {self.url}. This is not Ollama's generate endpoint. Start Ollama, then use http://localhost:11434/api/generate.","endpoint":self.url}
            return {"error":f"Ollama HTTP {exc.code}: {exc.reason}","endpoint":self.url}
        except json.JSONDecodeError as exc:return {"error":f"Ollama returned invalid JSON despite JSON mode: {exc}"}
        except Exception as exc:return {"error":f"Could not reach local Ollama at {self.url}: {exc}"}
    @staticmethod
    def pid_candidate(obj):
        if not isinstance(obj,dict):raise ValueError("proposal must be object")
        result={k:float(obj[k]) for k in ("kp","ki","kd")}
        if any(not 0<=x<=500 for x in result.values()):raise ValueError("PID limits")
        return result
