package com.example.routeamobilemvp

import org.json.JSONArray
import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL

interface TuningAdvisor { fun propose(apiKey:String,baseUrl:String,model:String,pid:Pid,m:Metrics,maxPct:Double):String }
class DeepSeekClient: TuningAdvisor {
    override fun propose(apiKey:String,baseUrl:String,model:String,pid:Pid,m:Metrics,maxPct:Double):String {
        val system="""PROMPT_VERSION: route-a-pid-v4-mobile
You are a PID tuning adviser for a two-wheel balance robot. The deterministic Harness owns validation, acceptance and rollback. Return exactly one JSON object with exactly: schema_version, decision, stage, candidate, expected_effect, requested_test, confidence. decision is propose, hold, or rollback. stage is balance. candidate always contains all six numeric fields AP, AD, VP, VI, TP, TD. Change only AP/AD and keep VP/VI/TP/TD exactly equal to the champion. Prefer explainable changes within harness_limits. requested_test is balance_recovery or quiet_balance. Never output PWM, motor commands, safety changes, Markdown, or claim acceptance."""
        val current=JSONObject().put("AP",pid.ap).put("AD",pid.ad).put("VP",pid.vp).put("VI",pid.vi).put("TP",pid.tp).put("TD",pid.td)
        val metrics=JSONObject().put("pitch_rms_deg",m.rmsAngle).put("pitch_peak_deg",m.peakAngle).put("pitch_rate_rms_deg_s",m.rmsGyro).put("pwm_saturation_fraction",m.saturationRate).put("sample_count",m.samples)
        val prompt=JSONObject().put("schema_version",1).put("current_pid",current).put("stage","balance").put("trial_scenario",JSONObject().put("name","balance_recovery").put("completed",true).put("motion_command","stop")).put("data_quality",JSONObject().put("complete",m.samples>=10)).put("metrics",metrics).put("recent_history",JSONArray()).put("harness_limits",JSONObject().put("max_relative_change",maxPct/100.0).put("firmware_hard_cap_percent",5.0)).toString()
        val body=JSONObject().put("model",model).put("response_format",JSONObject().put("type","json_object")).put("messages",JSONArray().put(JSONObject().put("role","system").put("content",system)).put(JSONObject().put("role","user").put("content",prompt)))
        val c=(URL(baseUrl.trimEnd('/')+"/chat/completions").openConnection() as HttpURLConnection); c.requestMethod="POST";c.connectTimeout=15000;c.readTimeout=30000;c.doOutput=true;c.setRequestProperty("Authorization","Bearer $apiKey");c.setRequestProperty("Content-Type","application/json");c.outputStream.use{it.write(body.toString().toByteArray())};if(c.responseCode !in 200..299)throw IllegalStateException("HTTP ${c.responseCode}");val root=JSONObject(c.inputStream.bufferedReader().readText());return root.getJSONArray("choices").getJSONObject(0).getJSONObject("message").getString("content")
    }
}
