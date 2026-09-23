package com.example.routeamobilemvp

import org.json.JSONObject
import kotlin.math.abs
import kotlin.math.sqrt

data class Metrics(val rmsAngle:Double,val peakAngle:Double,val rmsGyro:Double,val saturationRate:Double,val samples:Int)
data class LlmProposal(val decision:String,val stage:String,val candidate:Pid,val expectedEffect:String,val requestedTest:String,val confidence:Double)

object ExperimentLogic {
    fun metrics(xs:List<FastTelemetry>):Metrics {
        require(xs.isNotEmpty())
        val angleRms=sqrt(xs.sumOf{it.angle*it.angle}/xs.size)
        val gyroRms=sqrt(xs.sumOf{it.gyro*it.gyro}/xs.size)
        val saturation=xs.count{abs(it.ml)>=2595||abs(it.mr)>=2595}.toDouble()/xs.size
        return Metrics(angleRms,xs.maxOf{abs(it.angle)},gyroRms,saturation,xs.size)
    }
    fun proposal(json:String,base:Pid,maxPct:Double):LlmProposal {
        val root=JSONObject(json)
        require(root.keys().asSequence().toSet()==setOf("schema_version","decision","stage","candidate","expected_effect","requested_test","confidence"))
        require(root.getInt("schema_version")==1)
        val decision=root.getString("decision");require(decision in setOf("propose","hold","rollback"))
        val stage=root.getString("stage");require(stage=="balance")
        val requested=root.getString("requested_test");require(requested in setOf("balance_recovery","quiet_balance"))
        val effect=root.getString("expected_effect");require(effect.isNotBlank()&&effect.length<=500)
        val confidence=root.getDouble("confidence");require(confidence.isFinite()&&confidence in 0.0..1.0)
        val c=root.getJSONObject("candidate")
        require(c.keys().asSequence().toSet()==setOf("AP","AD","VP","VI","TP","TD"))
        val p=Pid(c.getDouble("AP"),c.getDouble("AD"),c.getDouble("VP"),c.getDouble("VI"),c.getDouble("TP"),c.getDouble("TD"))
        require(listOf(p.ap,p.ad,p.vp,p.vi,p.tp,p.td).all{it.isFinite()})
        require(abs(p.vp-base.vp)<0.005&&abs(p.vi-base.vi)<0.005&&abs(p.tp-base.tp)<0.005&&abs(p.td-base.td)<0.005)
        if(decision=="propose"){
            require(abs(p.ap-base.ap)*100/base.ap<=maxPct+1e-9)
            require(abs(p.ad-base.ad)*100/base.ad<=maxPct+1e-9)
        }
        return LlmProposal(decision,stage,p,effect,requested,confidence)
    }
    fun harness(base:Pid,m:Metrics):LlmProposal {
        var ap=base.ap;var ad=base.ad
        if(m.peakAngle>8.0)ap*=1.04
        if(m.rmsGyro>20.0)ad*=1.04
        return LlmProposal("propose","balance",base.copy(ap=ap,ad=ad),"Simulation MockLLM balance-stage rule.","balance_recovery",0.7)
    }
}

enum class TrialState{IDLE,BASELINE,REVIEW,PREPARING,PREPARED,APPLYING,TRIAL}
class TrialMachine{var state=TrialState.IDLE;private set;var id=0L;private set
    fun baseline(){check(state==TrialState.IDLE);state=TrialState.BASELINE}
    fun reviewed(){check(state==TrialState.BASELINE);state=TrialState.REVIEW}
    fun prepareSent(newId:Long){check(state==TrialState.REVIEW);id=newId;state=TrialState.PREPARING}
    fun prepared(){check(state==TrialState.PREPARING);state=TrialState.PREPARED}
    fun applySent(){check(state==TrialState.PREPARED);state=TrialState.APPLYING}
    fun applied(){check(state==TrialState.APPLYING);state=TrialState.TRIAL}
    fun finish(){check(state in setOf(TrialState.TRIAL,TrialState.APPLYING,TrialState.PREPARED,TrialState.PREPARING));state=TrialState.IDLE;id=0}
}
