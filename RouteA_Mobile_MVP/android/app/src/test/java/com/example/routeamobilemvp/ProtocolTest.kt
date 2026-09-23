package com.example.routeamobilemvp
import org.junit.Assert.*
import org.junit.Test
class ProtocolTest{
 @Test fun crcVector(){assertEquals(0x29b1,Wire.crc16("123456789".toByteArray()))}
 @Test fun fragmentsAndCoalescing(){val p=StreamParser();val a=Wire.getPid();val b=Wire.command("HB",4);assertTrue(p.feed(a.take(4).toByteArray()).isEmpty());assertEquals(listOf(a,b),p.feed((a.drop(4)+b).toByteArray()))}
 @Test fun pidStatus(){val f=Wire.frame("P1,PID,96.00,48.00,62.00,31.00,17.00,20.00,0,1,1");val s=Wire.parsePidStatus(f)!!;assertEquals(17.0,s.pid.tp,0.0);assertTrue(s.trainingLocked);assertTrue(s.balanceStarted)}
 @Test fun manualAndTrainingFrames(){assertTrue(Wire.verify(Wire.manualSet(Pid(96.0,48.0,62.0,31.0,17.0,20.0))));assertTrue(Wire.verify(Wire.training("START")))}
 @Test fun motorDiagnosticAndCcrTelemetry(){assertTrue(Wire.verify(Wire.motorDiag("L+")));val t=Wire.parseFast(Wire.frame("T1,F,200,A0.10,G1.0,EL2,ER0,ML0,MR0,S0,L0,R0,C10,C21500,C30,C40"))!!;assertEquals(1500,t.c2);assertEquals(0,t.c3)}
 @Test fun livePidAndTrainingTelemetry(){val s=Wire.parseSlow(Wire.frame("T1,S,1000,V11.40,B12,V-3,T0,AP96.00,AD48.00,VP62.00,VI31.00,TP17.00,TD20.00,D0,L1,R1,RA1.5-TEST-MODE"))!!;assertEquals(62.0,s.pid.vp,0.0);assertEquals(20.0,s.pid.td,0.0);assertTrue(s.trainingLocked);assertTrue(s.balanceStarted);assertEquals("RA1.5-TEST-MODE",s.firmware)}
 @Test fun simulationSchemaAndLimits(){val base=Pid(96.0,48.0,62.0,31.0,17.0,20.0);val json="""{"schema_version":1,"decision":"propose","stage":"balance","candidate":{"AP":97,"AD":48.5,"VP":62,"VI":31,"TP":17,"TD":20},"expected_effect":"test","requested_test":"balance_recovery","confidence":0.7}""";assertEquals(97.0,ExperimentLogic.proposal(json,base,2.0).candidate.ap,0.0);val bad=json.replace("\"VP\":62","\"VP\":63");assertThrows(IllegalArgumentException::class.java){ExperimentLogic.proposal(bad,base,2.0)}}
 @Test fun stateMachine(){val m=TrialMachine();m.baseline();m.reviewed();m.prepareSent(7);m.prepared();m.applySent();m.applied();m.finish();assertEquals(TrialState.IDLE,m.state)}
}
