package com.example.routeamobilemvp

data class Pid(val ap:Double,val ad:Double,val vp:Double,val vi:Double,val tp:Double,val td:Double)
data class PidStatus(val pid:Pid,val state:Int,val trainingLocked:Boolean,val balanceStarted:Boolean)
data class FastTelemetry(val ms:Long,val angle:Double,val gyro:Double,val el:Int,val er:Int,val ml:Int,val mr:Int,val state:Int,val trainingLocked:Boolean=false,val balanceStarted:Boolean=false,val c1:Int=0,val c2:Int=0,val c3:Int=0,val c4:Int=0)
data class SlowTelemetry(val ms:Long,val battery:Double,val balancePwm:Int,val velocityPwm:Int,val turnPwm:Int,val pid:Pid,val dropped:Int,val trainingLocked:Boolean,val balanceStarted:Boolean,val firmware:String)

object Wire {
    fun crc16(bytes:ByteArray):Int { var crc=0xffff; bytes.forEach { v -> crc=crc xor ((v.toInt() and 255) shl 8); repeat(8){crc=if(crc and 0x8000!=0)((crc shl 1) xor 0x1021) and 0xffff else (crc shl 1) and 0xffff} }; return crc }
    fun frame(body:String):String="$${body},C${crc16(body.toByteArray()).toString(16).uppercase().padStart(4,'0')}#"
    fun verify(raw:String):Boolean { if(!raw.startsWith('$')||!raw.endsWith('#'))return false; val p=raw.lastIndexOf(",C"); if(p<0||raw.length-p!=7)return false; return raw.substring(p+2,p+6).toIntOrNull(16)==crc16(raw.substring(1,p).toByteArray()) }
    fun prepare(id:Long,ttl:Long,p:Pid)=frame("P1,PREP,$id,$ttl,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f".format(java.util.Locale.US,p.ap,p.ad,p.vp,p.vi,p.tp,p.td))
    fun command(name:String,id:Long)=frame("P1,$name,$id")
    fun training(action:String)=frame("P1,TRAIN,$action")
    fun manualSet(p:Pid)=frame("P1,MSET,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f".format(java.util.Locale.US,p.ap,p.ad,p.vp,p.vi,p.tp,p.td))
    fun motorDiag(action:String)=frame("P1,DIAG,$action")
    fun getPid()=frame("P1,GET")
    fun parsePidStatus(raw:String):PidStatus? { if(!verify(raw))return null;val x=raw.substring(1,raw.lastIndexOf(",C")).split(',');if(x.size<11||x[0]!="P1"||x[1]!="PID")return null;return runCatching{PidStatus(Pid(x[2].toDouble(),x[3].toDouble(),x[4].toDouble(),x[5].toDouble(),x[6].toDouble(),x[7].toDouble()),x[8].toInt(),x[9]=="1",x[10]=="1")}.getOrNull() }
    fun parsePid(raw:String):Pid?=parsePidStatus(raw)?.pid
    fun parseFast(raw:String):FastTelemetry? { if(!verify(raw))return null; val x=raw.substring(1,raw.lastIndexOf(",C")).split(','); if(x.size<10||x[0]!="T1"||x[1]!="F")return null; return runCatching{FastTelemetry(x[2].toLong(),x[3].drop(1).toDouble(),x[4].drop(1).toDouble(),x[5].drop(2).toInt(),x[6].drop(2).toInt(),x[7].drop(2).toInt(),x[8].drop(2).toInt(),x[9].drop(1).toInt(),x.getOrNull(10)?.drop(1)=="1",x.getOrNull(11)?.drop(1)=="1",x.getOrNull(12)?.drop(2)?.toIntOrNull()?:0,x.getOrNull(13)?.drop(2)?.toIntOrNull()?:0,x.getOrNull(14)?.drop(2)?.toIntOrNull()?:0,x.getOrNull(15)?.drop(2)?.toIntOrNull()?:0)}.getOrNull() }
    fun parseSlow(raw:String):SlowTelemetry? { if(!verify(raw))return null;val x=raw.substring(1,raw.lastIndexOf(",C")).split(',');if(x.size<17||x[0]!="T1"||x[1]!="S")return null;return runCatching{SlowTelemetry(x[2].toLong(),x[3].drop(1).toDouble(),x[4].drop(1).toInt(),x[5].drop(1).toInt(),x[6].drop(1).toInt(),Pid(x[7].drop(2).toDouble(),x[8].drop(2).toDouble(),x[9].drop(2).toDouble(),x[10].drop(2).toDouble(),x[11].drop(2).toDouble(),x[12].drop(2).toDouble()),x[13].drop(1).toInt(),x[14].drop(1)=="1",x[15].drop(1)=="1",x[16])}.getOrNull() }
}

class StreamParser(private val max:Int=256) {
    private val b=StringBuilder()
    fun feed(bytes:ByteArray):List<String>{val out=mutableListOf<String>(); bytes.forEach{v->val c=(v.toInt() and 255).toChar();if(c=='$'){b.clear();b.append(c)}else if(b.isNotEmpty()){b.append(c);if(b.length>max)b.clear() else if(c=='#'){out+=b.toString();b.clear()}}};return out}
}
