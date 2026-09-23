package com.example.routeamobilemvp

import android.Manifest
import android.annotation.SuppressLint
import android.app.AlertDialog
import android.bluetooth.BluetoothDevice
import android.os.*
import android.view.View
import android.view.ViewGroup.LayoutParams.MATCH_PARENT
import android.view.ViewGroup.LayoutParams.WRAP_CONTENT
import android.view.WindowManager
import android.widget.*
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import java.util.concurrent.Executors

class MainActivity:AppCompatActivity(),BleClient.Events{
    private lateinit var ble:BleClient
    private val parser=StreamParser()
    private val devices=linkedMapOf<String,BluetoothDevice>()
    private lateinit var logView:TextView
    private lateinit var live:TextView
    private lateinit var pidLive:TextView
    private lateinit var trainingStatus:TextView
    private lateinit var chart:TelemetryChartView
    private lateinit var deviceBox:Spinner
    private lateinit var manualButton:Button
    private lateinit var startTrainingButton:Button
    private lateinit var stopTrainingButton:Button
    private val diagButtons=mutableListOf<Button>()
    private val pidFields=linkedMapOf<String,EditText>()
    private val samples=mutableListOf<FastTelemetry>()
    private var pid=Pid(96.0,48.0,62.0,31.0,17.0,20.0)
    private var pidKnown=false
    private var trainingLocked=false
    private var balanceStarted=false
    private var firmwareState=0
    private var firmwareVersion="等待固件"
    private var proposal:LlmProposal?=null
    private val machine=TrialMachine()
    private val handler=Handler(Looper.getMainLooper())
    private val worker=Executors.newSingleThreadExecutor()
    private val ttl=8000L
    private val permissions=registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()){}

    private fun button(label:String,fn:()->Unit)=Button(this).apply{text=label;textSize=16f;setOnClickListener{fn()}}
    private fun line(root:LinearLayout,vararg views:View){root.addView(LinearLayout(this).apply{orientation=LinearLayout.HORIZONTAL;views.forEach{addView(it,LinearLayout.LayoutParams(0,WRAP_CONTENT,1f))}})}
    private fun addPidField(root:LinearLayout,key:String,value:Double){val e=EditText(this).apply{setText("%.2f".format(java.util.Locale.US,value));inputType=2 or 8192};pidFields[key]=e;line(root,TextView(this).apply{text=key;textSize=17f},e)}
    private fun log(s:String)=runOnUiThread{logView.append("\n$s")}

    @SuppressLint("SetTextI18n")
    override fun onCreate(savedInstanceState:Bundle?){
        super.onCreate(savedInstanceState);window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        ble=BleClient(this,this)
        if(Build.VERSION.SDK_INT>=31)permissions.launch(arrayOf(Manifest.permission.BLUETOOTH_SCAN,Manifest.permission.BLUETOOTH_CONNECT))
        else permissions.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
        val root=LinearLayout(this).apply{orientation=LinearLayout.VERTICAL;setPadding(18,18,18,18)}
        logView=TextView(this).apply{setTextIsSelectable(true);text="Route A Android v1.4 · 仅用于模式21 TEST MODE"}
        deviceBox=Spinner(this)
        root.addView(TextView(this).apply{text="蓝牙设备（官方 JDY-23：FFE0 / FFE1）";textSize=18f})
        root.addView(deviceBox,LinearLayout.LayoutParams(MATCH_PARENT,WRAP_CONTENT))
        line(root,button("扫描"){ble.scan()},button("连接"){(deviceBox.selectedItem as?String)?.let{devices[it]?.let(ble::connect)}},button("断开"){ble.send(Wire.command("STOP",machine.id));handler.postDelayed({ble.disconnect()},150)})
        trainingStatus=TextView(this).apply{text="等待 TEST MODE 状态";textSize=19f}
        root.addView(trainingStatus)
        live=TextView(this).apply{text="等待遥测";textSize=18f};root.addView(live)
        pidLive=TextView(this).apply{text="实时 PID：等待固件同步";textSize=19f}
        root.addView(pidLive)
        chart=TelemetryChartView(this);root.addView(chart,LinearLayout.LayoutParams(MATCH_PARENT,280))

        root.addView(TextView(this).apply{text="架空电机诊断（仅在启动平衡前；每次约0.6秒）";textSize=19f})
        val leftForward=button("左轮 +"){motorDiag("L+")}
        val leftReverse=button("左轮 −"){motorDiag("L-")}
        val rightForward=button("右轮 +"){motorDiag("R+")}
        val rightReverse=button("右轮 −"){motorDiag("R-")}
        val diagStop=button("诊断停止"){motorDiag("STOP")}
        diagButtons.addAll(listOf(leftForward,leftReverse,rightForward,rightReverse,diagStop))
        line(root,leftForward,leftReverse)
        line(root,rightForward,rightReverse)
        root.addView(diagStop,LinearLayout.LayoutParams(MATCH_PARENT,WRAP_CONTENT))

        root.addView(TextView(this).apply{text="TEST MODE 人工设定值（上方显示固件实际值）";textSize=19f})
        addPidField(root,"AP",96.0);addPidField(root,"AD",48.0);addPidField(root,"VP",62.0)
        addPidField(root,"VI",31.0);addPidField(root,"TP",17.0);addPidField(root,"TD",20.0)
        manualButton=button("应用人工 PID"){applyManualPid()}
        line(root,button("读取 PID"){ble.send(Wire.getPid())},manualButton)
        startTrainingButton=button("开始训练并锁定人工调参"){startTraining()}
        stopTrainingButton=button("停止训练并解锁"){ble.send(Wire.training("STOP"))}
        line(root,startTrainingButton,stopTrainingButton)

        line(root,button("采集基线"){startBaseline()},button("结束基线"){finishBaseline()})
        line(root,button("Simulation Harness 建议"){harnessProposal()},button("DeepSeek 建议"){requestLlm()},button("设置"){settings()})
        line(root,button("审核并 PREPARE"){prepare()},button("APPLY 候选"){applyCandidate()})
        line(root,button("ACCEPT"){acceptCandidate()},button("ROLLBACK"){rollbackCandidate()})
        root.addView(TextView(this).apply{text="先架空车轮。训练开始后手机和固件都会拒绝人工调参；停止训练会先回滚未完成候选。";textSize=17f})
        root.addView(logView)
        val scroll=ScrollView(this).apply{addView(root)}
        ViewCompat.setOnApplyWindowInsetsListener(scroll){view,insets->
            val bars=insets.getInsets(WindowInsetsCompat.Type.systemBars())
            view.setPadding(0,bars.top,0,bars.bottom)
            insets
        }
        setContentView(scroll)
        ViewCompat.requestApplyInsets(scroll)
        setTrainingLock(false)
    }

    private fun pidFromFields():Pid?=runCatching{Pid(pidFields["AP"]!!.text.toString().toDouble(),pidFields["AD"]!!.text.toString().toDouble(),pidFields["VP"]!!.text.toString().toDouble(),pidFields["VI"]!!.text.toString().toDouble(),pidFields["TP"]!!.text.toString().toDouble(),pidFields["TD"]!!.text.toString().toDouble())}.getOrNull()
    private fun showPid(p:Pid,source:String){listOf("AP" to p.ap,"AD" to p.ad,"VP" to p.vp,"VI" to p.vi,"TP" to p.tp,"TD" to p.td).forEach{(k,v)->if(pidFields[k]?.hasFocus()!=true)pidFields[k]?.setText("%.2f".format(java.util.Locale.US,v))};runOnUiThread{pidLive.text="实时 PID [$source]  AP %.2f  AD %.2f\nVP %.2f  VI %.2f  TP %.2f  TD %.2f".format(java.util.Locale.US,p.ap,p.ad,p.vp,p.vi,p.tp,p.td)}}
    private fun firmwareStateName()=when(firmwareState){0->"IDLE/基线";1->"PREPARED/候选待应用";2->"TRIAL/候选试验中";else->"未知($firmwareState)"}
    private fun updateTrainingStatus()=runOnUiThread{trainingStatus.text="模式21 TEST MODE · $firmwareVersion\n平衡：${if(balanceStarted)"已启动" else "未启动"}　训练锁：${if(trainingLocked)"已锁定" else "未锁定"}\n固件：${firmwareStateName()}　手机事务：${machine.state}"}
    private fun setTrainingLock(locked:Boolean){trainingLocked=locked;runOnUiThread{pidFields.values.forEach{it.isEnabled=!locked};manualButton.isEnabled=!locked;startTrainingButton.isEnabled=!locked;stopTrainingButton.isEnabled=locked;diagButtons.forEach{it.isEnabled=!locked&&!balanceStarted}};updateTrainingStatus()}
    private fun motorDiag(action:String){if(balanceStarted)return log("电机诊断被拒绝：请重新上电，在启动平衡前架空测试");if(trainingLocked)return log("电机诊断被训练锁拒绝");ble.send(Wire.motorDiag(action));log("电机诊断 $action：观察对应车轮、EL/ER 与 C1..C4")}
    private fun applyManualPid(){if(trainingLocked)return log("training lock rejects manual tuning");val p=pidFromFields()?:return log("PID must be six finite numbers");if(listOf(p.ap,p.ad,p.vp,p.vi,p.tp,p.td).any{!it.isFinite()})return log("PID must be finite");ble.send(Wire.manualSet(p));log("manual PID sent; waiting for firmware ACK and readback")}
    private fun startTraining(){if(!pidKnown)return log("read PID before training");if(!balanceStarted)return log("press the car start key and wait for balance ready");if(machine.state!=TrialState.IDLE)return log("finish the active candidate first");ble.send(Wire.training("START"));log("TRAIN START sent; waiting for firmware lock ACK")}
    private fun startBaseline(){if(!trainingLocked)return log("start training first; manual tuning must be locked");if(machine.state!=TrialState.IDLE)return log("experiment is busy");samples.clear();machine.baseline();log("baseline started")}
    private fun finishBaseline(){if(machine.state!=TrialState.BASELINE)return log("baseline is not running");if(samples.isEmpty())return log("no telemetry samples");machine.reviewed();log("metrics=${ExperimentLogic.metrics(samples)}")}
    private fun harnessProposal(){if(machine.state!=TrialState.REVIEW)return log("collect and finish baseline first");proposal=ExperimentLogic.harness(pid,ExperimentLogic.metrics(samples));log("Simulation MockLLM proposal=$proposal")}
    private fun prepare(){if(!trainingLocked)return log("training is not locked");val p=proposal?:return log("no proposal");if(p.decision!="propose")return log("LLM decision=${p.decision}; no candidate will be sent");if(machine.state!=TrialState.REVIEW)return log("collect/finish baseline first");val id=System.currentTimeMillis()/1000;ble.send(Wire.prepare(id,ttl,p.candidate));machine.prepareSent(id);log("PREP sent; wait for ACK")}
    private fun applyCandidate(){if(machine.state!=TrialState.PREPARED)return log("wait for PREP ACK");ble.send(Wire.command("APPLY",machine.id));machine.applySent();heartbeat()}
    private fun acceptCandidate(){if(machine.state!=TrialState.TRIAL)return log("candidate is not active");ble.send(Wire.command("ACCEPT",machine.id));machine.finish();handler.postDelayed({ble.send(Wire.getPid())},150)}
    private fun rollbackCandidate(){if(machine.state !in setOf(TrialState.PREPARING,TrialState.PREPARED,TrialState.APPLYING,TrialState.TRIAL))return log("no candidate to rollback");ble.send(Wire.command("ROLLBACK",machine.id));machine.finish();handler.postDelayed({ble.send(Wire.getPid())},150)}

    override fun device(d:BluetoothDevice)=runOnUiThread{val k="${d.name?:"BLE"} ${d.address}";devices[k]=d;deviceBox.adapter=ArrayAdapter(this,android.R.layout.simple_spinner_dropdown_item,devices.keys.toList())}
    override fun status(s:String)=log(s)
    override fun ready(){log("双向链路探测：发送 GET");ble.send(Wire.getPid())}
    override fun rssi(v:Int)=log("RSSI=$v")
    override fun bytes(bytes:ByteArray){parser.feed(bytes).forEach{raw->
        log("RX $raw")
        Wire.parsePidStatus(raw)?.let{s->pid=s.pid;pidKnown=true;firmwareState=s.state;balanceStarted=s.balanceStarted;showPid(pid,"GET");setTrainingLock(s.trainingLocked);log("PID synchronized: ${s.pid}")}
        if(raw.contains("P1,ACK,MSET,"))handler.postDelayed({ble.send(Wire.getPid())},100)
        if(raw.contains("P1,ACK,TRAIN_START,")){setTrainingLock(true);handler.postDelayed({ble.send(Wire.getPid())},100)}
        if(raw.contains("P1,ACK,TRAIN_STOP,"))handler.postDelayed({ble.send(Wire.getPid())},100)
        if(raw.contains("P1,ACK,PREP,")&&machine.state==TrialState.PREPARING)machine.prepared()
        Wire.parseFast(raw)?.let{t->firmwareState=t.state;balanceStarted=t.balanceStarted;setTrainingLock(t.trainingLocked);if(t.state==2&&machine.state==TrialState.APPLYING)machine.applied();if(t.state==0&&machine.state in setOf(TrialState.PREPARING,TrialState.PREPARED,TrialState.APPLYING,TrialState.TRIAL)){machine.finish();log("firmware ended trial or rolled back")};samples+=t;updateTrainingStatus();runOnUiThread{live.text="angle %.2f° gyro %.1f enc %d/%d cmd %d/%d CCR %d,%d/%d,%d state %d".format(t.angle,t.gyro,t.el,t.er,t.ml,t.mr,t.c1,t.c2,t.c3,t.c4,t.state);chart.add(t.angle.toFloat())}}
        Wire.parseSlow(raw)?.let{s->pid=s.pid;pidKnown=true;firmwareVersion=s.firmware;balanceStarted=s.balanceStarted;showPid(pid,"${s.ms} ms");setTrainingLock(s.trainingLocked)}
    }}

    private fun heartbeat(){handler.postDelayed(object:Runnable{override fun run(){if(machine.state==TrialState.APPLYING||machine.state==TrialState.TRIAL){ble.send(Wire.command("HB",machine.id));handler.postDelayed(this,500)}}},500)}
    private fun requestLlm(){if(!trainingLocked)return log("start training first");if(machine.state!=TrialState.REVIEW)return log("collect and finish baseline first");val prefs=getSharedPreferences("settings",0);val key=KeyVault(this).load();if(key.isBlank())return log("API key missing; open 设置");val pct=prefs.getString("pct","5.0")!!.toDoubleOrNull()?.coerceIn(0.1,5.0)?:5.0;worker.execute{runCatching{val raw=DeepSeekClient().propose(key,prefs.getString("base","https://api.deepseek.com")!!,prefs.getString("model","deepseek-flash")!!,pid,ExperimentLogic.metrics(samples),pct);proposal=ExperimentLogic.proposal(raw,pid,pct);log("LLM proposal=$proposal")}.onFailure{log("LLM rejected: ${it.message}")}}}
    private fun settings(){val prefs=getSharedPreferences("settings",0);val box=LinearLayout(this).apply{orientation=LinearLayout.VERTICAL};val key=EditText(this).apply{hint="DeepSeek API key";setText(KeyVault(this@MainActivity).load())};val base=EditText(this).apply{hint="Base URL";setText(prefs.getString("base","https://api.deepseek.com"))};val model=EditText(this).apply{hint="Model";setText(prefs.getString("model","deepseek-flash"))};val pct=EditText(this).apply{hint="Real-hardware max change %, <=5";setText(prefs.getString("pct","5.0"))};listOf(key,base,model,pct).forEach(box::addView);AlertDialog.Builder(this).setTitle("Route A settings").setView(box).setPositiveButton("保存"){_,_->KeyVault(this).save(key.text.toString());prefs.edit().putString("base",base.text.toString()).putString("model",model.text.toString()).putString("pct",pct.text.toString()).apply()}.setNegativeButton("取消",null).show()}
    override fun onDestroy(){if(!balanceStarted)ble.send(Wire.motorDiag("STOP"));if(machine.state in setOf(TrialState.PREPARING,TrialState.PREPARED,TrialState.APPLYING,TrialState.TRIAL))ble.send(Wire.command("ROLLBACK",machine.id));ble.disconnect();worker.shutdownNow();super.onDestroy()}
}
