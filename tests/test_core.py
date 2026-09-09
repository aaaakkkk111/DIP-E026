import time, math, json
from pathlib import Path
from app.physics.simulation import SimulationBackend,PhysicsConfig
from app.hardware.backends import ReplayBackend,MockHardwareBackend,HardwareBackend
from app.types import RobotState,MotorCommand
from app.control.policies import PIDPolicy,CascadedPIDPolicy,LQRPolicy,PPOPolicy,load_policy
from app.control.disturbances import DisturbanceGenerator,DisturbanceConfig
from app.safety.layer import SafetyLayer
from app.reward.spec import RewardSpec
from app.telemetry.store import TelemetryStore,LineAnalyzer
from app.engine import ControlEngine
from app.experiments.manager import ExperimentManager,ExperimentConfig
from app.training.ollama import OllamaAdvisor
from app.training.pipelines import PIDTuningPipeline,RewardSearchPipeline
from app.config import load_config
from app.experiments.physical_validation import PhysicalValidationRunner,telemetry_metrics,summary

def test_physics_has_separate_sensor_state():
    b=SimulationBackend(PhysicsConfig(sensor_noise=.1,sensor_latency_s=0),1);b.step(.01);assert b.observe().pitch!=b.true.pitch
def test_custom_simulation_pose_reset():
    b=SimulationBackend(PhysicsConfig(sensor_noise=0,sensor_latency_s=0));s=b.set_pose(position=2,pitch=.2,velocity=.3,pitch_rate=.4);assert (s.position,s.pitch,s.velocity,s.pitch_rate)==(2,.2,.3,.4)
def test_pid_lqr_and_loader():
    s=RobotState(pitch=.1);assert PIDPolicy().act(s,.01).terms["P"]<0;assert abs(LQRPolicy().act(s,.01).command.left)<=1;assert isinstance(load_policy("pid",{}),PIDPolicy);assert isinstance(CascadedPIDPolicy().act(s,.01).command,MotorCommand)
def test_disturbance_manual_and_seed():
    d=DisturbanceGenerator(DisturbanceConfig(magnitude=3,duration_s=.1));d.trigger();assert d.force(.01)==3
    a=DisturbanceGenerator(DisturbanceConfig(kind="random",probability=1,seed=2));b=DisturbanceGenerator(DisturbanceConfig(kind="random",probability=1,seed=2));assert [a.force(.1) for _ in range(5)]==[b.force(.1) for _ in range(5)]
def test_safety_disarmed_estop_nan_limits():
    safety=SafetyLayer();assert safety.filter(RobotState(),MotorCommand(1,1)).left==0;safety.arm();assert safety.filter(RobotState(pitch=9),MotorCommand(1,1)).left==0;safety.reset_estop();safety.arm();assert safety.filter(RobotState(),MotorCommand(float('nan'),0)).left==0
def test_reward_validation():
    assert RewardSpec().validate().score(RobotState(),MotorCommand())>0
    try:RewardSpec(angle=-1).validate();assert False
    except ValueError:pass
def test_telemetry_and_line_analyzer(tmp_path):
    t=TelemetryStore(tmp_path);t.record(RobotState(pitch=.1),MotorCommand(),SafetyLayer().status,{},1);t.flush("x");assert (tmp_path/"x.jsonl").exists();assert LineAnalyzer(t).bounds("pitch")==(.1,.1)
def test_backend_interchangeability_and_mock():
    for b in (SimulationBackend(),MockHardwareBackend(),ReplayBackend([RobotState()])):
        b.reset(1);b.apply(MotorCommand());assert isinstance(b.step(.01),RobotState)
    try:HardwareBackend().apply(MotorCommand());assert False
    except RuntimeError:pass
def test_ollama_parser():
    assert OllamaAdvisor.pid_candidate({"kp":1,"ki":2,"kd":3})["kd"]==3
def test_small_pid_and_reward_pipelines():
    metrics={"rms_angle":.1,"survival_time":1}
    assert len(PIDTuningPipeline(lambda p:metrics).run({"kp":2,"ki":0,"kd":1},2))==2
    assert len(RewardSearchPipeline(lambda r,randomized:{"rms_angle":.1,"survival_time":1}).run(RewardSpec(),2))==2
def test_config():
    assert load_config("configs/default.json")["backend"]=="simulation"
def test_end_to_end_and_reproducible_experiment(tmp_path):
    t=TelemetryStore(tmp_path);e=ControlEngine(SimulationBackend(seed=1),PIDPolicy(),SafetyLayer(),t);e.start();time.sleep(.04);e.pause();assert len(t.rows)>0 and all("reward" in x for x in t.rows)
    m=ExperimentManager(tmp_path);c=ExperimentConfig(timesteps=30,seed=4);a=m.run(c,PIDPolicy);b=m.run(c,PIDPolicy);assert a==b
def test_physical_protocol_plan_metrics_and_disarmed_gate(tmp_path):
    cfg=json.loads(Path("configs/physical_validation_development.json").read_text());cfg["experiment_id"]="test-physical";cfg["hardware_config"]=str(Path("configs/hardware_unverified.json").resolve());path=tmp_path/"protocol.json";path.write_text(json.dumps(cfg));runner=PhysicalValidationRunner(path,tmp_path)
    assert len(runner.schedule())==45 and runner.next_trial()[0]>=0
    try:runner.validate_preflight();assert False
    except RuntimeError:pass
    m=telemetry_metrics([{"pitch":.1,"pitch_rate":.2,"left_command":.3,"position":.4,"armed":True}]);assert set(m)==set(("survival_time","rms_pitch","peak_pitch","recovery_time","angular_velocity_rms","motor_effort","position_drift","success"))
