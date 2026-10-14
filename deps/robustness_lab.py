"""Frozen-policy robustness experiments and interactive view of EXISTING MuJoCo plant.

No training, no new simulator, no LLM API calls. real_robot.xml, existing
TzejunPlant and existing motor_model are the base. Every new run is separate.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
import platform
import sys
import time
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
import mujoco
import numpy as np
from motor_model import action_to_pwm, pwm_to_torque
from tzejun_experiment import (TzejunPlant, EpisodeConfig, ObservationState,
    ExportedPolicy, decode_state, quat_from_roll_pitch, parse_c_array,
    CONTROL_DT, XML_PATH, CHECKPOINT_PATH, SOURCE_COMMIT)

ROOT=Path(__file__).resolve().parent
PPO_DIR=ROOT/'keil_llm_reward_ppo_pid'
PPO_WEIGHTS=PPO_DIR/'stm32_Balance_Car_L'/'APP'/'YbHybrid'/'policy_weights.h'
PPO_CHECKPOINT=ROOT/'experiment_comparison'/'llm_reward_iter_04.zip'
MODELS=('tzejun','ppo-pid')
IMPLEMENTATIONS=('checkpoint','firmware-adapter')
LABELS={'tzejun':'tzejun standalone PPO','ppo-pid':'LLM-Reward-PPO + PID'}
PID_GAINS=json.loads((ROOT/'experiment_comparison'/'protocol.json').read_text())['pid_gains']
FIELDS=('t_s','pitch_deg','roll_deg','yaw_deg','pitch_rate_rad_s','yaw_rate_rad_s',
    'v_forward_m_s','target_v_m_s','target_yaw_rad_s','x_m','y_m','radius_m',
    'action_left','action_right','pwm_left','pwm_right','torque_left_nm','torque_right_nm',
    'raw_torque_left_nm','raw_torque_right_nm','force_x_n','force_y_n',
    'payload_kg','friction_mu','terminated',
    *(f'qpos_{i}' for i in range(9)),*(f'qvel_{i}' for i in range(8)))

def save_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8')
    os.replace(temp,path)

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

class FrozenActor:
    def __init__(self,model,fast=False):
        path=ROOT/'firmware'/'policy_weights.h' if model=='tzejun' else PPO_WEIGHTS
        n=34 if model=='tzejun' else 9
        self.weights=[];self.fast=fast
        for i,shape in enumerate(((64,n),(64,64),(2,64))):
            self.weights.append((parse_c_array(path,f'policy_w{i}').reshape(shape),
                                 parse_c_array(path,f'policy_b{i}')))
    def predict(self,obs):
        x=np.asarray(obs,dtype=np.float32)
        for i,(w,b) in enumerate(self.weights):
            x=w@x+b
            if i<2:
                if self.fast:
                    x2=x*x
                    x=np.where(x>3,1,np.where(x<-3,-1,x*(27+x2)/(27+9*x2))).astype(np.float32)
                else:x=np.tanh(x)
            else:x=np.clip(x,-1,1)
        return x

class Controller:
    def __init__(self,model,implementation='checkpoint'):
        self.model_name=model;self.implementation=implementation
        self.actor=FrozenActor(model,fast=model=='tzejun' and implementation=='firmware-adapter')
        self.plant=None;self.reset()
    def reset(self):
        self.speed_i=0.;self.prev_counts=np.zeros(2,dtype=np.int64)
        self.history=None;self.pos_err=0.;self.yaw_err=0.
        self.last_pid=np.zeros(2);self.last_residual=np.zeros(2)
        self.last_pwm=np.zeros(2)
    def predict(self,obs):
        p=self.plant;d=p.data
        roll,pitch,_,lv,yr=decode_state(d)
        rates=np.asarray(d.qvel[3:6]);wl,wr=d.qvel[6:8];v=float(lv[0])
        firmware=self.implementation=='firmware-adapter'
        if firmware:
            # Idealized reproduction of encoder quantization and C adapter.
            # IMU fusion delay/noise and ARM floating-point execution NOT modeled.
            counts=np.rint(d.qpos[7:9]*1320/(2*np.pi)).astype(np.int64)
            wl,wr=(counts-self.prev_counts)*(2*np.pi/(1320*CONTROL_DT));self.prev_counts=counts
            rates=np.rint(rates*939.65078)/939.65078;yr=float(rates[2])
            if self.model_name=='tzejun':v=float(.5*(wl+wr)*.0335)
            else:v=float((.5*(wl+wr)+rates[1])*.0335*math.cos(pitch))
        if self.model_name=='tzejun':
            if firmware:
                sample=np.asarray([pitch,rates[1],v],dtype=np.float32)
                if self.history is None:self.history=[sample.copy() for _ in range(48)]
                self.history.insert(0,sample);del self.history[48:]
                self.pos_err=.99750312*self.pos_err+(v-p.target_v)*CONTROL_DT
                self.yaw_err=float(np.clip(.99983335*self.yaw_err+(yr-p.target_yaw)*CONTROL_DT,-.2,.2))
                base=np.concatenate([[.05],quat_from_roll_pitch(roll,pitch),
                    counts*(2*np.pi/1320),[v,0,0],rates,[wl,wr,p.target_v,p.target_yaw]])
                obs=np.concatenate([base,*[self.history[t] for t in (2,5,11,23,47)],
                                    [self.pos_err*10,self.yaw_err*5]]).astype(np.float32)
            action=self.actor.predict(obs)
        else:
            state=np.clip(np.asarray([pitch/.35,rates[1]/5,roll/.35,rates[0]/5,
                yr/2,wl/35,wr/35,p.target_v/.3,p.target_yaw/.5],dtype=np.float32),-5,5)
            self.last_residual=self.actor.predict(state)*np.float32(.3)
            err=p.target_v-v;g=PID_GAINS
            self.speed_i=float(np.clip(self.speed_i+err*CONTROL_DT,-.4,.4))
            balance=g['pitch']*pitch+g['pitch_rate']*rates[1]-g['speed']*err-g['speed_i']*self.speed_i
            turn=g['yaw']*(p.target_yaw-yr)-g['yaw_rate']*yr
            self.last_pid=np.asarray([balance-turn,balance+turn],dtype=np.float32)
            action=np.clip(self.last_pid+self.last_residual,-1,1)
        if firmware:
            # Native int PWM -> native deadband once -> native 2600 ceiling.
            pre=np.trunc(np.clip(action,-1,1)*1500)
            self.last_pwm=np.clip(np.where(pre!=0,pre+np.sign(pre)*1300,0),-2600,2600)
            # Existing plant applies motor_model; inverse map produces same PWM.
            action=np.where(self.last_pwm!=0,np.sign(self.last_pwm)*(np.abs(self.last_pwm)-1300)/1500,0)
        else:self.last_pwm=action_to_pwm(action)
        return np.asarray(action,dtype=np.float32)

class RobustPlant(TzejunPlant):
    """The SAME existing plant, with explicit payload inertia/constant updates."""
    def __init__(self,policy):
        super().__init__(policy);policy.plant=self
        self.chassis_id=self.model.body('chassis').id
        self.mu=.9;self.payload=0.
    def set_parameters(self,mu,payload,preserve=True):
        if not (math.isfinite(mu) and .05<=mu<=2.):raise ValueError('friction must be 0.05..2')
        if not (math.isfinite(payload) and 0<=payload<=4.):raise ValueError('payload must be 0..4 kg')
        d=self.data;m=self.model
        snapshot=(d.qpos.copy(),d.qvel.copy(),d.ctrl.copy(),float(d.time))
        self.mu=float(mu);self.payload=float(payload)
        # MuJoCo mixes equal-priority geom friction by MAX. Change BOTH surfaces.
        m.geom_friction[:,0]=mu
        m.body_mass[self.payload_id]=payload
        # Uniform box: dimensions from existing payload geom (.06,.06,.04 m).
        # Assumption for the ADDED payload, not a measurement of real hardware.
        hx,hy,hz=.03,.03,.02
        m.body_inertia[self.payload_id]=payload*np.asarray([hy*hy+hz*hz,hx*hx+hz*hz,hx*hx+hy*hy])/3
        m.geom_rgba[m.geom('payload_geom').id,3]=1. if payload else 0.
        mujoco.mj_setConst(m,d)
        if preserve:
            d.qpos[:],d.qvel[:],d.ctrl[:],d.time=snapshot
        mujoco.mj_forward(m,d)
    def reset(self,cfg):
        self.policy.reset();self.set_parameters(self.mu,cfg.payload_kg,preserve=False)
        obs=super().reset(cfg)
        self.data.xfrc_applied[:]=0
        return obs
    def control_step(self,obs,force):
        self.data.xfrc_applied[:]=0
        self.data.xfrc_applied[self.chassis_id,:3]=force
        obs,row=super().step(obs)
        # Report ACTUAL actuator force, not requested torque before ctrlrange.
        row['raw_torque_left_nm']=row['torque_left_nm'];row['raw_torque_right_nm']=row['torque_right_nm']
        row['torque_left_nm'],row['torque_right_nm']=map(float,self.data.actuator_force[:2])
        row.update(x_m=float(self.data.qpos[0]),y_m=float(self.data.qpos[1]),
            radius_m=float(np.linalg.norm(self.data.qpos[:2])),target_v_m_s=self.target_v,
            target_yaw_rad_s=self.target_yaw,payload_kg=self.payload,friction_mu=self.mu,
            force_x_n=float(force[0]),force_y_n=float(force[1]))
        for i,val in enumerate(self.data.qpos):row[f'qpos_{i}']=float(val)
        for i,val in enumerate(self.data.qvel):row[f'qvel_{i}']=float(val)
        return obs,row

@dataclass(frozen=True)
class Scenario:
    name:str
    mu:float=.9
    payload:float=0.
    force_x:float=0.
    force_y:float=0.
    push_start:float=6.
    push_duration:float=.2
    command:str='stand'

def scenarios():
    result=[Scenario('nominal')]
    result += [Scenario(f'friction_{mu:.2f}',mu=mu) for mu in (.2,.45,1.2)]
    result += [Scenario(f'payload_{kg:.2f}kg',payload=kg) for kg in (.5,1,2,4)]
    for force in (1.,2.):
        for label,x,y in (('posX',force,0),('negX',-force,0),('posY',0,force),('negY',0,-force)):
            result.append(Scenario(f'push_{label}_{force:.0f}N',force_x=x,force_y=y))
    result += [Scenario('drive_tracking',command='drive'),Scenario('turn_tracking',command='turn')]
    return result

def initial_config(sc,seed):
    rng=np.random.default_rng(20261009+seed)
    return EpisodeConfig(sc.name,seed,seed,float(rng.uniform(-math.radians(.5),math.radians(.5))),
        math.radians(10 if seed%2==0 else -10),False,0.,0.,sc.payload)

def command_at(sc,t):
    if sc.command=='stand':return 0.,0.
    value=1. if 2<=t<5 else (-1. if 7<=t<10 else 0.)
    return (value*.15,0.) if sc.command=='drive' else (0.,value*.25)

def force_at(sc,t):
    if sc.push_start<=t<sc.push_start+sc.push_duration:
        return np.asarray([sc.force_x,sc.force_y,0.])
    return np.zeros(3)

def first_dwell(times,angle,start,end):
    mask=(times>=start-1e-9)&(times<end-1e-9)
    t=times[mask];p=angle[mask];n=100
    if len(t)<n:return None
    hits=np.convolve((np.abs(p)<2).astype(np.int32),np.ones(n,dtype=np.int32),'valid')
    idx=np.flatnonzero(hits==n)
    return float(max(0.,t[idx[0]]-start)) if len(idx) else None

def metrics(rows,cfg,sc,duration):
    a=np.asarray([[r[k] for k in FIELDS] for r in rows],dtype=np.float64)
    c={k:a[:,i] for i,k in enumerate(FIELDS)}
    t=c['t_s'];theta=c['pitch_deg'];survived=bool(not rows[-1]['terminated'] and t[-1]>=duration-1e-8)
    tail=(t>=duration-2.-1e-9) if survived else np.zeros(len(t),dtype=bool)
    def rms(x,mask=None):
        v=x if mask is None else x[mask]
        return float(np.sqrt(np.mean(v*v))) if len(v) else None
    def mean(x,mask=None):
        v=x if mask is None else x[mask]
        return float(np.mean(v)) if len(v) else None
    has_push=bool(sc.force_x or sc.force_y)
    settle=first_dwell(t,theta,0.,sc.push_start if has_push else min(2.,duration))
    recover=first_dwell(t,theta,sc.push_start+sc.push_duration,duration) if has_push and survived else None
    torque=np.column_stack([c['torque_left_nm'],c['torque_right_nm']])
    steady_rms=rms(theta,tail);speed_err=c['v_forward_m_s']-c['target_v_m_s'];yaw_err=c['yaw_rate_rad_s']-c['target_yaw_rad_s']
    speed_tail=rms(speed_err,tail);yaw_tail=rms(yaw_err,tail);drift=float(np.max(c['radius_m'][tail])) if tail.any() else None
    no_push_yet=has_push and t[-1]<sc.push_start
    stationary=sc.command=='stand'
    success=bool(survived and settle is not None and settle<1.5 and steady_rms<1.5
        and speed_tail<.05 and yaw_tail<.1 and (not stationary or drift<.25)
        and (not has_push or (recover is not None and recover<2.)))
    result={'scenario':sc.name,'seed':cfg.seed,'initial_pitch_deg':math.degrees(cfg.initial_pitch),
        'initial_roll_deg':math.degrees(cfg.initial_roll),'steps':len(rows),'duration_s':float(t[-1]),
        'survived':survived,'stationkeeping_or_tracking_success':success,'push_reached':has_push and not no_push_yet,
        'initial_settling_s':settle,'push_recovery_s':recover,
        'opposite_side_overshoot_deg':float(max(0.,np.max(-np.sign(cfg.initial_pitch)*theta[t<min(2.,duration)]))),
        'max_pitch_deg':float(np.max(np.abs(theta))),'max_roll_deg':float(np.max(np.abs(c['roll_deg']))),
        'steady_rms_pitch_deg':steady_rms,'steady_rms_roll_deg':rms(c['roll_deg'],tail),
        'steady_speed_error_rms_m_s':speed_tail,'steady_yaw_error_rms_rad_s':yaw_tail,
        'steady_mean_yaw_rad_s':mean(c['yaw_rate_rad_s'],tail),
        'max_displacement_m':float(np.max(c['radius_m'])),'steady_max_displacement_m':drift,
        'steady_rms_torque_nm':rms(torque[tail]) if tail.any() else None,
        'steady_torque_variance':float(np.var(torque[tail])) if tail.any() else None,
        'whole_mean_abs_torque_nm':mean(np.abs(torque)),
        'whole_torque_variance':float(np.var(torque)),
        'pwm_saturation_fraction':float(np.mean(np.max(np.abs(np.column_stack([c['pwm_left'],c['pwm_right']])),axis=1)>=2799.5)),
        'raw_torque_limit_fraction':float(np.mean(np.max(np.abs(np.column_stack([c['raw_torque_left_nm'],c['raw_torque_right_nm']])),axis=1)>.4)),
        'actual_torque_max_nm':float(np.max(np.abs(torque)))}
    return result,a

def stats(values,proportion=False):
    v=np.asarray([float(x) for x in values if x is not None],dtype=np.float64);n=len(v)
    if not n:return {'mean':None,'ci95_low':None,'ci95_high':None,'n':0}
    avg=float(np.mean(v))
    if proportion:
        z=1.95996398454;den=1+z*z/n
        center=(avg+z*z/(2*n))/den;half=z*math.sqrt(avg*(1-avg)/n+z*z/(4*n*n))/den
        lo,hi=max(0.,center-half),min(1.,center+half)
    elif n>1:
        rng=np.random.default_rng(20261009)
        means=v[rng.integers(0,n,(10000,n))].mean(axis=1)
        lo,hi=map(float,np.quantile(means,[.025,.975]))
    else:lo=hi=avg
    return {'mean':avg,'ci95_low':lo,'ci95_high':hi,'n':n}

_AGG_CACHE={}
def aggregate(raw):
    groups={}
    for r in raw:groups.setdefault((r['implementation'],r['scenario'],r['model']),[]).append(r)
    summary=[]
    for (implementation,scenario,model),rows in groups.items():
        cache_key=(implementation,scenario,model,len(rows),rows[-1]['seed'])
        if cache_key in _AGG_CACHE:
            summary.append(_AGG_CACHE[cache_key]);continue
        result={'implementation':implementation,'scenario':scenario,'model':model,'episodes':len(rows)}
        for key,value in rows[0].items():
            if key in ('scenario','seed','model','implementation','initial_pitch_deg','initial_roll_deg','artifact','exception'):continue
            if isinstance(value,(int,float,bool)) or value is None:
                result[key]=stats([r[key] for r in rows],proportion=isinstance(value,bool))
        _AGG_CACHE[cache_key]=result;summary.append(result)
    return summary

def write_rollout(folder,rows,array,meta):
    folder.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(folder/'rollout.npz',data=array,columns=np.asarray(FIELDS),
        initial_qpos=np.asarray(meta['initial_qpos']),initial_qvel=np.asarray(meta['initial_qvel']))
    with (folder/'rollout.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=FIELDS);writer.writeheader()
        writer.writerows({k:r[k] for k in FIELDS} for r in rows)
    save_json(folder/'meta.json',meta)

def run_trial(controller,sc,seed,duration):
    plant=RobustPlant(controller);plant.mu=sc.mu;cfg=initial_config(sc,seed)
    obs=plant.reset(cfg);qpos=plant.data.qpos.copy();qvel=plant.data.qvel.copy()
    com=plant.data.subtree_com[plant.chassis_id].copy();rows=[]
    for _ in range(int(round(duration/CONTROL_DT))):
        t=plant.step_count*CONTROL_DT;plant.target_v,plant.target_yaw=command_at(sc,t)
        # The base 34-input observation contains command channels; rebuild only
        # their values, not history/integrals (which advance exactly once/tick).
        obs[15:17]=[plant.target_v,plant.target_yaw]
        obs,row=plant.control_step(obs,force_at(sc,t));rows.append(row)
        if row['terminated']:break
    result,array=metrics(rows,cfg,sc,duration)
    meta={'config':asdict(cfg),'scenario':asdict(sc),'initial_qpos':qpos.tolist(),'initial_qvel':qvel.tolist(),
        'actual_total_mass_kg':float(mujoco.mj_getTotalmass(plant.model)),
        'payload_inertia_kg_m2':plant.model.body_inertia[plant.payload_id].tolist(),
        'com_world_m_at_reset':com.tolist(),'implementation':controller.implementation,
        'model':controller.model_name,'metrics':result}
    return rows,result,array,meta

def run_experiment(args):
    folder=args.out or ROOT/'experiment_robustness'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    folder.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter();raw=[];failures=[];chosen=scenarios()
    if args.scenarios:chosen=[s for s in chosen if s.name in args.scenarios.split(',')]
    if not chosen:raise ValueError('No matching scenarios')
    config={'started_utc':datetime.now(timezone.utc).isoformat(),'models':list(MODELS),'seeds':list(range(args.seeds)),
        'duration_s':args.duration,'scenarios':[asdict(s) for s in chosen],
        'secondary_firmware_adapter_scenarios':['nominal','payload_1.00kg','push_posX_1N'],
        'physics_dt_s':.001,'control_dt_s':CONTROL_DT,'physics_integrator':'existing XML Euler; unchanged',
        'protocol':'frozen deterministic checkpoints, paired initial pitch +/-10deg, roll U[-0.5,+0.5]deg from seed; zero velocities/wheel positions',
        'interpretation':'15 evaluation initial-state trials, NOT 15 independently trained policies',
        'disturbance':'world-frame horizontal force at chassis CoM, t=6.0..6.2s',
        'payload':'uniform .06x.06x.04m box at existing payload_body 0.105m above axle; mj_setConst after mass/inertia updates',
        'friction':'same sliding coefficient on all contact geoms, original torsional/rolling coefficients unchanged',
        'settling':'first 0.5s consecutive abs(pitch)<2deg before t=2s, or before push; not steady-state RMS',
        'steady_state':'last 2 seconds ONLY for completed episodes; failed episodes have null steady-state metrics',
        'success':'survival, initial settling<1.5s, steady pitch RMS<1.5deg, speed error RMS<.05m/s, yaw error RMS<.1rad/s, stand drift<.25m, push recovery<2s when applicable',
        'ci':'continuous: 10000 paired-trial bootstrap mean samples; proportions: Wilson 95%; absent/censored metrics excluded with n reported',
        'source_commit':SOURCE_COMMIT,'xml_sha256':digest(XML_PATH),
        'checkpoints':{'tzejun':digest(CHECKPOINT_PATH),'ppo-pid':digest(PPO_CHECKPOINT)},
        'versions':{'python':platform.python_version(),'numpy':np.__version__,'mujoco':mujoco.__version__},
        'training_performed':False,'ollama_calls':0,
        'firmware_adapter_emulation':'secondary sensitivity study, NOT target binary or hardware validation; gyro quantization, encoder quantization/odometry, tzejun height/yaw substitutions/fast tanh, native int PWM/2600 cap; perfect Euler IMU without fusion delay; safety cutoffs not emulated'}
    save_json(folder/'protocol.json',config)
    total=len(chosen)*args.seeds*2+len([s for s in chosen if s.name in config['secondary_firmware_adapter_scenarios']])*args.seeds*2
    print(f'OUTPUT={folder}\nFROZEN_POLICY_EVALUATIONS={total} duration={args.duration}s physics=1000Hz control=200Hz',flush=True)
    try:
        for implementation in IMPLEMENTATIONS:
            suite=chosen if implementation=='checkpoint' else [s for s in chosen if s.name in config['secondary_firmware_adapter_scenarios']]
            for sc in suite:
                for model in MODELS:
                    controller=Controller(model,implementation)
                    for seed in range(args.seeds):
                        try:
                            rows,result,array,meta=run_trial(controller,sc,seed,args.duration)
                            artifact=Path('episodes')/implementation/sc.name/model/f'seed_{seed:02d}'
                            result.update(model=model,implementation=implementation,artifact=artifact.as_posix())
                            write_rollout(folder/artifact,rows,array,meta);raw.append(result)
                            print(f'{len(raw):04d}/{total} {implementation:16s} {sc.name:20s} {model:7s} seed={seed:02d} survive={result["survived"]} success={result["stationkeeping_or_tracking_success"]} rms={result["steady_rms_pitch_deg"]} yaw={result["steady_mean_yaw_rad_s"]}',flush=True)
                        except Exception as exc:
                            failure={'implementation':implementation,'scenario':sc.name,'model':model,'seed':seed,'error':repr(exc),'traceback':traceback.format_exc()}
                            failures.append(failure);print('FAILED',json.dumps(failure),flush=True)
                        save_json(folder/'metrics_raw.json',raw)
                        save_json(folder/'summary.json',aggregate(raw))
                        save_json(folder/'failures.json',failures)
    finally:
        config.update(finished_utc=datetime.now(timezone.utc).isoformat(),wall_time_s=time.perf_counter()-started,
            completed_episodes=len(raw),expected_episodes=total,failed_episodes=len(failures),complete=len(raw)==total and not failures)
        save_json(folder/'protocol.json',config)
        save_json(folder/'summary.json',aggregate(raw));save_json(folder/'metrics_raw.json',raw);save_json(folder/'failures.json',failures)
        save_json(ROOT/'experiment_robustness'/'latest.json',{'path':str(folder)})
    make_report(folder)
    print('COMPLETE=',config['complete'],'wall_time_s=',config['wall_time_s'],flush=True)
    return 0 if config['complete'] else 1

# Viewer, verification and report functions are below; all reuse RobustPlant.

def make_report(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    raw=json.loads((folder/'metrics_raw.json').read_text());summary=aggregate(raw)
    protocol=json.loads((folder/'protocol.json').read_text())
    save_json(folder/'summary.json',summary)
    figures=folder/'figures';figures.mkdir(exist_ok=True)
    # Flat table with a confidence interval and valid n for EVERY metric.
    with (folder/'summary_table.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['implementation','scenario','model','metric','mean','ci95_low','ci95_high','n'])
        for group in summary:
            for key,val in group.items():
                if isinstance(val,dict):writer.writerow([group['implementation'],group['scenario'],group['model'],key,val['mean'],val['ci95_low'],val['ci95_high'],val['n']])
    groups={(g['implementation'],g['scenario'],g['model']):g for g in summary}
    scenario_names=[s['name'] for s in protocol['scenarios']]
    colors={'tzejun':'#0072B2','ppo-pid':'#D55E00'}
    for metric,filename,title in [('survived','survival_matrix.png','Survival (no fall for full episode)'),
        ('stationkeeping_or_tracking_success','success_matrix.png','Strict standstill / tracking success')]:
        data=np.asarray([[groups.get(('checkpoint',sc,model),{}).get(metric,{}).get('mean',np.nan)
                          for sc in scenario_names] for model in MODELS],dtype=float)
        fig,ax=plt.subplots(figsize=(16,3.6),constrained_layout=True)
        im=ax.imshow(data,vmin=0,vmax=1,cmap='RdYlGn',aspect='auto')
        ax.set_xticks(range(len(scenario_names)),scenario_names,rotation=50,ha='right',fontsize=8)
        ax.set_yticks(range(2),[LABELS[m] for m in MODELS])
        for i in range(2):
            for j in range(len(scenario_names)):
                if np.isfinite(data[i,j]):ax.text(j,i,f'{100*data[i,j]:.0f}%',ha='center',va='center',fontsize=8)
        ax.set_title(title+f' | {protocol["seeds"].__len__()} paired evaluation trials/scenario')
        fig.colorbar(im,ax=ax,label='fraction');fig.savefig(figures/filename,dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(13,12),constrained_layout=True)
    for col,category in enumerate(('friction','payload')):
        suite=[s for s in protocol['scenarios'] if s['name']=='nominal' or s['name'].startswith(category+'_')]
        suite=sorted(suite,key=lambda s:s['mu'] if category=='friction' else s['payload'])
        xx=[s['mu'] if category=='friction' else s['payload'] for s in suite]
        for row,metric in enumerate(('survived','steady_rms_pitch_deg','steady_yaw_error_rms_rad_s')):
            ax=axes[row,col]
            for model in MODELS:
                vals=[groups[('checkpoint',s['name'],model)][metric] for s in suite]
                means=np.asarray([v['mean'] if v['mean'] is not None else np.nan for v in vals])
                lows=np.asarray([v['ci95_low'] if v['ci95_low'] is not None else np.nan for v in vals])
                highs=np.asarray([v['ci95_high'] if v['ci95_high'] is not None else np.nan for v in vals])
                ax.plot(xx,means,'o-',color=colors[model],label=LABELS[model]);ax.fill_between(xx,lows,highs,color=colors[model],alpha=.15)
                if row>0:
                    for x,v,y in zip(xx,vals,means):
                        if np.isfinite(y):ax.annotate(f'n={v["n"]}',(x,y),xytext=(2,5),textcoords='offset points',fontsize=7)
            ax.set(xlabel='sliding friction coefficient' if category=='friction' else 'added payload (kg)',ylabel=metric)
            ax.grid(alpha=.25)
            if row==0:ax.legend(fontsize=8)
    fig.suptitle('Frozen checkpoint robustness; shaded 95% CIs; steady-state metrics exclude falls')
    fig.savefig(figures/'friction_payload.png',dpi=150);plt.close(fig)
    # Representative complete sensor/actuator traces, also useful for failures.
    for scenario in scenario_names:
        fig,axes=plt.subplots(3,2,figsize=(13,10),constrained_layout=True)
        for model in MODELS:
            matching=[r for r in raw if r['implementation']=='checkpoint' and r['scenario']==scenario and r['model']==model and r['seed']==0]
            if not matching:continue
            r=matching[0]
            with np.load(folder/r['artifact']/'rollout.npz') as z:
                c={str(k):z['data'][:,i] for i,k in enumerate(z['columns'])}
            t=c['t_s'];color=colors[model]
            for ax,key in zip(axes.flat,('pitch_deg','roll_deg','v_forward_m_s','yaw_rate_rad_s','radius_m','torque_left_nm')):
                ax.plot(t,c[key],color=color,label=LABELS[model]);ax.set(xlabel='time (s)',ylabel=key);ax.grid(alpha=.25)
            axes[1,0].plot(t,c['target_v_m_s'],'k--',alpha=.5)
            axes[1,1].plot(t,c['target_yaw_rad_s'],'k--',alpha=.5)
        axes[0,0].axhline(2,color='grey',ls='--');axes[0,0].axhline(-2,color='grey',ls='--');axes[0,0].legend(fontsize=8)
        fig.suptitle(scenario+' | paired seed 0, +10deg initial pitch; lines end at fall')
        fig.savefig(figures/(scenario+'.png'),dpi=140);plt.close(fig)
    def fmt(v,digits=3):
        if v['mean'] is None:return 'not measured (n=0)'
        return f'{v["mean"]:.{digits}f} [{v["ci95_low"]:.{digits}f}, {v["ci95_high"]:.{digits}f}]'
    lines=['# Frozen tzejun versus LLM-Reward-PPO + PID: robustness report','',
        '**Executed evaluation, not new training.** Both saved policies stayed frozen; no Ollama calls or gain changes occurred.',
        f'Run: `{folder.name}`. Completed **{protocol["completed_episodes"]}/{protocol["expected_episodes"]}** episodes; execution failures **{protocol["failed_episodes"]}**. Wall time **{protocol["wall_time_s"]:.1f} s** (headless evaluation, not simulated time).',
        '', '## Provenance and method','',
        f'- Existing `real_robot.xml`, SHA256 `{protocol["xml_sha256"]}`; branch commit `{SOURCE_COMMIT}` from the recorded source manifest.',
        f'- tzejun checkpoint SHA256 `{protocol["checkpoints"]["tzejun"]}`; standalone 34→64→64→2 actor.',
        f'- PPO+PID checkpoint SHA256 `{protocol["checkpoints"]["ppo-pid"]}`; iteration 04, 9→64→64→2 residual scaled 0.3 plus the **simulation-selected** PID gains `{PID_GAINS}`. Not the original Yahboom 9600/48 cascade.',
        '- Physics 1 ms, control 5 ms; original Euler/contact/motor settings retained. No change to the XML file.',
        f'- {len(protocol["seeds"])} matched evaluation trials per condition: initial pitch alternating ±10°, pitch rate and wheel positions zero, independent seeded roll offsets within ±0.5°. These are **not independently trained RL seeds**.',
        '- Nominal μ=.90; friction .20/.45/1.20; payload .5/1/2/4 kg; ±X/±Y chassis-CoM pushes at 1 or 2 N for .2 s from t=6 s; forward/reverse and left/right command schedules.',
        '- Added payload geometry .06×.06×.04 m at .105 m above axle; **uniform-density payload inertia is assumed**. Both inertia and derived constants are updated. These artificial loads are not certified hardware capacities.',
        '- Friction changes apply to both contact surfaces. Base XML claims measured values, but no fresh mass/friction/motor measurements were made in this session; rotor armature .00225 kg·m² is an inherited estimate.',
        '- Force directions are WORLD axes, not camera-relative. No force or payload sensor was measured on a physical robot.',
        '', '### Physical parameters actually used (existing XML, not freshly measured)','',
        '| Parameter | Value | Provenance boundary |',
        '|---|---|---|',
        '| Chassis mass / COM above axle | .872 kg / .034 m | inherited XML, not remeasured |',
        '| Chassis diagonal inertia | .0014229, .0014229, .0019093 kg·m² | inherited XML |',
        '| Each wheel mass / radius / full width | .035 kg / .0335 m / .024 m | inherited XML |',
        '| Wheel track / unloaded total mass | .167 m / .942 kg | inherited XML / checked against loaded model |',
        '| Reflected wheel-joint armature | .00225 kg·m² | XML explicitly describes rotor-inertia assumption |',
        '| Actuator gear / torque range | 1 / ±.4 N·m | existing wheel-side actuator, not a PWM-count actuator |',
        '| Added payload height / geometry | .105 m above axle / .06×.06×.04 m box | existing location; uniform-density inertia assumed |',
        '| Nominal μ / normal contact solref | .90 / .020, 1 | inherited XML, stress μ changed on both surfaces |',
        '| Firmware encoder / gyro assumptions | 1320 counts/rev / 16.4 LSB/(deg/s) | inherited port constants, hardware calibration unverified |',
        '', '## Metrics and uncertainty','',
        '- Initial settling: first continuous .5 s with |pitch|<2° before the command/push transient. Push recovery starts after the .2 s pulse ends.',
        '- Opposite-side overshoot is the maximum pitch excursion opposite the initial tilt, not peak |pitch| minus initial angle.',
        '- Steady-state RMS/torque are the last 2 s **only when the episode completes**. Null means not measured; this prevents a fallen robot resting on the floor appearing accurate.',
        '- Strict success additionally requires survival, settling<1.5 s, pitch RMS<1.5°, speed error RMS<.05 m/s, yaw error RMS<.1 rad/s, standstill displacement<.25 m, and push recovery<2 s where applicable.',
        '- Continuous intervals: 10,000 bootstrap samples of trial means. Proportion intervals: Wilson 95%. Valid n is recorded for every metric in `summary_table.csv`. Each fixed checkpoint is evaluated repeatedly; these intervals do not quantify training variability.',
        '', '## Nominal checkpoint comparison','',
        '| Model | Settle s [95% CI] | Pitch RMS ° [95% CI] | Idle yaw RMS rad/s [95% CI] | Mean abs torque N·m [95% CI] | Survival [95% CI] | Strict success [95% CI] |',
        '|---|---|---|---|---|---|---|']
    for model in MODELS:
        g=groups.get(('checkpoint','nominal',model))
        if g:lines.append('| '+LABELS[model]+' | '+' | '.join(fmt(g[k]) for k in ('initial_settling_s','steady_rms_pitch_deg','steady_yaw_error_rms_rad_s','whole_mean_abs_torque_nm','survived','stationkeeping_or_tracking_success'))+' |')
    lines += ['', '## All checkpoint conditions','',
        '| Scenario | Model | Survival | Strict success | Pitch RMS ° (survivors) | Yaw error RMS rad/s (survivors) | Push recovery s (survivors) |',
        '|---|---|---|---|---|---|---|']
    for sc in scenario_names:
        for model in MODELS:
            g=groups.get(('checkpoint',sc,model))
            if g:lines.append('| '+sc+' | '+LABELS[model]+' | '+' | '.join(fmt(g[k]) for k in ('survived','stationkeeping_or_tracking_success','steady_rms_pitch_deg','steady_yaw_error_rms_rad_s','push_recovery_s'))+' |')
    lines += ['', '## Firmware-adapter sensitivity (secondary, idealized)','',
        'Same frozen weights, but emulates the C ports’ encoder/gyro quantization, odometry, integer PWM and 2600 ceiling; tzejun also uses its fast tanh, height=.05, zero absolute yaw/lateral/vertical velocity. The PPO port uses exact tanh. **This is not execution of the ARM binary**: ideal Euler angles replace real Kalman fusion, and battery variation, latency and firmware safety cutoffs are not emulated.',
        '', '| Scenario | Model | Survival | Strict success | Pitch RMS ° | Yaw RMS rad/s |', '|---|---|---|---|---|---|']
    for g in summary:
        if g['implementation']=='firmware-adapter':lines.append('| '+g['scenario']+' | '+LABELS[g['model']]+' | '+' | '.join(fmt(g[k]) for k in ('survived','stationkeeping_or_tracking_success','steady_rms_pitch_deg','steady_yaw_error_rms_rad_s'))+' |')
    lines += ['', '## Interpretation and limitations','']
    if ('checkpoint','nominal','tzejun') in groups and ('checkpoint','nominal','ppo-pid') in groups:
        tg=groups['checkpoint','nominal','tzejun'];pg=groups['checkpoint','nominal','ppo-pid']
        lines += [f'- **Nominal trade-off:** tzejun settled in {tg["initial_settling_s"]["mean"]:.3f} s versus {pg["initial_settling_s"]["mean"]:.3f} s for PPO+PID; pitch RMS was {tg["steady_rms_pitch_deg"]["mean"]:.3f}° versus {pg["steady_rms_pitch_deg"]["mean"]:.3f}°. These do not make PPO+PID a safe standstill controller.',
            f'- **Idle yaw defect:** PPO+PID mean yaw was {pg["steady_mean_yaw_rad_s"]["mean"]:.3f} rad/s with zero command (yaw RMS {pg["steady_yaw_error_rms_rad_s"]["mean"]:.3f}). tzejun mean yaw was {tg["steady_mean_yaw_rad_s"]["mean"]:.6f} rad/s, but yaw RMS {tg["steady_yaw_error_rms_rad_s"]["mean"]:.3f} was just above the predefined .10 cutoff. Do not equate tzejun’s zero nominal strict success with PPO’s much larger signed rotation.',
            '- The output traces contain rapid alternating torque. Inspect the [saved-rollout chatter and command-interval analysis](ADDITIONAL_ANALYSIS.md); this additional analysis leaves the original protocol and strict success criteria unchanged.']
    for model in MODELS:
        rr=[r for r in raw if r['implementation']=='checkpoint' and r['model']==model]
        lines.append(f'- **{LABELS[model]}**: survived {sum(r["survived"] for r in rr)}/{len(rr)} and passed strict stationkeeping/tracking {sum(r["stationkeeping_or_tracking_success"] for r in rr)}/{len(rr)} over this predefined suite. This aggregate mixes distinct tasks; consult per-scenario results before declaring a winner.')
    lines += ['- A good pitch RMS alone does not demonstrate stationkeeping. Inspect yaw, translation and torque before transferring either model.',
        '- No policy was retrained for these stress tests. No new LLM reward proposal or paid API was used. The result does not isolate the causal effect of LLM reward design; these two policies have different observation architectures/training histories.',
        '- Simulation figures cannot validate real motor direction, IMU calibration, 5 ms timing, battery-dependent torque or safe payload limits. No physical robot was flashed or tested by this experiment.',
        '- **Firmware delivery is source-only pending a local rebuild:** today’s ARMCC launch returned WinError623 and uVision did not produce fresh outputs. Existing Oct7 HEX files are stale for the edited source; see `../firmware_build_status.md`. Do not flash them as current firmware.',
        '- Fallen episodes are retained in raw data and survival rates, not silently discarded. Execution exceptions, if any, are listed verbatim in `failures.json`.',
        '', '## Artifacts','', '[Protocol](protocol.json) · [Every episode metric](metrics_raw.json) · [Summary JSON](summary.json) · [Full CI table](summary_table.csv) · [Execution failures](failures.json)',
        '', '![Survival](figures/survival_matrix.png)', '', '![Strict success](figures/success_matrix.png)',
        '', '![Friction and payload](figures/friction_payload.png)', '', '![Nominal traces](figures/nominal.png)',
        '', 'Every scenario has its own plot in `figures/`. Every trial has `rollout.npz`, `rollout.csv` and `meta.json` under `episodes/`.',
        '', 'See the project-root `SIMULATION_INSTRUCTIONS.md` and `BURN_INSTRUCTIONS.md` for exact commands and hardware checks.']
    (folder/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return summary

def check_checkpoint_parity():
    # Actual SB3 checkpoint inference is checked once, then faster exported
    # NumPy inference is used for large evaluations. Same saved actors.
    import torch
    from stable_baselines3 import PPO
    import stable_baselines3
    torch.set_num_threads(1)
    out={};rng=np.random.default_rng(7)
    for model,path,n in [('tzejun',CHECKPOINT_PATH,34),('ppo-pid',PPO_CHECKPOINT,9)]:
        frozen=FrozenActor(model);saved=PPO.load(path,device='cpu')
        for i,(w,b) in enumerate(frozen.weights):
            key='action_net' if i==2 else f'mlp_extractor.policy_net.{2*i}'
            state=saved.policy.state_dict()
            np.testing.assert_array_equal(w,state[key+'.weight'].cpu().numpy())
            np.testing.assert_array_equal(b,state[key+'.bias'].cpu().numpy())
        obs=rng.uniform(-1,1,(256,n)).astype(np.float32)
        expected,_=saved.predict(obs,deterministic=True)
        actual=np.asarray([frozen.predict(o) for o in obs]);error=float(np.max(np.abs(expected-actual)))
        assert error<2e-5,(model,error)
        out[model]={'checkpoint_sha256':digest(path),'parameters':sum(w.size+b.size for w,b in frozen.weights),
                    'cases':len(obs),'max_action_error_vs_actual_sb3':error}
        print(f'PASS {model}: exact checkpoint tensors; 256 actual SB3 inference cases; max_error={error:.9g}',flush=True)
    out['versions']={'torch':torch.__version__,'stable_baselines3':stable_baselines3.__version__}
    return out

def self_test(out):
    report={'checkpoint_parity':check_checkpoint_parity(),'tests':[],'executed_utc':datetime.now(timezone.utc).isoformat()}
    for model in MODELS:
        for implementation in IMPLEMENTATIONS:
            controller=Controller(model,implementation);plant=RobustPlant(controller)
            sc=Scenario('test');cfg=initial_config(sc,0);cfg.initial_pitch=0.;cfg.initial_roll=0.
            plant.reset(cfg);q=plant.data.qpos.copy();v=plant.data.qvel.copy()
            plant.reset(cfg);np.testing.assert_array_equal(q,plant.data.qpos);np.testing.assert_array_equal(v,plant.data.qvel)
            def rollout():
                obs=plant.reset(cfg);trace=[]
                plant.target_v=.15;obs[15]=.15
                for _ in range(100):
                    obs,row=plant.control_step(obs,np.zeros(3));trace.append([row[k] for k in FIELDS])
                return np.asarray(trace)
            one,two=rollout(),rollout();np.testing.assert_array_equal(one,two)
            print(f'PASS {model}/{implementation}: reset idempotent; 100 control ticks replay bit-identical',flush=True)
            report['tests'].append(f'{model}/{implementation} reset/replay')
    plant=RobustPlant(Controller('tzejun'));cfg=initial_config(Scenario('test'),0)
    plant.reset(cfg);nominal=float(mujoco.mj_getTotalmass(plant.model));q=plant.data.qpos.copy()
    plant.set_parameters(.2,1.)
    assert abs(float(mujoco.mj_getTotalmass(plant.model))-nominal-1)<1e-9
    np.testing.assert_array_equal(q,plant.data.qpos)
    assert np.all(plant.model.body_inertia[plant.payload_id]>0)
    assert np.all(plant.model.geom_friction[:,0]==.2)
    plant.data.qpos[3:7]=[1,0,0,0];mujoco.mj_forward(plant.model,plant.data)
    for _ in range(30):mujoco.mj_step(plant.model,plant.data)
    contacts=[float(c.friction[0]) for c in plant.data.contact[:plant.data.ncon]]
    assert contacts and all(abs(c-.2)<1e-9 for c in contacts),contacts
    print('PASS payload: +1kg total mass, positive box inertia, exact pose preservation; contact friction=.2 on both surfaces',flush=True)
    report['tests'].append('mass/inertia/contact friction')
    assert sum(bool(np.any(force_at(Scenario('push',force_x=1),i*CONTROL_DT))) for i in range(2400))==40
    print('PASS push: 1N for exactly 40 control ticks = .2Ns impulse',flush=True)
    report['tests'].append('push duration')
    # Actual OpenGL renderer, not just importing a viewer module.
    renderer=mujoco.Renderer(plant.model,height=480,width=640)
    renderer.update_scene(plant.data);pixels=renderer.render();renderer.close()
    assert pixels.shape==(480,640,3) and pixels.std()>5
    import matplotlib.image as mpimg
    out.mkdir(parents=True,exist_ok=True);mpimg.imsave(out/'renderer_check.png',pixels)
    report['tests'].append('OpenGL rendered nonempty 640x480 frame')
    print('PASS actual MuJoCo OpenGL rendering: 640x480 RGB, nonempty frame',flush=True)
    save_json(out/'verification.json',report)
    return report

class LiveViewer:
    """MuJoCo native renderer + GLFW press/RELEASE callbacks, one window.

    Not the built-in passive viewer: it cannot consume/override built-in WASD
    reliably in MuJoCo 3.2.3. Physics/model/controller remain the existing ones.
    """
    def __init__(self,args):
        import glfw
        self.glfw=glfw;self.args=args
        self.model_name=args.model;self.implementation=args.implementation
        self.plant=RobustPlant(Controller(self.model_name,self.implementation))
        self.mu=.9;self.payload=0.;self.push_strength=1.;self.force=np.zeros(3);self.force_end=0.
        self.held=set();self.paused=False;self.fallen=False;self.follow=True
        self.events=[];self.rows=[];self.segment=0;self.frames=0
        self.out=args.out or ROOT/'interactive_runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.out.mkdir(parents=True,exist_ok=False);self.writer=None;self.stream=None
        self.reset()
        if not glfw.init():raise RuntimeError('GLFW initialization failed: no native display/context')
        glfw.window_hint(glfw.VISIBLE,glfw.TRUE)
        self.window=glfw.create_window(1440,900,'DIP-E026 | tzejun vs LLM-Reward-PPO + PID',None,None)
        if not self.window:glfw.terminate();raise RuntimeError('Could not create MuJoCo OpenGL viewer')
        glfw.make_context_current(self.window);glfw.swap_interval(0)
        self.camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(self.camera)
        self.camera.distance=.7;self.camera.azimuth=135;self.camera.elevation=-22
        self.option=mujoco.MjvOption();self.scene=mujoco.MjvScene(self.plant.model,maxgeom=2000)
        self.context=mujoco.MjrContext(self.plant.model,mujoco.mjtFontScale.mjFONTSCALE_100)
        self.last_mouse=glfw.get_cursor_pos(self.window)
        glfw.set_key_callback(self.window,self.on_key)
        glfw.set_window_focus_callback(self.window,lambda window,focused:self.held.clear() if not focused else None)
        glfw.set_cursor_pos_callback(self.window,self.on_mouse)
        glfw.set_scroll_callback(self.window,self.on_scroll)
        print('VIEWER_CREATED: native GLFW window; MuJoCo renderer; custom press/release keys only',flush=True)
        print('MODEL=',self.model_name,'IMPLEMENTATION=',self.implementation,'LOGS=',self.out,flush=True)

    def finish_segment(self):
        if self.stream:self.stream.flush();self.stream.close();self.stream=None
        if not self.rows:return
        folder=self.out/f'segment_{self.segment:04d}'
        array=np.asarray([[r[k] for k in FIELDS] for r in self.rows])
        np.savez_compressed(folder/'rollout.npz',data=array,columns=np.asarray(FIELDS))
        save_json(folder/'meta.json',{'model':self.model_name,'implementation':self.implementation,
            'interactive':True,'arbitrary_user_commands':True,'samples':len(self.rows)})
        self.rows=[];self.segment+=1

    def reset(self):
        self.finish_segment();self.held.clear();self.force[:]=0.;self.force_end=0.;self.fallen=False
        self.plant.mu=self.mu
        cfg=EpisodeConfig('interactive',self.segment,0,0.,0.,False,0.,0.,self.payload)
        self.obs=self.plant.reset(cfg)
        folder=self.out/f'segment_{self.segment:04d}';folder.mkdir(parents=True,exist_ok=True)
        self.stream=(folder/'rollout.csv').open('w',newline='',encoding='utf-8')
        self.writer=csv.DictWriter(self.stream,fieldnames=FIELDS);self.writer.writeheader()

    def select(self,model):
        self.finish_segment();self.model_name=model
        # Keep ONE model/scene; change only the controller and reset state.
        self.plant.policy=Controller(model,self.implementation);self.plant.policy.plant=self.plant
        self.reset()

    def on_key(self,window,key,scancode,action,mods):
        g=self.glfw
        if action==g.RELEASE:
            self.held.discard(key);return
        if key in (g.KEY_W,g.KEY_A,g.KEY_S,g.KEY_D):
            if action in (g.PRESS,g.REPEAT):self.held.add(key)
            return
        if action!=g.PRESS:return
        if key in (g.KEY_ESCAPE,g.KEY_Q):g.set_window_should_close(self.window,True)
        elif key==g.KEY_SPACE:self.held.clear()
        elif key==g.KEY_R:self.reset()
        elif key==g.KEY_P:self.paused=not self.paused;self.held.clear()
        elif key==g.KEY_C:self.follow=not self.follow
        elif key==g.KEY_1:self.select('tzejun')
        elif key==g.KEY_2:self.select('ppo-pid')
        elif key in (g.KEY_LEFT_BRACKET,g.KEY_RIGHT_BRACKET):
            self.payload=float(np.clip(self.payload+(.25 if key==g.KEY_RIGHT_BRACKET else -.25),0,4))
            self.plant.set_parameters(self.mu,self.payload)
        elif key in (g.KEY_SEMICOLON,g.KEY_APOSTROPHE):
            self.mu=float(np.clip(round(self.mu+(.1 if key==g.KEY_APOSTROPHE else -.1),2),.05,2.))
            self.plant.set_parameters(self.mu,self.payload)
        elif key in (g.KEY_COMMA,g.KEY_PERIOD):
            self.push_strength=float(np.clip(self.push_strength+(.25 if key==g.KEY_PERIOD else -.25),.25,5.))
        elif key in (g.KEY_UP,g.KEY_DOWN,g.KEY_LEFT,g.KEY_RIGHT):
            directions={g.KEY_UP:(1,0,0),g.KEY_DOWN:(-1,0,0),g.KEY_LEFT:(0,1,0),g.KEY_RIGHT:(0,-1,0)}
            self.force=np.asarray(directions[key],dtype=float)*self.push_strength
            self.force_end=self.plant.step_count*CONTROL_DT+.2
        elif key==g.KEY_F12:self.capture()
        self.events.append({'wall_s':time.perf_counter()-self.started if hasattr(self,'started') else 0,
            'sim_s':self.plant.step_count*CONTROL_DT,'key':key,'model':self.model_name,
            'payload_kg':self.payload,'friction_mu':self.mu,'force_n':self.force.tolist()})

    def on_mouse(self,window,x,y):
        g=self.glfw;oldx,oldy=self.last_mouse;self.last_mouse=(x,y)
        if g.get_mouse_button(window,g.MOUSE_BUTTON_LEFT)!=g.PRESS:return
        w,h=g.get_window_size(window)
        if x>w-390:return
        mujoco.mjv_moveCamera(self.plant.model,self.glfw_mouse_action(),(x-oldx)/h,(y-oldy)/h,self.scene,self.camera)
    def glfw_mouse_action(self):return mujoco.mjtMouse.mjMOUSE_ROTATE_V
    def on_scroll(self,window,x,y):
        mujoco.mjv_moveCamera(self.plant.model,mujoco.mjtMouse.mjMOUSE_ZOOM,0.,-.08*y,self.scene,self.camera)

    def tick(self):
        if self.paused or self.fallen:return
        g=self.glfw
        self.plant.target_v=.15*((g.KEY_W in self.held)-(g.KEY_S in self.held))
        self.plant.target_yaw=.25*((g.KEY_A in self.held)-(g.KEY_D in self.held))
        self.obs[15:17]=[self.plant.target_v,self.plant.target_yaw]
        t=self.plant.step_count*CONTROL_DT
        force=self.force if t<self.force_end-1e-9 else np.zeros(3)
        self.obs,row=self.plant.control_step(self.obs,force)
        self.rows.append(row);self.writer.writerow({k:row[k] for k in FIELDS})
        if len(self.rows)%100==0:self.stream.flush()
        if row['terminated']:self.fallen=True;self.held.clear();self.plant.data.ctrl[:]=0.

    def add_arrow(self,start,end,color,width=.006):
        i=self.scene.ngeom
        geom=self.scene.geoms[i]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
        mujoco.mjv_makeConnector(geom,mujoco.mjtGeom.mjGEOM_ARROW,width,*start,*end)
        self.scene.ngeom+=1

    def render(self):
        g=self.glfw;w,h=g.get_framebuffer_size(self.window);panel=min(440,int(w*.34))
        left=mujoco.MjrRect(0,0,w-panel,h);right=mujoco.MjrRect(w-panel,0,panel,h)
        if self.follow:self.camera.lookat[:]=self.plant.data.xpos[self.plant.chassis_id]
        mujoco.mjv_updateScene(self.plant.model,self.plant.data,self.option,None,self.camera,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
        self.add_arrow([0,0,.002],[.25,0,.002],[1,.15,.1,1])
        self.add_arrow([0,0,.002],[0,.25,.002],[.1,1,.2,1])
        if self.plant.step_count*CONTROL_DT<self.force_end:
            pos=self.plant.data.xipos[self.plant.chassis_id].copy()
            self.add_arrow(pos,pos+self.force*.10,[1,.85,.1,1],.012)
        mujoco.mjr_render(left,self.scene,self.context)
        mujoco.mjr_rectangle(right,.055,.065,.085,1.)
        roll,pitch,yaw,lv,yr=decode_state(self.plant.data)
        status_text='\n'.join([
            'DIP-E026 - FROZEN MODELS',
            *getattr(self.args,'controller_labels',('1  tzejun standalone PPO','2  LLM-Reward-PPO + PID')),
            'Active: '+self.model_name,'Variant: '+self.implementation,
            'PAUSED - P resumes' if self.paused else ('FALLEN - R resets' if self.fallen else 'RUNNING - balance active'),
            f't = {self.plant.step_count*CONTROL_DT:6.2f} s',
            f'pitch {math.degrees(pitch):+6.2f} | roll {math.degrees(roll):+6.2f} deg',
            f'speed {lv[0]:+.3f} m/s | yaw {yr:+.3f} rad/s',
            f'x/y {self.plant.data.qpos[0]:+.2f} / {self.plant.data.qpos[1]:+.2f} m',
            f'command {self.plant.target_v:+.2f} m/s, {self.plant.target_yaw:+.2f} rad/s','',
            f'PAYLOAD  {self.payload:.2f} kg', '[ / ]  decrease / increase .25kg',
            f'FRICTION mu = {self.mu:.2f}', "; / '  decrease / increase .10"])
        help_text='\n'.join([
            'HOLD W/S forward/back; A/D left/right','Release keys = zero target; Space stop','',
            f'PUSH {self.push_strength:.2f} N for .20 s',
            'Up/Down arrows   world +X/-X','Left/Right arrows world +Y/-Y',
            ', / .  decrease / increase force','Yellow arrow = applied push','',
            'R reset upright (same load/mu)','P pause   C camera follow',
            'Mouse drag orbit; wheel zoom','F12 screenshot   Esc/Q quit','',
            'Physics 1000Hz | control 200Hz','Render 30Hz | no built-in hotkeys',
            'Simulation only - not hardware proof'])
        # MuJoCo truncates long overlay strings; use two bounded text blocks.
        assert len(status_text)<900 and len(help_text)<900
        top_height=int(h*.46)
        status_rect=mujoco.MjrRect(right.left,h-top_height,right.width,top_height)
        help_rect=mujoco.MjrRect(right.left,0,right.width,h-top_height-5)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,status_rect,status_text,'',self.context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,help_rect,help_text,'',self.context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,left,
            LABELS[self.model_name]+'\nRed: world +X   Green: world +Y\nClick here, then hold WASD','',self.context)
        self.frames+=1
        return w,h

    def capture(self):
        w,h=self.render();rgb=np.empty((h,w,3),dtype=np.uint8)
        mujoco.mjr_readPixels(rgb,None,mujoco.MjrRect(0,0,w,h),self.context)
        import matplotlib.image as mpimg
        path=self.out/f'viewer_{self.frames:05d}.png';mpimg.imsave(path,np.flipud(rgb))
        assert rgb.std()>5
        print('SCREENSHOT=',path,flush=True);return path

    def smoke(self,stage):
        g=self.glfw
        def press(key):self.on_key(self.window,key,0,g.PRESS,0)
        if stage==0:press(g.KEY_W);assert g.KEY_W in self.held
        elif stage==1:
            self.on_key(self.window,g.KEY_W,0,g.RELEASE,0);assert g.KEY_W not in self.held
        elif stage==2:press(g.KEY_RIGHT_BRACKET);assert self.payload==.25
        elif stage==3:press(g.KEY_SEMICOLON);assert self.mu==.8
        elif stage in (4,5,6,7):press((g.KEY_UP,g.KEY_DOWN,g.KEY_LEFT,g.KEY_RIGHT)[stage-4])
        elif stage==8:press(g.KEY_2);assert self.model_name=='ppo-pid'
        elif stage==9:press(g.KEY_A);assert g.KEY_A in self.held
        elif stage==10:
            self.on_key(self.window,g.KEY_A,0,g.RELEASE,0);assert g.KEY_A not in self.held
            # A real person may interact with this visible smoke-test window.
            # Check release of OUR key, not absence of every other live key.
            press(g.KEY_SPACE)
        elif stage==11:
            press(g.KEY_R);q=self.plant.data.qpos.copy();v=self.plant.data.qvel.copy()
            press(g.KEY_R);np.testing.assert_array_equal(q,self.plant.data.qpos);np.testing.assert_array_equal(v,self.plant.data.qvel)
        elif stage==12:press(g.KEY_P);assert self.paused
        elif stage==13:press(g.KEY_P);assert not self.paused
        elif stage==14:self.capture()
        print(f'PASS viewer callback stage={stage} model={self.model_name} load={self.payload:.2f} mu={self.mu:.2f}',flush=True)

    def run(self):
        g=self.glfw;self.started=time.perf_counter();next_control=self.started;next_render=self.started
        duration=(12. if self.args.viewer_test else self.args.duration);stage=0;error=None
        schedule=(.5,2.,2.5,3.,3.5,3.8,4.1,4.4,5.,5.5,6.5,7.,7.5,8.,9.)
        try:
            while not g.window_should_close(self.window):
                now=time.perf_counter();g.poll_events()
                if duration is not None and now-self.started>=duration:break
                if self.args.viewer_test and stage<len(schedule) and now-self.started>=schedule[stage]:
                    self.smoke(stage);stage+=1
                # Fixed control/physics ticks. Rendering never advances physics.
                while now>=next_control:
                    self.tick();next_control+=CONTROL_DT
                    if now-next_control>.25:
                        # Slow computer: retain fixed numerical dt, slow wall-time
                        # playback instead of skipping physics or giant catch-up.
                        next_control=now;break
                if now>=next_render:
                    self.render();g.swap_buffers(self.window);next_render=now+1/30
                time.sleep(.001)
            if self.args.viewer_test:assert stage==len(schedule) and self.frames>20
        except Exception:
            error=traceback.format_exc();raise
        finally:
            self.finish_segment()
            save_json(self.out/'viewer_summary.json',{'frames_rendered':self.frames,
                'callback_stages_completed':stage if self.args.viewer_test else None,'exception':error,
                'smoke_test_scope':'real native window/context/rendering; controls invoked through registered callbacks, not physical keyboard presses',
                'events':self.events})
            self.context.free();g.destroy_window(self.window);g.terminate()
        print('VIEWER_CLOSED frames=',self.frames,'callback_stages=',stage,'logs=',self.out,flush=True)
        return 0

def interactive_view(args):return LiveViewer(args).run()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--experiment',action='store_true');ap.add_argument('--check',action='store_true')
    ap.add_argument('--view',action='store_true');ap.add_argument('--viewer-test',action='store_true')
    ap.add_argument('--model',choices=MODELS,default='tzejun')
    ap.add_argument('--implementation',choices=IMPLEMENTATIONS,default='checkpoint')
    ap.add_argument('--seeds',type=int,default=15);ap.add_argument('--duration',type=float)
    ap.add_argument('--out',type=Path);ap.add_argument('--scenarios');ap.add_argument('--render-report',type=Path)
    a=ap.parse_args()
    if not 1<=a.seeds<=200:ap.error('seeds must be 1..200')
    if a.duration is not None and (not math.isfinite(a.duration) or a.duration<1):ap.error('duration must be >=1s')
    if a.check:self_test(a.out or ROOT/'experiment_robustness'/'verification');return 0
    if a.render_report:make_report(a.render_report);return 0
    if a.experiment:
        a.duration=a.duration or 12.
        if a.duration<9:ap.error('experiment duration must be >=9s for post-push steady-state metrics')
        return run_experiment(a)
    return interactive_view(a)

if __name__=='__main__':raise SystemExit(main())
