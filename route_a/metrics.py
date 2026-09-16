"""Metrics computed exclusively from deployable telemetry."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from statistics import fmean
from typing import Any, Sequence

from .telemetry import FastTelemetry


@dataclass(frozen=True)
class PerformanceMetrics:
    pitch_rms_deg: float
    pitch_peak_deg: float
    pitch_rate_rms_deg_s: float
    settling_time_s: float | None
    steady_state_pitch_error_deg: float
    speed_tracking_rmse_m_s: float
    speed_overshoot_m_s: float
    yaw_tracking_rmse_deg_s: float
    wheel_mismatch_m_s: float
    pwm_saturation_ratio: float
    control_effort_rms_pwm: float
    fall_count: int
    protection_trigger_count: int
    telemetry_loss_ratio: float
    telemetry_staleness_s: float
    sample_count: int
    residual_speed_m_s: float = 0.0
    longitudinal_displacement_m: float = 0.0
    longitudinal_excursion_m: float = 0.0

    def as_dict(self) -> dict[str, Any]: return asdict(self)


SCORE_WEIGHTS = {
    "balance": {"pitch": 4.0, "peak": 1.5, "rate": 0.08, "speed": 0.15, "yaw": 0.01, "effort": 0.002},
    "velocity": {"pitch": 2.0, "peak": 1.0, "rate": 0.04, "speed": 12.0, "yaw": 0.01, "effort": 0.0015},
    "turn": {"pitch": 2.0, "peak": 1.0, "rate": 0.04, "speed": 2.0, "yaw": 0.12, "effort": 0.0015},
}


def _rms(values: Sequence[float]) -> float:
    return math.sqrt(fmean(value*value for value in values)) if values else math.inf


def _count_rising(samples: Sequence[FastTelemetry], bit: int) -> int:
    result = 0; previous = False
    for sample in samples:
        current = bool(sample.flags & bit)
        if current and not previous: result += 1
        previous = current
    return result


def compute_metrics(samples: Sequence[FastTelemetry], quality: dict[str, Any], *,
                    target_speed_m_s: float = 0.0, target_yaw_deg_s: float = 0.0,
                    mid_angle_deg: float = 1.0, targets_from_command: bool = False) -> PerformanceMetrics:
    if not samples:
        return PerformanceMetrics(
            pitch_rms_deg=math.inf, pitch_peak_deg=math.inf, pitch_rate_rms_deg_s=math.inf,
            settling_time_s=None, steady_state_pitch_error_deg=math.inf,
            speed_tracking_rmse_m_s=math.inf, speed_overshoot_m_s=math.inf,
            residual_speed_m_s=math.inf, longitudinal_displacement_m=math.inf,
            longitudinal_excursion_m=math.inf, yaw_tracking_rmse_deg_s=math.inf,
            wheel_mismatch_m_s=math.inf, pwm_saturation_ratio=math.inf,
            control_effort_rms_pwm=math.inf, fall_count=0, protection_trigger_count=0,
            telemetry_loss_ratio=float(quality.get("loss_ratio", 1.0)),
            telemetry_staleness_s=math.inf, sample_count=0)
    pitch_error = [sample.pitch_deg-mid_angle_deg for sample in samples]
    pitch_rate = [sample.pitch_rate_deg_s for sample in samples]
    # E1F contains the encoder sum for its own fixed 100 ms producer window.
    # A missing UART frame loses that window; it must not stretch the next one.
    tick_periods = [0.1]*len(samples)
    distance_per_count = (math.pi*0.067)/1320
    left_speed = [sample.encoder_left*distance_per_count/dt for sample,dt in zip(samples,tick_periods)]
    right_speed = [sample.encoder_right*distance_per_count/dt for sample,dt in zip(samples,tick_periods)]
    speed = [(left+right)/2 for left,right in zip(left_speed,right_speed)]
    distance_steps = [(sample.encoder_left+sample.encoder_right)*0.5*distance_per_count for sample in samples]
    positions=[]; position=0.0
    for distance in distance_steps:
        position += distance; positions.append(position)
    if targets_from_command:
        speed_targets=[0.18 if sample.command == 1 else (-0.18 if sample.command == 2 else 0.0) for sample in samples]
        yaw_targets=[45.0 if sample.command in (4,6) else (-45.0 if sample.command in (3,5) else 0.0) for sample in samples]
    else:
        speed_targets=[target_speed_m_s]*len(samples); yaw_targets=[target_yaw_deg_s]*len(samples)
    speed_error = [value-target for value,target in zip(speed,speed_targets)]
    yaw_error = [sample.yaw_rate_deg_s-target for sample,target in zip(samples,yaw_targets)]
    effort = [math.sqrt((sample.pwm_left**2+sample.pwm_right**2)/2) for sample in samples]
    settling_time = None
    for index, error in enumerate(pitch_error):
        if abs(error) <= 2.0 and all(abs(item) <= 2.0 for item in pitch_error[index:]):
            settling_time = (samples[index].tick_ms-samples[0].tick_ms)/1000.0; break
    tail_count = max(1, len(samples)//4)
    tick_gaps = [(b.tick_ms-a.tick_ms)/1000.0 for a,b in zip(samples,samples[1:])]
    staleness = max(tick_gaps, default=0.0)
    overshoot = max((abs(value)-abs(target) for value,target in zip(speed,speed_targets)), default=0.0)
    residual_speed = abs(fmean(speed[-tail_count:]))
    displacement = positions[-1]
    excursion = max(positions)-min(positions)
    return PerformanceMetrics(
        _rms(pitch_error), max(map(abs,pitch_error)), _rms(pitch_rate), settling_time,
        abs(fmean(pitch_error[-tail_count:])), _rms(speed_error), max(0.0,overshoot),
        _rms(yaw_error), fmean(abs(a-b) for a,b in zip(left_speed,right_speed)),
        sum(1 for sample in samples if abs(sample.pwm_left)>=2600 or abs(sample.pwm_right)>=2600)/len(samples),
        _rms(effort), _count_rising(samples,1), _count_rising(samples,8),
        float(quality.get("loss_ratio",0.0)), staleness, len(samples),
        residual_speed, displacement, excursion)


def score_metrics(metrics: PerformanceMetrics, stage: str) -> float:
    w = SCORE_WEIGHTS[stage]
    penalty = (w["pitch"]*metrics.pitch_rms_deg + w["peak"]*metrics.pitch_peak_deg +
               w["rate"]*metrics.pitch_rate_rms_deg_s + w["speed"]*metrics.speed_tracking_rmse_m_s +
               w["yaw"]*metrics.yaw_tracking_rmse_deg_s + w["effort"]*metrics.control_effort_rms_pwm)
    penalty += 40*metrics.pwm_saturation_ratio + 60*metrics.telemetry_loss_ratio
    if stage == "balance":
        penalty += 2.0*metrics.residual_speed_m_s + 5.0*max(0.0, metrics.longitudinal_excursion_m-0.35)
    penalty += 100*metrics.fall_count + 50*metrics.protection_trigger_count
    return max(-1000.0, 100.0-penalty)
