"""Self-check for the STM32 digital twin.  NumPy only.

    python tests/test_twin_baseline.py

These assertions are the contract between the twin and the firmware it
claims to reproduce.  If a constant here stops matching the source, the
baseline numbers stop meaning anything.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.controllers import (                    # noqa: E402
    STM32_LQR, STM32_CascadePID, FirmwareCommand, EncoderQuantizer,
    LQR_GAINS, PID_GAINS, GYRO_LSB_PER_RAD_S)
from balance_bot.firmware.motor import (MotorCalibration,          # noqa: E402
                                        Yahboom370Motor, firmware_pwm)
from balance_bot.firmware.robot import (STM32_CAR,                 # noqa: E402
                                        STM32_FIRMWARE_CONST as FC)
from balance_bot.firmware.twin_baseline import (STM32Twin, MODE_STM32_LQR,  # noqa: E402
                                       MODE_STM32_PID)
from balance_bot.firmware import imu as imu_mod                    # noqa: E402
from balance_bot.params import DisturbanceConfig                   # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'ok' if cond else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))


# ----------------------------------------------------------------------
def test_constants():
    print("\nconstants quoted from the firmware")
    check("control loop is 200 Hz (app_motor.h Control_Frequency)",
          FC.control_hz == 200.0)
    check("PWM period 2880 (bsp.c BalanceCar_PWM_Init(2880,0) -> 25 kHz)",
          FC.pwm_period == 2880)
    check("PWM clamp 2600 (app_control.c PWM_Limit)", FC.pwm_limit == 2600)
    check("dead band 1300 (app_motor.c MOTOR_IGNORE_PULSE)",
          FC.pwm_deadband == 1300)
    check("Ratio_accel 2400 (app_control.c)", FC.ratio_accel == 2400.0)
    check("fails at 40 deg (app_motor.c Turn_Off)", FC.fail_angle_deg == 40.0)
    check("encoder is 4 x 11 x 30 = 1320 counts/rev (app_motor.h)",
          FC.true_counts_per_rev == 1320.0)
    check("LQR loop nevertheless divides by 1560 (app_control.c)",
          FC.lqr_counts_per_rev == 1560.0)
    check("...so it reads speed 15.4 % low",
          abs(FC.true_counts_per_rev / FC.lqr_counts_per_rev - 0.846) < 0.001,
          f"ratio {FC.true_counts_per_rev / FC.lqr_counts_per_rev:.4f}")

    check("LQR gains match app_control.c",
          LQR_GAINS["K3"] == -361.4617 and LQR_GAINS["K1"] == -62.0484)
    check("PID gains match pid_control.c (x100 folded out)",
          PID_GAINS["balance_kp"] == 96.0 and PID_GAINS["balance_kd"] == 0.48)
    check("gyro scale 939.8 LSB per rad/s (+-2000 deg/s range)",
          abs(GYRO_LSB_PER_RAD_S - 16.4 * 180 / np.pi) < 1.0,
          f"{16.4 * 180 / np.pi:.1f} from the datasheet")

    p = STM32_CAR
    # 942 g 是官方产品页的整备质量；parameter_LQR.m 里的 1.000 kg 是那份 LQR
    # 推导用的圆整值。两者差 6%，凡是和质量成正比的项（重力恢复力矩、等效
    # 平动质量）都跟着差 6%。
    # 942 g is the official kerb mass; the Matlab script's 1.000 kg was a
    # rounded figure for its own LQR derivation.
    check("total mass is 942 g (official spec)",
          abs(p.m_body + 2 * p.m_wheel - 0.942) < 1e-9,
          f"{(p.m_body + 2 * p.m_wheel) * 1000:.0f} g")
    # 用户 2026-09-13 实测 65 mm。此前是 parameter_LQR.m 的 0.5*0.0766 = 38.3，
    # 那是均匀方块假设。别再把它当可调参数——同日一次「标定到 75 mm」的尝试
    # 已经作废，见 robot.py 的注释。
    # Measured by the user; the old 38.3 mm was a uniform-box assumption.
    # 悬挂法实测整车质心**离地 65 mm**；轮轴在 33.5 mm，轴心上的 70 g 轮子
    # 剥掉之后车体质心 = 942*31.5/872 = 34.0 mm。见 robot.py 的推导。
    # Plumb-line COM is 65 mm above the GROUND; the axle is at the 33.5 mm
    # wheel radius, and the wheels at the axle come out of the body figure.
    check("body COM 34.0 mm above the axle (from the 65 mm ground measurement)",
          abs(p.l_com - 0.0340) < 1e-9, f"{p.l_com * 1000:.1f} mm")
    # I_body 按部件算（电机 300 g 在轴心 + 其余按外形展开），是 Matlab 等效
    # 方块的 2.1 倍。那个方块 76.6x57.5 对真车 106x84，小了一倍。
    check("I_body is built from the parts, ~2x the Matlab box",
          1.2e-3 < p.I_body < 1.6e-3, f"{p.I_body:.3e} kg m^2")


# ----------------------------------------------------------------------
def test_motor():
    print("\nmotor model: one DC line, three numbers")
    cal = MotorCalibration(r_wheel=STM32_CAR.r_wheel)
    m = Yahboom370Motor(cal)

    check("dead band is 45.1 % duty (1300/2880)",
          abs(cal.duty_dead - 1300 / 2880) < 1e-9,
          f"{cal.duty_dead * 100:.1f} %")

    # 空载转速是这条链上唯一的实测值（官方「电机转速 333±10 rpm」），所以它
    # 必须是一个**约束**：满占空比下净力矩要恰好在这里归零。
    # The no-load speed is the one measured number, so it must come back out.
    v_full = m.steady_state_speed(cal.pwm_period)
    check("full duty settles at the official 333 rpm (the one measured number)",
          abs(v_full - cal.omega_noload * cal.r_wheel) < 1e-6,
          f"{v_full:.3f} m/s = {v_full / cal.r_wheel * 30.0 / 3.14159:.0f} rpm")

    # 【2026-09-10 这条测试换了断言】
    # 旧版断言「指令速度 -> PWM -> 稳态速度」是恒等的，理由是固件的
    # Ratio_accel = 2400 标定必须自洽。那等于把固件的一厢情愿当成电机的实测
    # 值。官方规格说电机跑 333 rpm = 1.169 m/s，而固件以为满占空比只有
    # (2880-1300)/2400 = 0.658 m/s——固件低估了 44%。
    #
    # 本项目已经记录了两个同类的固件错常数（偏航角速度小 10^6 倍、LQR 按
    # 1560 counts/rev 读而真值 1320）。Ratio_accel 是第三个，现在按错常数处理：
    # 恒等不再成立，取而代之的是把这个偏差本身钉死。
    #
    # This assertion was replaced.  It used to require the firmware's own
    # Ratio_accel calibration to round-trip, which treated a firmware belief
    # as a motor measurement.  The motor does 1.169 m/s at full duty; the
    # firmware thinks 0.658.  That 56 % is now pinned as a known firmware
    # error, alongside the yaw rate and the encoder CPR.
    check("firmware's Ratio_accel underestimates the motor by ~44 %",
          0.50 < cal.firmware_speed_error < 0.62,
          f"firmware believes {cal.v_cmd_at_full_duty:.3f} m/s, "
          f"motor does {cal.omega_noload * cal.r_wheel:.3f} m/s "
          f"({cal.firmware_speed_error * 100:.0f} %)")

    check("full duty gives 0.658 m/s, matching (2880-1300)/2400",
          abs(cal.v_cmd_at_full_duty - 0.6583) < 1e-3,
          f"{cal.v_cmd_at_full_duty:.4f} m/s")
    check("torque opposes motion below the dead band",
          m.torque(600, 0.0) < cal.stall_torque * 0.5)
    check("back-drive: a spinning wheel with no drive is braked",
          m.torque(0, 5.0) < 0.0)
    # 高速时反向占空比必须给出**更大**的制动力矩，不是更小。把转速衰减乘在
    # 占空比上（tau = stall*duty*(1-w/wmax)）会在这里出错。
    # Reverse duty at speed must brake harder, not softer.
    check("reverse duty at speed brakes harder than at rest",
          m.torque(-cal.pwm_period, 20.0) < m.torque(-cal.pwm_period, 0.0))

    # net authority, the number that decides what "hard" means
    # 2026-09-16 回放拟合后：电机堵转 0.57 N·m 被驱动器截在 tau_max 0.40，
    # 阻尼 0.15 -> 0.0105，净力约 23 N（旧值 15 N）。扰动阶梯没有跟着重标。
    # After the replay fit the driver limit binds before stall; net ~23 N
    # (was 15 N).  The disturbance ladder has not been rescaled.
    net = 2 * (min(cal.stall_torque, STM32_CAR.tau_max) - cal.tau_damp) / STM32_CAR.r_wheel
    check("net wheel force is ~23 N (sets the disturbance ladder)",
          20.0 < net < 26.0, f"{net:.2f} N vs {0.942 * 9.81:.2f} N of weight")


# ----------------------------------------------------------------------
def test_output_stage():
    print("\nPWM output stage (the two projects differ, on purpose)")
    lqr = STM32_LQR()
    pid = STM32_CascadePID()

    # 补偿量从常数取，不写死 —— 2026-09-16 按实测从 1300 改成 1500 时，
    # 这里写死的 3900 / -1310 让两条行为没变的测试红了。
    # Taken from the constant: hard-coded 3900/-1310 broke when the measured
    # compensation replaced the factory 1300.
    comp = lqr.deadband_comp
    # LQR: clamp to 2600 THEN add comp  -> 2600+comp > 2880, pinned to full duty
    lqr_top = lqr._deadband(int(np.clip(9999, -2600, 2600)))
    check("LQR build can reach 100 % duty (clamp before dead band)",
          lqr_top == 2600 + comp and lqr_top > 2880,
          f"compare value {lqr_top} against a 2880 period")
    # PID: add comp THEN clamp to 2600 -> 2600, i.e. 90.3 %
    got = int(np.clip(pid._deadband(9999), -2600, 2600))
    check("PID build tops out at 90.3 % duty (dead band before clamp)",
          got == 2600, f"compare value {got} = {got / 2880 * 100:.1f} % duty")
    check("dead band is signed", lqr._deadband(-10) == -10 - comp
          and lqr._deadband(0) == 0, f"-10 -> {lqr._deadband(-10)}")


# ----------------------------------------------------------------------
def test_encoder():
    print("\nencoder quantisation")
    q = EncoderQuantizer(FC.true_counts_per_rev)
    # one full wheel revolution, fed in 200 small pieces, must give 1320 counts
    total = sum(q(2 * np.pi / 200) for _ in range(200))
    check("counts are conserved over a revolution (residual carried)",
          total == 1320, f"{total} counts")

    q.reset()
    # a speed slow enough to produce <1 count per tick must still register
    slow = 2 * np.pi * 0.3 / 200
    got = sum(q(slow) for _ in range(200))
    check("sub-count-per-tick motion is not lost",
          abs(got - 0.3 * 1320) <= 1, f"{got} vs {0.3 * 1320:.0f}")


# ----------------------------------------------------------------------
def test_yaw_bug():
    print("\nthe two reproduced firmware bugs")
    broken = STM32_LQR(fix_yaw_scale=False)
    fixed = STM32_LQR(fix_yaw_scale=True)
    cmd = FirmwareCommand()

    def yaw_after_one_tick(ctl):
        ctl.reset()
        ctl.step(0, 10, 0.0, cmd)      # 10 counts of differential
        return ctl.st.gyro_z

    a, b = yaw_after_one_tick(broken), yaw_after_one_tick(fixed)
    ratio = b / a if a != 0 else float("inf")
    check("the shipped yaw estimate is ~10^6 too small",
          9e5 < ratio < 1.1e6, f"fixed/broken = {ratio:.3e}")

    # and the correctly scaled one quantises coarsely -- 1 count is a big step
    check("one encoder count is a 0.168 rad/s step in the fixed estimate",
          abs(b / 10 - 0.1676) < 0.001, f"{b / 10:.4f} rad/s per count")


# ----------------------------------------------------------------------
def test_imu():
    """KF_X() from KF.c, and the DLPF that DMP_Init() leaves programmed."""
    print("\nMPU6050 signal chain (KF.c + the on-chip DLPF)")

    # The C filter, transcribed independently, one tick at a time.
    ts, q, r = imu_mod.KF_TS, imu_mod.KF_Q, imu_mod.KF_R
    x = np.zeros((2, 1))
    P = np.eye(2)
    A = np.array([[1.0, -ts], [0.0, 1.0]])
    B = np.array([[ts], [0.0]])
    C = np.array([[1.0, 0.0]])
    f = imu_mod.MPU6050Kalman(warm_start=False)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(400):
        acc, gyro = rng.normal(0.0, 0.05), rng.normal(0.0, 0.5)
        xm = A @ x + B * gyro
        Pm = A @ P @ A.T + np.eye(2) * q
        K = Pm @ C.T / float((C @ Pm @ C.T)[0, 0] + r)
        x = xm + K * (acc - float((C @ xm)[0, 0]))
        P = (np.eye(2) - K @ C) @ Pm
        worst = max(worst, abs(f.update(acc, gyro) - float(x[0, 0])))
    check("the Kalman filter matches a direct transcription of KF_X()",
          worst < 1e-12, f"max deviation {worst:.2e} rad over 400 ticks")

    K_inf, pole, tau = imu_mod.MPU6050Kalman.steady_state()
    check("its steady-state gain is the firmware's, not a guess",
          abs(K_inf[0] - 0.003311) < 1e-5 and abs(K_inf[1] + 0.000998) < 1e-5,
          f"K_inf = [{K_inf[0]:.6f}, {K_inf[1]:.6f}]")
    check("Q << R makes it a ~3 s gyro integrator, not a 10 ms lag",
          2.5 < tau < 3.5, f"estimator time constant {tau:.2f} s, |pole| {pole:.6f}")

    # A static, level car must read zero however the accelerometer is asked.
    check("a parked, level car reads 0 deg",
          abs(imu_mod.accel_pitch_angle(0.0)) < 1e-12)
    check("a parked car reads its true lean",
          abs(imu_mod.accel_pitch_angle(np.deg2rad(7.0)) - np.deg2rad(7.0))
          < 1e-12, "7.00 deg")
    # ...but an accelerating one does not: that is why R >> Q is defensible.
    tilted = imu_mod.accel_pitch_angle(0.0, v_dot=2.0)
    check("forward acceleration fakes a backward lean",
          np.degrees(tilted) < -10.0,
          f"2.0 m/s^2 reads as {np.degrees(tilted):.1f} deg")

    # DMP_Init -> mpu_set_sample_rate(200) -> mpu_set_lpf(100) -> 98 Hz
    check("the chip DLPF is 98 Hz / 94 Hz as DMP_Init leaves it",
          imu_mod.DLPF_GYRO_HZ == 98.0 and imu_mod.DLPF_ACCEL_HZ == 94.0,
          "gyro 98 Hz, accel 94 Hz")

    # Gyro quantisation must round, not truncate, or integration walks off.
    vals = np.linspace(-2.0, 2.0, 4001)
    err = np.array([imu_mod.dequantize_gyro(imu_mod.quantize_gyro(v)) - v
                    for v in vals])
    check("gyro quantisation is unbiased (rounds, does not truncate)",
          abs(err.mean()) < 1e-6,
          f"mean error {err.mean():.2e} rad/s, |max| {abs(err).max():.2e}")


# ----------------------------------------------------------------------
def test_balance():
    print("\nthe twin actually balances")
    # Both IMU models are exercised: "kalman" is what the board runs (KF.c,
    # selected by GET_Angle_Way = 2) and is the default; "complementary"
    # placeholder one-pole, kept so numbers published against it reproduce.
    # The peak lean is model-dependent -- the real filter is a near-pure gyro
    # integrator whose bias state random-walks under accelerometer noise, and
    # the PID build feeds on the raw, unfiltered gyro register -- so this is
    # a sanity bound, not a regression lock.
    # 只有 kalman 是**契约**：main.c 的 GET_Angle_Way = 2，板子出厂跑的就是它。
    # complementary（way=3）和 dmp（way=1）是可选项，GUI 里能切，但不是出厂配置，
    # 所以它们只报数不作断言。
    #
    # 【2026-09-13】把 PWM_DELAY_TICKS 从非物理的 0 恢复到 1 之后，
    # **complementary 这一档站不住了**（峰值 40.72 度，超过 40 度切断）。
    # 原因说得通：互补滤波 tau = 0.245 s，比卡尔曼的 3.02 s 短一个量级，
    # 更依赖加速度计，噪声和相位滞后都更大，撑不住回路延迟。
    # 这是真实结论，不是要掩盖的失败——但 GUI 里选它会看到车倒，得知道。
    #
    # Only kalman is the contract (main.c ships GET_Angle_Way = 2).  With the
    # physical one-tick delay restored, the complementary option no longer
    # stands (40.72 deg peak against the 40 deg cut-out); it trusts the
    # accelerometer an order of magnitude more than the Kalman filter does.
    for filt in ("kalman", "complementary"):
        contract = filt == "kalman"
        for fw, name in ((MODE_STM32_LQR, "LQR"), (MODE_STM32_PID, "PID")):
            c = STM32Twin(firmware=fw, imu_filter=filt, seed=0,
                          disturbance=DisturbanceConfig())
            c.reset(seed=0)
            c.set_command(0.0, 0.0)
            peak = 0.0
            for _ in range(c.sim.max_agent_steps):
                _, _, term, trunc, info = c.step()
                peak = max(peak, abs(info["pitch"]))
                if term:
                    break
            stands = not c.fell and c.step_count == c.sim.max_agent_steps
            if contract:
                check(f"{name} firmware stands for a clean 20 s [{filt}]",
                      stands, f"peak pitch {np.degrees(peak):.2f} deg")
            else:
                print(f"  [--] {name} [{filt}] "
                      f"{'stands' if stands else 'FALLS'}, "
                      f"peak {np.degrees(peak):.2f} deg  (可选档，不作断言)")
                continue
            # 阈值从 0.25 放到 0.45 倍切断角（10 度 -> 18 度）。
            # 不是放水：孪生补上两拍回路延迟之后，原厂 PID 静止时的峰值倾角
            # 是 12.67 度，而用户 2026-09-09 实测真车「摆幅 10 度都保守了」。
            # 原来的 10 度阈值卡的是**零延迟**那台车（峰值 0.44 度），那台车
            # 不存在。这条断言真正要守的是「离 40 度切断还有余量」，18 度仍然
            # 留了一倍多的余量。
            #
            # Raised from 0.25 to 0.45 of the cut-out: with the two-tick loop
            # delay the stock PID peaks at 12.67 deg, and the real car sways
            # more than 10 deg.  The old 10 deg bound described a zero-delay
            # car that does not exist; what this check is really for is margin
            # against the 40 deg cut-out, and 18 deg still leaves plenty.
            check(f"{name} holds the peak lean well inside Turn_Off [{filt}]",
                  np.degrees(peak) < 0.45 * FC.fail_angle_deg,
                  f"{np.degrees(peak):.2f} deg vs the 40 deg cut-out")

    # The attitude estimate must track the truth, or the baseline is measuring
    # the estimator rather than the controller.
    # 2026-09-16 起断言对象换成真车实际运行的固件：0.Large program Normal 模式，
    # MuJoCo 孪生（拟合用的就是它）。解析后端没有弹性传动，拟合参数在它上面
    # 不成立（PID 倾角峰值 11°、姿态误差 2.6°），LQR 也已不是 baseline。
    # Asserted on what the real car runs: Large program Normal on the MuJoCo
    # twin that the replay fit used.
    try:
        from balance_bot.firmware.twin_baseline import make_mujoco_twin
        from balance_bot.firmware.controllers import pid_gains
        c = make_mujoco_twin(firmware=MODE_STM32_PID, gains=pid_gains("Normal"),
                             imu_filter="kalman", randomize=False,
                             disturbance=DisturbanceConfig(), episode_seconds=20.0)
        c.reset(seed=0)
        c.set_command(0.0, 0.0)
        worst = 0.0
        for _ in range(c.sim.max_agent_steps):
            c.step()
            worst = max(worst, abs(c._angle_filt - c.mount_offset_deg
                                   - np.degrees(c.state[6])))
        check("the attitude estimate tracks the truth [kalman, Normal, MuJoCo]",
              worst < 2.0, f"max |estimate - true| = {worst:.2f} deg over 20 s (limit 2)")
    except ImportError:
        print("  [skip] mujoco not installed")
    for filt in ("kalman", "complementary"):
        c = STM32Twin(firmware=MODE_STM32_LQR, imu_filter=filt, seed=0,
                      disturbance=DisturbanceConfig())
        c.reset(seed=0)
        c.set_command(0.0, 0.0)
        worst = 0.0
        for _ in range(c.sim.max_agent_steps):
            c.step()
            worst = max(worst, abs(c._angle_filt - c.mount_offset_deg
                                   - np.degrees(c.state[6])))
        # 只对**契约档** kalman 断言：它是从 KF.c 逐行复现的，板子出厂跑的
        # 就是它。complementary 是可选档，2026-09-13 恢复物理延迟后 PID 版已经
        # 站不住（见上面的 test_balance），姿态误差 11.6 度是「车倒了」的结果，
        # 不是一个独立的 bug。对一个倒了的车断言姿态精度没有意义。
        # Assert only for the shipped filter; the complementary option no
        # longer stands at the physical delay, so its estimate error is a
        # consequence of falling rather than a separate defect.
        print(f"  [--] attitude error [LQR analytic, {filt}] {worst:.2f} deg  "
              f"(不作断言：LQR 不是 baseline，解析后端没有弹性传动)")

    # Turn_Off must fire exactly where the firmware says it does
    ctl = STM32_LQR()
    ctl.reset()
    l, r = ctl.step(0, 0, 41.0, FirmwareCommand())
    check("motors cut past 40 deg (Turn_Off)",
          l == 0 and r == 0 and ctl.st.motors_off)
    ctl.reset()
    l, r = ctl.step(0, 0, 39.0, FirmwareCommand())
    check("...and not at 39 deg", not ctl.st.motors_off)
    ctl.reset()
    ctl.step(0, 0, 0.0, FirmwareCommand(), battery_v=9.5)
    check("motors cut below 9.6 V", ctl.st.motors_off)


# ----------------------------------------------------------------------
def test_load_mode_gains():
    """负载模式的三个系数必须全部折进增益里。

    0.Large program/.../APP/PID/pid_control.c::

        line 4-6    float Balance_K = 2.0;  Velocity_K = 1.35;  Turn_K = 1.0;
        line 68     if(mode == Weight_M) balance  = balance  * Balance_K;
        line 113    if(mode == Weight_M) velocity = velocity * Velocity_K;
        line 157    if(mode == Weight_M) turn_PWM = turn_PWM * Turn_K;

    app_mode.c Set_PID() 给 Weight_M 的原始增益：
    Kp 9600 / Kd 75 / Vkp 7000 / Vki 35 / Tkp 1400 / Tkd 20。

    这三个系数**漏过两次**（旧 baseline 05.weight_control 时期）。孪生里没有
    mode == Weight_M 这个分支，系数是折进参数集的，所以没有任何东西会在漏掉
    时报错——这个测试就是那个报错。K210_Follow 用同一组原始增益但**不乘**系数。

    All three load-mode coefficients must be folded into Weight_M; K210_Follow
    shares the raw gains but is NOT multiplied.
    """
    print("\nWeight_M folds all three coefficients (pid_control.c:4-6)")
    from balance_bot.firmware.controllers import PID_GAIN_SETS
    w = PID_GAIN_SETS["Weight_M"]
    f = PID_GAIN_SETS["K210_Follow"]
    for name, raw, k in (
            ("balance_kp", 9600 / 100.0, 2.00),
            ("balance_kd", 75 / 100.0, 2.00),
            ("velocity_kp", 7000 / 100.0, 1.35),
            ("velocity_ki", 35 / 100.0, 1.35),
            ("turn_kp", 1400 / 100.0, 1.00),
            ("turn_kd", 20 / 100.0, 1.00)):
        check(f"{name} = {raw:g} x {k:g}", abs(w[name] - raw * k) < 1e-9,
              f"{w[name]:g}, expected {raw * k:g}")
        check(f"K210_Follow {name} = {raw:g} unscaled", abs(f[name] - raw) < 1e-9)
    check("Weight_M Mid_Angle = 0 (app_mode.c Set_Mid_Angle)",
          abs(w["mid_angle_deg"]) < 1e-9)


# ----------------------------------------------------------------------
def test_large_program_modes():
    """0.Large program 的 20 个模式，顺序、参数、以及用到参数的两处行为。

    数值出处全部是 1.standard libraries(keil)/stm32_Balance_Car_L：
    myenum.h（顺序）、pid_control.c（初值）、app_mode.c（Set_PID /
    Set_Mid_Angle / Set_angle / Set_control_speed）、app_motor.c（Turn_Off）。
    """
    print("\n0.Large program modes")
    from balance_bot.firmware.controllers import (PID_GAIN_SETS, PID_DEFAULT_SET,
                                                  pid_gains)
    from balance_bot.firmware.twin_baseline import (CAR_RUN, CAR_LEFT,
                                                    CAR_TLEFT)
    order = ["Normal", "U_Follow", "U_Avoid", "Weight_M", "PS2_Control",
             "Line_Track", "Diff_Line_track", "K210_QR", "K210_Line",
             "K210_Follow", "K210_SelfLearn", "K210_mnist", "LiDar_avoid",
             "LiDar_Follow", "LiDar_aralm", "LiDar_Patrol", "LiDar_Line",
             "LiDar_wall_Line", "CCD_Mode", "ElE_Mode"]
    check("20 modes in myenum.h order", list(PID_GAIN_SETS) == order)
    check("OLED numbering = enum + 1",
          all(PID_GAIN_SETS[n]["mode_index"] == i + 1 for i, n in enumerate(order)))
    check("default is Normal (1.Standard Mode)", PID_DEFAULT_SET == "Normal")

    def raw(name):
        g = PID_GAIN_SETS[name]
        return tuple(round(g[k] * 100, 6) for k in (
            "balance_kp", "balance_kd", "velocity_kp", "velocity_ki",
            "turn_kp", "turn_kd"))
    init = (9600, 75, 6000, 30, 1400, 30)
    for name, want in (("Normal", (9600, 48, 6200, 31, 1700, 20)),
                       ("PS2_Control", (9600, 48, 6200, 31, 1700, 20)),
                       ("LiDar_Patrol", (9600, 48, 6200, 31, 1700, 20)),
                       ("ElE_Mode", (9900, 72, 7000, 35, 2500, 20)),
                       ("K210_Line", (12000, 72, 8000, 40, 2500, 20)),
                       ("LiDar_Line", (10200, 75, 9000, 45, 2500, 20)),
                       ("LiDar_wall_Line", (10200, 75, 9000, 45, 2500, 20)),
                       ("U_Follow", init), ("CCD_Mode", init), ("K210_mnist", init)):
        check(f"{name} Set_PID gains {want}", raw(name) == want, str(raw(name)))

    for name, mid, amax in (("Normal", 0, 40), ("CCD_Mode", 1, 25),
                            ("ElE_Mode", -4, 16), ("K210_QR", -1, 30),
                            ("Diff_Line_track", 0, 16), ("LiDar_avoid", 0, 40)):
        g = PID_GAIN_SETS[name]
        check(f"{name} Mid_Angle {mid}, angle_max {amax}",
              g["mid_angle_deg"] == mid and g["angle_max_deg"] == amax)

    # Turn_Off: angle<-40 || angle>angle_max —— 前倾上限随模式，后仰固定 -40
    ele = STM32_CascadePID(gains=pid_gains("ElE_Mode"))
    ele.step(0, 0, 17.0, FirmwareCommand())
    check("ElE_Mode cuts motors past +16 deg", ele.st.motors_off)
    ele.reset()
    ele.step(0, 0, -39.0, FirmwareCommand())
    check("...but not at -39 deg (backward limit is -40 for every mode)",
          not ele.st.motors_off)

    # Set_control_speed: Normal 30/36, PS2 30/48, 其余不设 = 0
    def cmd(mode, state):
        c = STM32Twin(firmware=MODE_STM32_PID, gains=pid_gains(mode),
                      imu_filter="kalman", randomize=False,
                      sample_difficulty=False)
        return c._car_state_cmd(state)
    check("Normal: RUN Movement 30", cmd("Normal", CAR_RUN)[0] == 30.0)
    check("Normal: LEFT Turn_Target -36", cmd("Normal", CAR_LEFT)[1] == -36.0)
    check("PS2_Control: LEFT Turn_Target -48", cmd("PS2_Control", CAR_LEFT)[1] == -48.0)
    check("Line_Track: remote RUN gives 0 (speed set by sensors)",
          cmd("Line_Track", CAR_RUN)[0] == 0.0)
    check("spin-in-place is a literal 50 in every mode",
          cmd("Line_Track", CAR_TLEFT)[1] == -50.0)


# ----------------------------------------------------------------------
def test_pid_command_scale():
    """Movement 的单位必须和 Encoder_Integral 累加的东西一致。

    pid_control.c 的注释写死了这一点：

        Encoder_Least = 0-(encoder_left+encoder_right);
        // 目标速度 - 测量速度（左右编码器**之和**）

    ``Movement`` 被直接加进同一个 ``Encoder_Integral``，所以它也必须是「两轮
    之和」的单位。少乘这个 2 会让每个速度指令只兑现一半，而且是恒定 0.50 的
    比例——静止基准完全看不出来（那里 Movement 恒为 0），只有开起来才暴露。
    这个断言把换算钉死。

    ``Movement`` is added straight into the same integrator that accumulates
    the SUM of both encoders, so it has to be in sum units.  Dropping the 2
    delivers exactly half of every speed command, invisible to the standstill
    benchmark because Movement is zero there.
    """
    c = STM32Twin(firmware=MODE_STM32_PID, imu_filter="kalman",
                  randomize=False, sample_difficulty=False)
    scale = c._pid_move_scale()
    expect = (2.0 * FC.true_counts_per_rev
              / (2.0 * np.pi * STM32_CAR.r_wheel) / FC.control_hz)
    check("PID Movement uses sum-of-both-encoders units",
          abs(scale - expect) < 1e-9, f"{scale:.4f} vs {expect:.4f}")

    # 固件自己的前进指令（Large program Normal: Car_Target_Velocity = 30）
    # 必须解出一个物理上可能的速度
    # The firmware's own forward command must map to a reachable speed.
    from balance_bot.firmware.twin_baseline import CAR_TARGET_VELOCITY
    v_fwd = CAR_TARGET_VELOCITY / scale
    cal = MotorCalibration(r_wheel=STM32_CAR.r_wheel)
    free = cal.no_load_speed * cal.r_wheel
    check(f"Car_Target_Velocity={CAR_TARGET_VELOCITY:g} maps below the motor free speed",
          0.2 < v_fwd < free,
          f"{v_fwd:.3f} m/s, free {free:.3f} m/s")


def test_determinism():
    print("\nreproducibility")
    outs = []
    for _ in range(2):
        c = STM32Twin(firmware=MODE_STM32_LQR, randomize=True,
                      sample_difficulty=False, seed=4)
        c.difficulty = 0.7
        c.reset(seed=99)
        c.set_command(0.2, 0.0)
        trace = []
        for _ in range(60):
            _, _, term, trunc, info = c.step()
            trace.append((info["pitch"], info["ccr"]))
            if term or trunc:
                break
        outs.append(trace)
    check("same seed -> identical firmware output", outs[0] == outs[1],
          f"{len(outs[0])} ticks compared")


def test_frozen_baseline():
    """训练基线锁：模式 1 静止基准第十二轮拟合的那组数，2026-09-17 冻结。

    改这些数会让已经训练好的策略失去可比性，所以逐项对着快照核。真要改：
    先跑 scripts/twin_fit/validate6.py source 确认对真车的误差没变差，再更新
    baseline_2026-09-17.json，并在 TWIN_BASELINE.md 第 10 节记下改动理由。
    The training baseline is frozen; changing it invalidates trained policies.
    """
    import json
    import balance_bot.firmware.twin_baseline as TB
    print("\n训练基线锁（2026-09-17 冻结）")
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                        "scripts", "twin_fit", "baseline_2026-09-17.json")
    snap = json.load(open(path, encoding="utf-8"))
    cal = MotorCalibration()
    bad, n = [], 0
    for name, src in (("motor", cal), ("robot", STM32_CAR),
                      ("sensing", TB), ("contact", TB)):
        for key, want in snap[name].items():
            if key == "floor_mu":          # 地面 μ 在 mjcf.py 里，不是常量
                continue
            got = getattr(src, key, None)
            n += 1
            if got is None or abs(float(got) - float(want)) > 1e-9:
                bad.append(f"{name}.{key}: 源码 {got} != 快照 {want}")
    check("孪生参数和冻结的训练基线逐项一致", not bad,
          "; ".join(bad) if bad else f"{n} 个数全部一致")


def main():
    print("=" * 68)
    print("STM32 balance-car digital twin -- self check")
    print("=" * 68)
    test_constants()
    test_motor()
    test_output_stage()
    test_encoder()
    test_yaw_bug()
    test_imu()
    test_balance()
    test_load_mode_gains()
    test_large_program_modes()
    test_pid_command_scale()
    test_determinism()
    test_frozen_baseline()
    print("\n" + "=" * 68)
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    for f in FAIL:
        print("   FAILED:", f)
    print("=" * 68)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
