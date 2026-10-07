# -*- coding: utf-8 -*-
"""切换器 v4 = v3（负载判定锁存）+ **大倾角恢复模式**。

思路（用户 2026-09-30）：冲击工况都可以归结为「被打到一个大倾角 + 大角速度
的状态，再从那里回来」。所以不去识别冲击本身，而是把**大倾角回稳能力**做到
物理上限附近，冲击就一起解决了。

实测支持这个思路（修好积分器的孪生，推力 0.12 s 加在车体质心）：
    空载 8 N 推完瞬间 ≈ (20°, 290°/s)，三种控制器峰值 40.3-40.8° —— 差不到 1°
    就能挺过去；而 v3 的回稳包络在 (20°, 200°/s) 能救、(20°, 400°/s) 不能。
    8 N 正好落在包络边上。

恢复律来自 recovery_bound.py 的「物理上限」实验：有界输入下时间最优是
bang-bang，这里用它的饱和线性版本：
    θ_pred = (θ_filt - θ_ref) + S·ω          预测 S 秒后倒到多少度
    a      = clip(K·θ_pred, -1, 1)           K 大 -> 近似纯 bang-bang

**只用固件拿得到的量**（滤波倾角 _angle_filt、陀螺 _gyro_lsb），不读真值。
θ_ref 是平稳时滤波倾角的慢低通：自动扣掉安装零偏（站立 ~1.9°）和坡上驻位
的正常倾斜（带载 ±4.8°，见 BUGS.md 第十节）—— 否则坡上一站就误触发。

触发 |θ_pred| > REC_ON_DEG；退出 |θ_err| < REC_OFF_DEG 且 |ω| < REC_OFF_DPS
连续 REC_OFF_TICKS 拍，交还 v3。v3 在恢复期间照常运行（抖振读数、PID 内部
状态都继续更新），所以交还是连续的。

证据列 rec_ticks：恢复模式开了多少拍。**正常工况必须是 0** —— 和 L2 的门
同一个道理：先证明它在该开的时候开、平时是安静的。

    python scripts/rl/switch_v4.py     # 自检：正常站立不触发，推一下会触发
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for q in (ROOT, HERE):
    if q not in sys.path:
        sys.path.insert(0, q)

import switch_v3 as SV3                                          # noqa: E402

# --- 恢复模式参数（由 tune_v4.py 定；这里是初值）---
REC_S = 0.06          # 预测时长（秒）：切换线 θ + S·ω = 0 的斜率
REC_K = 1.0           # 每度多少满量程；1.0 = 偏 1° 就满出力
REC_ON_DEG = 15.0     # |θ_pred| 超过就进恢复
REC_OFF_DEG = 3.0     # 退出：|θ_err| 小于
REC_OFF_DPS = 60.0    #       且 |ω| 小于
REC_OFF_TICKS = 4     #       连续这么多拍
REF_TAU = 2.0         # 平衡角慢低通的时间常数（秒）
REF_CALM_DEG = 4.0    # 只在 |θ_err| 小于这个、且不在恢复时更新平衡角

# --- 按负载类别分开的恢复参数（2026-10-01）---
# (S, K, ON)；ON = inf 表示这一类不用恢复模式、保持 v3 行为。
# None = 用上面的 REC_S/REC_K/REC_ON_DEG（单一参数，旧行为）。
#
# 为什么分开：同一套参数在两类上效果相反（同一 19 项测试，v3 vs v4）：
#   带载：2kg 推 15N 0/4 -> 4/4，峰值普遍低 5-10°
#   空载：倾角 (20°,300°/s) 2/2 -> 0/2（一进全力出力就冲过头）
# 根因是速度上限：空载被推 8N 后车速已到 0.85 m/s（电机极限 1.17 m/s 的
# 73%），轮速峰值到空载转速的 91%，反电动势把力矩压到接近零 —— 这时
# 全力出力帮不上忙；带载同样推力换来的车速小得多，还有余量。
# tune_v4_class.py（runs_e2e/tune_v4_class.log），37 组候选含「关闭」，每类单独排：
#   轻载：「关闭」最好 7/14，任何恢复参数只有 3-5/14。空载推 5N 能扛、6N 所有
#         候选都救不回 —— 边界是电机转速，不是控制律。
#   重载：S 0.10 K 2.0 ON 16° 4/10，关闭 2/10；差别在 2kg 推 15N（0/2 -> 2/2）。
#   组合后正常停-动-停 8 局（0/1/2/4 kg x 两种开局）恢复模式 0 拍。
#
# 【2026-10-01 改为 stiff】用户思路：升档和受冲击是同一件事（倾角过大 = 增益不够），
# 冲击后硬档自激、再由抖振探测降回来。tune_v4_state.py（runs_e2e/tune_v4_state.log），
# 32 组 (S, 静止门限, 运动门限) 全部 stiff、两类负载同一套：
#   S 0.10 静 3° 动 12/16/20°（三者打平）推击得分 24，v3 14；误升档 0、正常 8 局全完成、
#   空载抖振和 v3 相同（5.9/0.5）。
#     静止推：空载 6 N 0/4 -> 4/4（原厂 HEAVY 2/4）；2 kg 15 N 0/4 -> 2/4（HEAVY 4/4）
#     边走边推：2 kg 12 N 0/4 -> 4/4（运动门限有限时；「运动不触发」则 0/4）
#   S 0.06 全部救不了空载 6 N —— 预测不够远，触发晚了。
# ⚠️ 静止门限 3° 对孪生里静止日常最大 2.1° 只有 1.4 倍余量；真车站立抖振比孪生
# 强约 5 倍（PARAMS.md G 节），上车前必须按实车重定，否则会不停误升档。
REC_LIGHT = (0.10, 1.0, 3.0, "stiff")
REC_HEAVY = (0.10, 1.0, 3.0, "stiff")
# 类别判定：锁存后 f 小于这个算轻载。锁存前（还在探测）按轻载处理。
CLASS_SPLIT_F = 0.26

# --- 触发门限随运动状态切换（2026-10-01）---
# 参数元组里的 ON 用于「静止」（速度指令为 0 已满 STILL_S 秒）；运动中或刚停
# 用 REC_ON_MOVE。依据（空载，S=0.10，|θ_pred| 最大值）：
#     行驶中 8.0°   刚停 <1 s 9.1°   **静止 2.1°**
#     推 6 N 后：20 ms 5.0°   30 ms 9.5°   40 ms 13.3°
# 「指令在变」和「匀速」一样都是 8°，按指令分不开；按静止/运动分得开。
# 单一门限时：够早（能救 6 N）就在行驶中不停误升档（8 局 1231-12228 次），
# 车一直停在硬档上，空车抖振 33.7 = 退化成原厂 HEAVY。
REC_ON_MOVE = 16.0            # 运动中/刚停的门限；inf = 运动中不触发
STILL_S = 1.0

# --- 上膛：每次锁存或升档后，触发先失效，平稳一段时间才重新生效 ---
# 否则「降档」那一刻硬档自激还没衰减完，θ_pred 立刻超门限又升回去：
# 实测空车没被推也「锁存 0.12 -> 升档 -> 再锁存 -> 再升档」循环，12 s 里 9 次。
# 和固件 mode 26 换档后的锁定期（LA_LOCK_S）同一个思路。
ARM_DEG = 2.5     # |θ_pred| 小于这个算平稳（静止日常最大 ~2°）
ARM_S = 0.2       # 连续平稳这么久才上膛
# 0.5 -> 0.2（2026-10-01）：2 kg 要 2.5-2.9 s 才锁存，再等 0.5 s 上膛，站 3 s 就推的
# 那几局正好落在空窗里 —— 3/4 没升档就摔了。上膛计数本来就要求平稳，缩短不会
# 让硬档残余振荡钻空子（由下面的误升档复查确认）。

# --- f > 1 紧急档（2026-10-01，用户要求）---
# 倾角/角速度过大时把增益推到比 HEAVY 还硬（f>1 外推），尽量逼近物理极限；
# 平稳后回到 f=1（抖振判定的标定点）、解除锁存重新探测，由探测降档。
# 依据：推击里原厂 HEAVY 比软增益扛得多（空载 6 N 2/4 vs 0/4），而 stiff 升档
# 只能升到 f=1。触发后紧急档期间**不判负载**（探测窗口清零），因为抖振和
# 负载的对应只在 f=1 标定过。
# (S, 门限°, F_EMERG, 速度环外推比例 0..1, 最短保持 s)；None = 关闭
#   速度环外推比例 0：只外推平衡环 kp/kd，速度环停在 HEAVY；1：一起外推。
#   转向环不外推。
# 【结论 2026-10-01：默认关闭】f>1 在所有测试里都没有改善：
#   推击：空载 7/8 N、2 kg 18 N 加不加紧急档摔倒时刻精确到 10 ms 都相同
#         （电机输出早已顶满 ±2800，增益再硬输出也一样）；
#   大倾角边界 9 格（0kg 30°/200、15°/400、20°/400、35°/200、38°/100；
#         2kg 20°/100、15°/200、25°/0、10°/300）：F2/F3、速度环外推与否，
#         救回数与关闭相同或更差（scratchpad/emerg_tilt.py）。
#   到这些状态时剩下的约束是电机力矩/转速，不是增益 —— v4 已顶到孪生的物理极限。
EMERG = None

# --- 2026-10-01 第二轮：平衡角双速跟踪 / 静止门限自适应 / 小事件快速撤回 ---
# 平衡角：行驶中和刚停不满 STILL_S 时快跟（不设幅度门），静止满 STILL_S 后慢跟
# （设 REF_CALM_DEG 门，推击不会被吸收进去）。原来一律慢跟（2 s）且有 4° 门：
# 坡上起停时平衡姿态变了，停车 1 s 启用静止门限时 th_ref 还没跟上（带载坡上的
# 驻位偏移本身就 ±4.8°，被门挡住永远跟不上），于是误升档 —— 全表「空载上坡 8°」
# v4 抖振 5.6、v3 只有 0.4。
REF_TAU_FAST = 0.25
# 静止门限 = max(元组里的 ON（下限）, NOISE_K x 静止时 θ_pred 的均方根噪声)。
# 依据：随机化孪生（每个种子一台不同的车）里空载 v4 抖振 3.0-4.4、v3 只有 1.5 ——
# 有些车本身振动大，固定 3° 门限误升档。真车站立抖振比孪生强 3-5 倍，同一个问题
# 只会更严重。让门限跟着本车的噪声走，就不全靠实车标定。NOISE_K=0 关闭。
# 2026-10-01 tune_v4_r2.py（19 候选 x 标称+随机化）：下限 3° + K=6 时正常行驶
# 15/15 局零误升档、随机化空载抖振 1.55（= v3），推击得分 27/32（v3 13，
# 第二轮前 23）。K=4 有 3-4 次误升档；下限 2° 时随机化抖振 2.2-8.8。
NOISE_K = 6.0
NOISE_TAU = 2.0
# 静止门限上限（2026-10-01 random_field）：正常运行里自适应门限最高见到 8.7°。
NOISE_CAP = 8.0
# 升档后 QR_T 秒检查：期间「低通角速度」算的预测倾角峰值 < QR_PEAK -> 是误触发或
# 轻碰，恢复升档前的锁存档位，不等 1.2 s 重新探测（那段时间空车在硬档上抖）。
# 低通（OMLP_HZ）是为了滤掉硬档自激，只看车身真实的大幅运动。QR_PEAK=None 关闭。
# 依据：回稳包络里 v4 小扰动回稳 1.31 s（5° 静止起），v3 只要 0.41 s。
QR_T = 0.3
QR_PEAK = 6.0      # tune_v4_r2：静止 5° 小扰动回稳 1.50 s -> 0.60 s；10° 时撤回更多、无额外收益
OMLP_HZ = 3.0

# --- 2026-10-01 第三轮：「任何情况都自洽收敛」---
# 原则：负载只在 f=1 判（轻/中档下站立抖振、平衡角、速度环积分对 0-4 kg 全重叠，
# scratchpad/load_signal.py）；负载可能变了或档位可能不够 -> 回 f=1 重判。
# (1) 停稳门：停车后先连续平稳 ARM_S 才启用静止门限，之后保持到下次起步。
#     坡上停车满 1 s 时车还在晃、th_ref 还落后 ~1.7°，第 202/208 拍误升档
#     （drive_test 空载 8° 坡驻停抖振 2.9，v3 1.1）。
#     采用（tune_v4_r3）：坡上停车误升档 3->0、驻停抖振 2.9->1.1，其他不变。
#     急刹（hard_brake.py）：第二轮 v4 在 0.6 m/s 急刹后同样误升档（空载 6/8）——
#     刹车后 1 s 内 θ_pred 峰值只有 10-12°，不是行驶门限的问题，而是满 1 s 切
#     静止门限时车还在晃，和坡上停车同一根因。
STOP_CALM = True
# 0.2 s 时急刹仍误升档 5/144（平稳 0.2 s 后又晃到 3.28° > 3.18°），0.4 -> 2，
# 0.6 -> 0；停后 0.3 s 被推的保护不变（那段由行驶门限 16° 管）。
STOP_CALM_S = 0.6     # 停车后要连续平稳这么久（|θ_pred| < ARM_DEG）
# (2) 停车复查：锁在轻档时，每次行驶 ≥ REPROBE_DRIVE_S 后停稳 -> 回 f=1 重判。
#     轻档带重物是唯一会摔的错判（f=0.12 时 2 kg 2/6 站不住、4 kg 全摔）；
#     锁在中档卸了载只是偏硬，照样收敛，不复查。
#     代价（tune_v4_r3）：空车每次停车在硬档上重判，停车收敛 1.2 -> 2.9 s、驻停抖振
#     1.2 -> 6。收益：行驶中加载再停，末档判对 1/6 -> 6/6。默认关，待用户定。
REPROBE_ON_STOP = False
REPROBE_DRIVE_S = 1.0
# (3) 重载类行驶门限（None = 同 REC_ON_MOVE）。2 kg 行驶中推 8 N 摔 1/4（HEAVY 4/4）。
#     不采用：6° 救回 2 kg 行驶推 8N（4/6->6/6）但正常行驶误升档 18 次、急刹 2 kg
#     0.45 m/s 8/8 误升档；8° 救 5/6、误升档 7；10/12° 救不回。带载行驶中
#     「被推」和「正常开/急刹」按倾角分不开。
REC_ON_MOVE_HEAVY = None


class SwitchV4(SV3.SwitchV3):
    def __init__(self, gain_hook=None):
        super().__init__(gain_hook=gain_hook)
        self.rec = False
        self.rec_ticks = 0            # 证据列
        self.rec_entries = 0
        self.calm = 0
        self.th_ref = None
        self.th_pred = 0.0
        self.n_cmd_still = 0
        self.arm_cnt = 0
        self.armed = False
        self._was_latched = False
        self.emerg = False
        self.emerg_n = 0
        self.emerg_calm = 0
        self.emerg_ticks = 0          # 证据列
        self.emerg_entries = 0
        self.f_max_seen = 0.0
        self.om_lp = 0.0
        self.noise = 0.0              # 静止时 θ_pred² 的慢平均
        self.qr_active = False
        self.qr_n = 0
        self.qr_peak = 0.0
        self.qr_prev = 1.0
        self.qr_reverts = 0           # 证据列：快速撤回了几次
        self.on_still = 0.0           # 证据列：当前静止门限
        self.stop_calm = False
        self.stop_calm_n = 0
        self.n_drive = 0              # 本段行驶拍数
        self.stop_probes = 0          # 证据列：停车复查次数

    def _gains(self):
        """f ≤ 1 同 v3；f > 1 外推：平衡环按 f，速度环按 1+(f-1)·比例，转向环停在 1。"""
        f = self.f
        if f <= 1.0 or EMERG is None:
            return super()._gains()
        vel_frac = float(EMERG[3])
        out = {}
        for k in SV3.KEYS:
            if k in ("balance_kp", "balance_kd"):
                fk = f
            elif k in ("velocity_kp", "velocity_ki"):
                fk = 1.0 + (f - 1.0) * vel_frac
            else:
                fk = 1.0
            out[k] = SV3.G_N.get(k, 0.0) + (SV3.G_H.get(k, 0.0) - SV3.G_N.get(k, 0.0)) * fk
        return out

    def __call__(self, env, moving=None):
        from balance_bot.rl.e2e_env import FS
        # 触发判断放在 v3 之前：stiff 动作要在**这一拍**就用上硬增益，
        # 放在后面会晚 1 拍（5 ms）。只用固件拿得到的量。
        tw = env.core
        th = float(tw._angle_filt)
        om = float(tw._gyro_lsb) / 16.4
        self.om_lp += (1.0 - np.exp(-2.0 * np.pi * OMLP_HZ / FS)) * (om - self.om_lp)
        if self.th_ref is None:
            self.th_ref = th
        err = th - self.th_ref
        s_, k_, on_, act_ = self._params()
        # 速度指令为 0 已满 STILL_S 才算静止（固件自己知道指令）；否则用运动门限
        cmd_still = abs(float(env.v_ref)) < 1e-6 and abs(float(getattr(env, "yaw_ref", 0.0))) < 1e-6
        self.n_cmd_still = self.n_cmd_still + 1 if cmd_still else 0
        still = self.n_cmd_still >= int(STILL_S * FS)
        self.th_pred = err + s_ * om
        if STOP_CALM or REPROBE_ON_STOP:
            if not cmd_still:
                self.n_drive += 1
                self.stop_calm, self.stop_calm_n = False, 0
            elif not self.stop_calm:
                self.stop_calm_n = self.stop_calm_n + 1 if abs(self.th_pred) < ARM_DEG else 0
                if self.stop_calm_n >= int(STOP_CALM_S * FS) and still:
                    self.stop_calm = True
                    if (REPROBE_ON_STOP and self.n_drive >= int(REPROBE_DRIVE_S * FS)
                            and self.latched and self.f_latched < CLASS_SPLIT_F and not self.rec):
                        self._rearm()
                        self.stop_probes += 1
                    self.n_drive = 0
            if STOP_CALM:
                still = still and self.stop_calm
        # 静止门限自适应：已锁存、已上膛、静止、不在恢复中时估计噪声
        if NOISE_K > 0.0:
            # 只从「门限以内」的样本学噪声，且门限封顶 NOISE_CAP —— 否则车真的歪了
            # （random_field 局 4：静止歪 -20° 好几秒）也被当成噪声，门限越学越高
            # （54 -> 118°），保护自己把自己关掉。超出门限的是事件，不是噪声。
            cur = max(on_, min(NOISE_CAP, NOISE_K * float(np.sqrt(max(self.noise, 0.0)))))
            if self.latched and self.armed and still and not self.rec and abs(self.th_pred) < cur:
                self.noise += (self.th_pred ** 2 - self.noise) / (NOISE_TAU * FS)
            on_ = max(on_, min(NOISE_CAP, NOISE_K * float(np.sqrt(max(self.noise, 0.0)))))
        self.on_still = on_
        if not still:
            mv = REC_ON_MOVE
            if REC_ON_MOVE_HEAVY is not None and self.latched and self.f_latched >= CLASS_SPLIT_F:
                mv = REC_ON_MOVE_HEAVY
            on_ = max(on_, mv)
        # 小事件快速撤回
        if self.qr_active:
            self.qr_n += 1
            self.qr_peak = max(self.qr_peak, abs(err + s_ * self.om_lp))
            if self.qr_n >= int(QR_T * FS):
                self.qr_active = False
                if QR_PEAK is not None and self.qr_peak < QR_PEAK and not self.latched:
                    self._revert(self.qr_prev)
        # stiff 只有在「已锁存、且不在最硬档」时才有意义。已经在硬档（含探测中）
        # 就不再触发 —— 否则硬档下空车自激（角速度上 100°/s）会让 θ_pred 一直
        # 超门限，每次都把探测清零，永远降不回来：实测 32 组候选全部卡在硬档，
        # 正常行驶 8 局误升档上千次、空车抖振 33.7（= 原厂 HEAVY）。
        can_up = self.latched and self.f_latched < 0.999
        # 上膛：锁存/升档后要连续平稳 ARM_S 秒，触发才生效。上膛后**保持**，
        # 只有锁存（含降档）和升档会解除 —— 第一版每拍按 |θ_pred|<ARM_DEG 重算，
        # 推击时 θ_pred 一路涨上去，先过 2.5° 再到门限，到门限前一拍就把自己
        # 解除了，于是永远不触发。
        if self.latched and not self._was_latched:
            self.armed, self.arm_cnt = False, 0  # 刚锁存：重新计时
        self._was_latched = self.latched
        if not self.armed:
            if self.latched and abs(self.th_pred) < ARM_DEG:
                self.arm_cnt += 1
                if self.arm_cnt >= int(ARM_S * FS):
                    self.armed = True
            else:
                self.arm_cnt = 0
        armed = self.armed
        if act_ == "stiff" and not (can_up and armed):
            on_ = float("inf")
        if not self.rec:
            if not still:
                self.th_ref += err * (1.0 / (REF_TAU_FAST * FS))   # 行驶/刚停：快跟、不设门
            elif abs(err) < REF_CALM_DEG:
                self.th_ref += err * (1.0 / (REF_TAU * FS))        # 静止：慢跟、设门
            if abs(self.th_pred) > on_:
                self.rec = True
                self.rec_entries += 1
                self.calm = 0
                if act_ == "stiff":
                    prev = self.f_latched
                    self._rearm()
                    self.qr_active, self.qr_n, self.qr_peak, self.qr_prev = True, 0, 0.0, prev
        else:
            if abs(err) < REC_OFF_DEG and abs(om) < REC_OFF_DPS:
                self.calm += 1
                if self.calm >= REC_OFF_TICKS:
                    self.rec = False
            else:
                self.calm = 0
                if act_ == "stiff" and abs(self.th_pred) > on_:
                    self._rearm()        # 只在还没升上去时有效（升档后 on_=inf）
        self._emergency(err, om, FS)
        a = super().__call__(env, moving)          # v3 照常：读数、锁存、PID
        self.f_max_seen = max(self.f_max_seen, self.f)
        if self.rec:
            self.rec_ticks += 1
            if act_ == "bang":
                a0 = float(np.clip(k_ * self.th_pred, -1.0, 1.0))
                return np.array([a0, 0.0])
        return a

    def _emergency(self, err, om, FS):
        """f > 1 紧急档：触发 -> 至少保持 hold 秒且平稳 -> 回 f=1 重新探测（降档）。

        不看锁存状态（保命优先）。紧急期间每拍把探测窗口清零，保证 f>1 时
        绝不判负载；结束时 _rearm() 回到 f=1，从标定点重新探测。
        """
        if EMERG is None:
            return
        s_e, on_e, f_e, _vf, hold_s = EMERG
        pe = err + s_e * om
        if not self.emerg and abs(pe) > on_e:
            self.emerg = True
            self.emerg_n = 0
            self.emerg_calm = 0
            self.emerg_entries += 1
        if not self.emerg:
            return
        self.emerg_n += 1
        self.emerg_ticks += 1
        if abs(err) < REC_OFF_DEG and abs(om) < REC_OFF_DPS:
            self.emerg_calm += 1
        else:
            self.emerg_calm = 0
        if self.emerg_n >= int(hold_s * FS) and self.emerg_calm >= REC_OFF_TICKS:
            self.emerg = False
            self._rearm()                    # 回 f=1，重新探测 -> 由抖振降档
            return
        self.f = float(f_e)
        self.f_latched = 1.0
        self.latched = False
        self.boost = 0.0
        self.n_still, self.win = 0, []
        self.n_move, self.mwin = 0, []

    def _revert(self, f_prev):
        """小事件快速撤回：恢复升档前锁存的档位，不重新探测（负载没变）。"""
        self.f = f_prev
        self.f_latched = f_prev
        self.latched = True
        self.event_probe = False
        self.boost = 0.0
        self.rec = False
        self.n_still, self.win = 0, []
        self.armed, self.arm_cnt = False, 0
        self.qr_reverts += 1

    def _rearm(self):
        """升到最硬档（f=1，也就是抖振判定的标定基准）并解除锁存，重新探测。

        「升档」和「受冲击」是同一个症状（倾角过大 = 增益不够），用同一个动作。
        之后降不降档由抖振探测决定：轻车在硬档下持续自激（读数 20-33）→ 判轻
        → 降回去；重车在硬档下安静（0.3-0.6）→ 留在重档。**判定只在硬档下做**
        —— 抖振和负载的对应关系只在 f=1 成立，边降边判就是 v2 的闭环污染。
        """
        self.f = 1.0
        self.f_latched = 1.0
        self.latched = False
        self.armed, self.arm_cnt = False, 0
        self.boost = 0.0
        self.n_still = 0
        self.win = []
        self.event_probe = True        # 事件后的重新探测：判轻用保守分界（switch_v3.EVENT_SPLIT）
        self.rearms = getattr(self, "rearms", 0) + 1

    def _params(self):
        """当前负载类别对应的 (S, K, ON, 动作)。动作缺省为 "bang"。"""
        heavy = self.latched and self.f_latched >= CLASS_SPLIT_F
        p = REC_HEAVY if heavy else REC_LIGHT
        if p is None:
            p = (REC_S, REC_K, REC_ON_DEG)
        return tuple(p) + (("bang",) if len(p) == 3 else ())


def _selftest():
    import goto_hold as GH
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    SV3.PROBE_MODE = "converge"
    for kg, F in ((0.0, 0.0), (0.0, 8.0), (2.0, 0.0)):
        env = E2EBalanceEnv(E2EConfig(seed=0, episode_seconds=9.0, taps=(0,),
                                      payload_p_empty=1.0, **dict(GH.BASE)))
        env._sample_payload = lambda: kg
        env.reset(seed=0)
        env.yaw_ref = 0.0
        env._cmd_left = 10 ** 9
        env.v_ref = 0.0
        env.pos_ref = env.pos
        sw = SwitchV4()
        fell = None
        for n in range(int(8.0 * FS)):
            if F and n == int(3.0 * FS):
                env.core.dist.fire_impulse(force=F, direction=0.0, duration=0.12)
            _o, _r, d, t, _ = env.step(sw(env))
            if d:
                fell = n / FS
                break
        print("  %gkg 推 %gN：恢复模式开 %d 拍（进入 %d 次） 摔 %s  f=%.2f"
              % (kg, F, sw.rec_ticks, sw.rec_entries, fell, sw.f))


if __name__ == "__main__":
    os.environ.setdefault("E026_REAL_DATA",
                          r"C:\Users\jiang li\Downloads\e026 keil\deadband_test")
    sys.path.insert(0, os.path.join(ROOT, "scripts", "twin_fit"))
    os.chdir(ROOT)
    print("v4 自检（初值参数）：正常站立必须 0 拍；推一下应该触发")
    _selftest()
