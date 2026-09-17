/* ------------------------------------------------------------------
 * 负载自动切档   自动生成，别手改
 * 2026-09-14 21:30
 *
 * 【为什么要两套】一套打不了 0~4 kg，冲突在 D 项：
 *     空车       需要 Balance_Kd <= 48，否则 ~31 Hz 剧烈抖
 *     4 kg 行驶  需要 Balance_Kd >= 96，否则起步加速时摔
 * 中间没有安全值：Kd 从 56 到 72，空车抖动 23.5 -> 107.3 deg/s，
 * 4.6 倍的跳变挤在 0.16 的区间里。
 *
 * 【怎么自动切】不测负重 —— 负重在这台车上基本不可观（往车顶加重既加质量
 * 又抬质心，轮轴反冲把多出来的俯仰惯量抵消大半，实测特征跨 0~4 kg 只差
 * 1.29 倍）。改成测**症状**：陀螺在 31 Hz 附近的能量。
 *     有 31 Hz 抖动 = 当前 Kd 对当前负载太大 = 该用 NORMAL
 * 它直接测的就是判据本身，不是「负重多少」这个代理量。这也是为什么负重
 * 测不出来而这个信号能用 —— 它只需要分辨临界点两侧，不需要分辨大小。
 *
 * 【必须带通，不能用高通】抖动有两个机制相反的来源：
 *     空车  Kd 太大 -> ~31 Hz 快模态限幅振荡
 *     4 kg  Kd 太小 -> 车撑不住，低频大幅晃动
 * 全带宽能量把两者混成一个数，结果对载重非单调（NORMAL 下 0kg 44.8 /
 * 2kg 8.8 / 4kg 35.5，两头高中间低），4 kg 被判成空车、留在 NORMAL、
 * 实测 0/8 必摔。
 *
 * 【验证】MuJoCo 孪生，扰动 difficulty 0.4，两个安全关键点各 20 种子：
 *     空车  存活 20/20   判定 20/20 -> NORMAL
 *     4 kg  存活 19/20   判定 20/20 -> HEAVY
 * 中间 0.5~2 kg 判定逐种子不一致，但两档都安全（7~8/8），无后果。
 *
 * 【上车前必读】
 *   - Balance_Kp 是原厂的 4 倍。按文件末尾清单分步试。
 *   - 阈值余量只有 1.8 倍，**必须按真车重标**，步骤见第 4 节。
 * ------------------------------------------------------------------ */

#include <math.h>

/* ==== 1. 两套增益 ==== */
/*   载重      NORMAL(Kd48)        HEAVY(Kd120)
 *   0 kg    5/5   22.5 deg/s    5/5  >120 deg/s   <<< 空车禁用 HEAVY
 *   1 kg    5/5    1.2          5/5    9.3
 *   2 kg    5/5    0.9          5/5    6.9
 *   4 kg    1/5   10.2          5/5   11.2         <<< NORMAL 到顶
 *   （行驶存活/5，及该载重下静止时的陀螺 rms） */

typedef struct { float kp, kd, vkp, vki, tkp, tkd; } LoadGains;

static const LoadGains G_NORMAL = { 38400.0f, 48.0f, 11000.0f, 92.0f, 1400.0f, 20.0f };
static const LoadGains G_HEAVY  = { 38400.0f, 120.0f, 15500.0f, 120.0f, 1400.0f, 20.0f };

/* 位置环就是 Velocity_Ki：固件的「速度环」是 P(速度) + I(位置)，
 *     Encoder_Integral += Encoder_bias;      // 速度的积分 = 位置
 *     velocity = -Encoder_bias*Vkp - Encoder_Integral*Vki;
 * 所以 Vki 是位置增益。相比原厂（Vkp 6200 / Vki 31）抬高之后，空车站定
 * 游走 74 -> 48 mm、4 kg 35 -> 28 mm。再往上抬会压掉检测信号，见第 4 节。 */

/* 负载模式那三个系数必须全是 1.0。这两套已经把负载考虑进去了，再乘
 * Balance_K=2.0 会把 D 项一起放大 —— 那正是原厂负载模式空车剧烈振荡的
 * 原因（毁掉空车的是 D 项，不是 P 项）。 */
float Balance_K  = 1.0f;
float Velocity_K = 1.0f;
float Turn_K     = 1.0f;

/* ==== 2. 检测器：200 Hz 中断里调一次 ==== */
/* 26.5~45.5 Hz 带通 -> 平方 -> 低通。两级一阶滤波，4 个乘加。 */

#define LD_HZ           200.0f
#define LD_A_HP         0.454545f   /* 高通拐点 26.5 Hz */
#define LD_A_LP         0.588235f   /* 低通拐点 45.5 Hz */
#define LD_A_EN         0.016393f   /* 能量平滑 */
#define LD_LSB_PER_DPS  16.4f

/* 下面两个数是孪生里的，**上车必须重标**，见第 4 节 */
float LD_LOADED_BELOW = 10.5f;   /* NORMAL 下低于此 -> 装了货 */
float LD_EMPTY_ABOVE  = 29.0f;   /* HEAVY  下高于此 -> 车空了 */

#define LD_PRIME_S      0.90f   /* 预热，之前不判决 */
#define LD_CONFIRM_S    2.0f    /* 判定要连续成立这么久 */
#define LD_LOCKOUT_S    3.0f    /* 换档后禁判，防止边界来回跳 */

static float ld_mean, ld_band, ld_energy, ld_hold, ld_lock, ld_prime;
int   ld_heavy = 1;        /* 开机进 HEAVY，理由见下 */
float ld_rms   = 0.0f;     /* 标定时读这个 */

/* 开机为什么是 HEAVY：风险不对称。
 *     NORMAL 用在 4 kg 上 -> 行驶时摔
 *     HEAVY  用在空车上   -> 抖 >120 deg/s，但不摔（20/20 存活）
 * 而那个抖恰好是最响亮的检测信号，所以检测失败的后果是抖两秒，不是摔车。
 * 试过开机进 NORMAL + 封锁行驶指令，两个安全关键点都会判错（空车有时切进
 * HEAVY、4 kg 有时留在 NORMAL），已否掉。 */
void LD_Reset(void)
{
    ld_mean = ld_band = ld_energy = 0.0f;
    ld_hold = ld_lock = ld_prime = 0.0f;
    ld_heavy = 1;
}

/* gyro   = Gyro_Balance（MPU6050 原始 LSB，和 Balance_Kd 吃的是同一个数）
 * moving = 正在被指令行驶（Move_X 或 Move_Z 非零）
 * 返回 1 = 该用 HEAVY */
int LD_Update(float gyro, int moving)
{
    const float dt = 1.0f / LD_HZ;
    float hp;

    ld_mean   += LD_A_HP * (gyro - ld_mean);      /* 去掉 27 Hz 以下 */
    hp         = gyro - ld_mean;
    ld_band   += LD_A_LP * (hp - ld_band);        /* 去掉 45 Hz 以上 */
    ld_energy += LD_A_EN * (ld_band * ld_band - ld_energy);
    ld_rms     = sqrtf(ld_energy) / LD_LSB_PER_DPS;

    if (ld_prime < LD_PRIME_S) { ld_prime += dt; return ld_heavy; }

    /* 行驶时冻结判决：抖动统计量只有静止时可标定，行驶时驾驶动作会污染它
     * （实测空车在行驶段会掉进「装了货」的区间，造成多余换档）。载重只在
     * 有人停下装卸货时才变，行驶中不更新判决没有损失。 */
    if (moving) { ld_hold = 0.0f; return ld_heavy; }

    if (ld_lock > 0.0f) { ld_lock -= dt; ld_hold = 0.0f; return ld_heavy; }

    {
        int want = ld_heavy;
        if (!ld_heavy && ld_rms < LD_LOADED_BELOW)      want = 1;
        else if (ld_heavy && ld_rms > LD_EMPTY_ABOVE)   want = 0;

        if (want != ld_heavy) {
            ld_hold += dt;
            if (ld_hold >= LD_CONFIRM_S) {
                ld_heavy = want;
                ld_hold  = 0.0f;
                ld_lock  = LD_LOCKOUT_S;
            }
        } else {
            ld_hold = 0.0f;
        }
    }
    return ld_heavy;
}

/* ==== 3. 接进 pid_control.c ==== */
/* 在 200 Hz 中断里、调 Balance_PD / Velocity_PI / Turn_PD **之前**： */
#if 0
    const LoadGains *g;
    LD_Update(Gyro_Balance, (Move_X != 0) || (Move_Z != 0));
    g = ld_heavy ? &G_HEAVY : &G_NORMAL;
    Balance_Kp = g->kp;   Balance_Kd = g->kd;
    Velocity_Kp= g->vkp;  Velocity_Ki= g->vki;
    Turn_Kp    = g->tkp;  Turn_Kd    = g->tkd;
#endif

/* ==== 4. 阈值标定（必做）==== */
/* 上面两个阈值是孪生里的数，余量只有 1.8 倍。真车的陀螺噪声底、机械间隙、
 * 电机响应都不一样，绝对值几乎肯定不同。四次读数：
 *
 *   1) 空车站稳（此时在 NORMAL），读 ld_rms         ->  R_empty_N
 *   2) 装 2 kg 站稳，读 ld_rms                      ->  R_load_N
 *          LD_LOADED_BELOW = sqrt(R_empty_N * R_load_N)
 *   3) 手动置 ld_heavy = 1，仍装 2 kg，读 ld_rms    ->  R_load_H
 *   4) 卸货变空车，**扶住车**（这一步会剧烈抖），读 ld_rms -> R_empty_H
 *          LD_EMPTY_ABOVE = sqrt(R_empty_H * R_load_H)
 *
 * 不标的后果：真车 rms 通常比孪生高，检测器会一直判「空车」、一直停在
 * NORMAL —— 而 NORMAL 在 4 kg 行驶时会摔。**这一步不能省。** */

/* ==== 5. 原厂值，对照用 ==== */
/* static const LoadGains G_FACTORY = { 9600.0f, 48.0f, 6200.0f, 31.0f, 1400.0f, 20.0f }; */

/* ------------------------------------------------------------------
 * 上车清单 —— 按顺序，每一步不过就停
 *
 *  0) 先确认 -O1（-O0 时 0x08010000 以上的 flash 存不住 RW 初值，全局变量
 *     开机就是乱的）。烧完读回 Balance_Kp，对一下是不是 38400。
 *  1) **先不接检测器**，手动固定 G_NORMAL：空车扶着开机、放手、前后左右
 *     各开一遍。
 *  2) 手动固定 G_HEAVY，**装 2 kg**，重复。这一步第一次把 Kd 提到 120，
 *     务必先扶着、确认不抖再放手。
 *  3) G_HEAVY + 4 kg，重复。
 *  4) 做第 4 节的标定，把两个阈值填进去。
 *  5) 接上检测器。空车开机：应该先抖一两秒（HEAVY 用在空车上），然后自己
 *     切到 NORMAL 安静下来。**这是正常的，不是故障。**
 *  6) 装 2 kg，停稳几秒，应该自己切到 HEAVY。
 *
 * 【高频抖，30 Hz 左右的嗡嗡声】
 *   NORMAL 下出现 -> Balance_Kd 48 降到 34，别动 Balance_Kp。
 *   HEAVY  下出现 -> 先查货够不够 2 kg；够了还抖，说明真车这个模态比孪生
 *                    更早，退回 NORMAL 用。
 * 【慢速前后晃，1 秒以上一个来回】速度环，调 Velocity_Kp。
 * 【起步就往前扑】Kd 不够，该切 HEAVY 了。
 * 【档位来回跳】阈值没标好，重做第 4 节。
 * ------------------------------------------------------------------ */
