/* ------------------------------------------------------------------
 * PPO 调出来的静态 PID 增益 + 导航外环
 * 生成时间 2026-09-11 01:30   训练步数 30,000
 *
 * 用法：
 *   1) 用下面的常量替换 APP/PID/pid_control.c 顶部那六个 float。
 *      注意固件用的是 x100 的整数形式，这里已经乘回去了。
 *   2) 把 nav_step() 抄进 APP/app_control.c，在 200 Hz 中断里、
 *      调用 Balance_PD/Velocity_PI/Turn_PD **之前**调用它，用它的
 *      输出覆盖 Move_X / Move_Z。
 *   3) 见文件末尾「板子上还缺什么」。
 * ------------------------------------------------------------------ */

#include <math.h>

/* app_control.c 里已经有 PI 了；单独编译这个文件时才需要这个兜底 */
#ifndef PI
#define PI 3.14159265358979f
#endif

/* ---- 1. PID 增益（替换 pid_control.c 顶部）---- */
float Balance_Kp   = 19358.1f;   /* 孪生里的 balance_kp = 193.6 */
float Balance_Kd   = 36.0f;   /* 孪生里的 balance_kd = 0.3603 */
float Velocity_Kp  = 7618.4f;   /* 孪生里的 velocity_kp = 76.18 */
float Velocity_Ki  = 11.1f;   /* 孪生里的 velocity_ki = 0.1107 */
float Turn_Kp      = 4200.0f;   /* 孪生里的 turn_kp = 42 */
float Turn_Kd      = 19.7f;   /* 孪生里的 turn_kd = 0.1967 */

/* 负载模式的三个系数必须全部设为 1.0：这套增益就是为 0~4 kg 全程
 * 训练的，再乘 Balance_K 会把 D 项放大，那正是空车振荡的原因。 */
float Balance_K  = 1.0f;
float Velocity_K = 1.0f;
float Turn_K     = 1.0f;

/* ---- 2. 导航外环（抄进 app_control.c）---- */
/* 固件原本没有位置/航向反馈，转向环是纯前馈，所以再怎么调 PID 也
 * 开不到指定坐标。这段就是缺的那条通路。 */

#define NAV_KV      1.8011f   /* 距离 -> 速度，1/s */
#define NAV_KW      1.1460f   /* 方位角 -> 偏航，1/s */
#define NAV_V_MAX   0.550f
#define NAV_W_MAX   1.200f
#define NAV_ARRIVE  0.050f   /* m，进这个圈算到了 */

/* 车当前位姿，由 nav_odom() 递推；目标点由上位机/按键设定 */
float nav_x = 0, nav_y = 0, nav_psi = 0;
float nav_tx = 0, nav_ty = 0;

void nav_step(float *out_v, float *out_w)
{
    float dx = nav_tx - nav_x;
    float dy = nav_ty - nav_y;
    float dist = sqrtf(dx * dx + dy * dy);
    float bearing = atan2f(dy, dx) - nav_psi;
    while (bearing >  PI) bearing -= 2.0f * PI;
    while (bearing < -PI) bearing += 2.0f * PI;

    /* 符号：Motor_Left = ...+Turn_Pwm 让左轮快、车顺时针转，而 psi
     * 是逆时针为正，所以指令要取负。写反了车会越转越远。 */
    float w = -NAV_KW * bearing;
    if (w >  NAV_W_MAX) w =  NAV_W_MAX;
    if (w < -NAV_W_MAX) w = -NAV_W_MAX;

    if (dist <= NAV_ARRIVE) { *out_v = 0.0f; *out_w = 0.0f; return; }

    /* cos 门控：没对准目标就先转、少走，对准了才全速 */
    float gate = cosf(bearing);
    if (gate < 0.0f) gate = 0.0f;
    float v = NAV_KV * dist;
    if (v >  NAV_V_MAX) v =  NAV_V_MAX;
    if (v < -NAV_V_MAX) v = -NAV_V_MAX;

    *out_v = v * gate;
    *out_w = w;
}

/* ---- 3. 里程计递推，每个 200 Hz 拍调一次 ---- */
/* 4 x 11 x 30 = 1320 计数/圈（app_motor.h），轮径 67 mm，轮距 167 mm */
#define CPR         1320.0f
#define WHEEL_D     0.067f
#define TRACK       0.167f

void nav_odom(int enc_l, int enc_r)
{
    float sl = (float)enc_l / CPR * PI * WHEEL_D;
    float sr = (float)enc_r / CPR * PI * WHEEL_D;
    float ds = 0.5f * (sl + sr);
    float dpsi = (sr - sl) / TRACK;
    nav_psi += dpsi;
    nav_x += ds * cosf(nav_psi);
    nav_y += ds * sinf(nav_psi);
}

/* ------------------------------------------------------------------
 * 板子上还缺什么（老实说明，别以为抄完就能跑）
 *
 * a) nav_odom 用的是编码器递推的航向。板子上也有 angle_z，但
 *    app_control.c 里它被算错了：Wheel_spacing 是 161.0 毫米，代码
 *    先除 Wheel_spacing 再除 1000，等于除了 161000 而不是 0.161，
 *    偏航反馈项被缩掉 10^6 倍。要用 angle_z 就得先修这个标度。
 *    上面的 nav_odom 绕开了它，只用编码器——代价是长时间会累积漂移。
 *
 * b) TRACK 用的是 STEP 装配量出来的 167 mm，不是固件里写的 161 mm。
 *    固件那个值偏小 3.6%，会让递推出来的航向偏快。
 *
 * c) 里程计只用编码器，打滑就丢定位。这套增益在孪生里是在**不打滑**
 *    的地面上训的。
 *
 * d) 本文件用 gcc -std=c99 -Wall -Wextra 编译无警告并跑通过
 *    （nav_odom + nav_step 的数值和 Python 侧一致）。但**没有在
 *    Keil / STM32F103 上编过**：Cortex-M3 没有硬件浮点，这里的
 *    sqrtf/atan2f/cosf 走软件库，在 200 Hz 中断里要量一下耗时。
 *    真嫌慢就把 nav_step 降到 50 Hz 调用（导航外环不需要 200 Hz）。
 *
 * e) 这些增益是在数字孪生里训的，孪生本身对真车的标定见 TWIN_BASELINE.md。
 *    上车第一次务必**空载、低速、有人扶**着试。
 * ------------------------------------------------------------------ */
