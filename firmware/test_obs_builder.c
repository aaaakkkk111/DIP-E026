/* Cross-checks the firmware's whole observation pipeline - encoder odometry,
 * observation builder, history buffer and both leaky integrals - against the
 * training wrapper itself (train_real_robot.py PWMCommandWrapper), tick by
 * tick, on a real simulated rollout. test_policy.c only exercises
 * policy_infer() on isolated vectors and cannot catch a bug here: a wrong tap
 * index, a missing clamp or an off-by-one in a ring buffer still produces a
 * plausible-looking action, just not the trained one.
 *
 * Built as a shared library and driven from Python (tools/check_obs_builder.py
 * style, see firmware/README-STM32-DEPLOYMENT.md Step 2):
 *
 *   gcc -O2 -shared -o test_obs_builder.dll test_obs_builder.c policy.c -lm
 *
 * Input per tick (9 floats): dcount_l dcount_r roll pitch gx gy gz cmd_f cmd_t
 * Row 0 is the arming tick: only pitch and gy are used, for
 * policy_reset_state(). Output per tick: the 34 observations (row 0 zeros).
 */
#include "policy.h"
#include <math.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

EXPORT int obs_builder_run(int n_ticks, const float *in, float *out)
{
    for (int t = 0; t < n_ticks; ++t) {
        const float *r = in + 9 * t;
        float *o = out + POLICY_N_OBS * t;
        if (t == 0) {
            policy_reset_state(r[3], r[5], POLICY_WHEEL_RADIUS * r[5] * cosf(r[3]));
            for (int i = 0; i < POLICY_N_OBS; ++i) o[i] = 0.0f;
            continue;
        }
        float wl, wr, v;
        const float gyro[3] = { r[4], r[5], r[6] };
        policy_odom_update((long)r[0], (long)r[1], r[3], r[5], &wl, &wr, &v);
        policy_build_obs_rp(o, r[2], r[3], v, gyro, wl, wr, r[7], r[8]);
    }
    return POLICY_N_OBS;
}
