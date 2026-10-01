/* Cross-checks the STATEFUL half of the observation builder - the history
 * buffer and both leaky integrals - against an independent Python replication
 * of the same recurrence (see the companion check this prints instructions
 * for). test_policy.c only exercises policy_infer() on isolated vectors and
 * cannot catch a bug here: a wrong tap index, a transposed decay constant, or
 * an off-by-one in the ring buffer would still produce a plausible-looking
 * action, just not the trained one.
 *
 *   gcc -O2 -o test_obs_builder test_obs_builder.c policy.c -lm
 *   ./test_obs_builder > obs_builder_c_output.txt
 *
 * Then diff against the Python reference (see bottom of this file for the
 * exact sequence it must match).
 */
#include <stdio.h>
#include "policy.h"

#define N_TICKS 60

int main(void)
{
    float pitch[N_TICKS], pitch_rate[N_TICKS], v_forward[N_TICKS], yaw_rate[N_TICKS];
    const float cmd_forward = 0.05f, cmd_turn = 0.1f;

    /* Simple polynomial sequence - deliberately not trigonometric, so a
     * float32-vs-float64 comparison against the Python reference isn't
     * fighting sin/cos rounding noise on top of whatever the logic itself
     * disagrees on. */
    for (int i = 0; i < N_TICKS; ++i) {
        pitch[i]      = 0.001f * (float)i - 0.02f;
        pitch_rate[i] = 0.0005f * (float)i - 0.01f;
        v_forward[i]  = 0.002f * (float)i - 0.05f;
        yaw_rate[i]   = 0.001f * (float)i - 0.03f;
    }

    policy_reset_state(pitch[0], pitch_rate[0], v_forward[0]);

    float obs[POLICY_N_OBS];
    const float gyro_xyz[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 1; i < N_TICKS; ++i) {
        float g[3] = { gyro_xyz[0], pitch_rate[i], yaw_rate[i] };
        policy_build_obs_rp(obs, 0.0f, pitch[i], 0.0f, 0.0f, v_forward[i],
                            g, 0.0f, 0.0f, cmd_forward, cmd_turn);
        printf("%d", i);
        for (int k = POLICY_N_BASE_OBS; k < POLICY_N_OBS; ++k) {
            printf(" %+.8f", obs[k]);
        }
        printf("\n");
    }
    return 0;
}

/* Python reference this must match (run separately, diff the two outputs -
 * see firmware/README-STM32-DEPLOYMENT.md Step 2 for the full invocation):

import numpy as np
N = 60
pitch      = [0.001*i - 0.02 for i in range(N)]
pitch_rate = [0.0005*i - 0.01 for i in range(N)]
v_forward  = [0.002*i - 0.05 for i in range(N)]
yaw_rate   = [0.001*i - 0.03 for i in range(N)]
cmd_forward, cmd_turn = 0.05, 0.1
DT = 1.0/200.0
POS_DECAY, YAW_DECAY = np.exp(-DT/2.0), np.exp(-DT/30.0)
TAPS = (2, 5, 11, 23, 47)

hist = [(pitch[0], pitch_rate[0], v_forward[0])] * 48
pos_err = yaw_err = 0.0
for i in range(1, N):
    pos_err = POS_DECAY*pos_err + (v_forward[i] - cmd_forward)*DT
    yaw_err = np.clip(YAW_DECAY*yaw_err + (yaw_rate[i] - cmd_turn)*DT, -0.2, 0.2)
    hist.insert(0, (pitch[i], pitch_rate[i], v_forward[i])); del hist[48:]
    row = [v for t in TAPS for v in hist[t]] + [pos_err*10.0, yaw_err*5.0]
    print(i, *[f"{v:+.8f}" for v in row])
*/
