/* Host-side check that the C port reproduces the trained network.
 *
 *   gcc -O2 -o test_policy test_policy.c policy.c -lm && ./test_policy
 *
 * Two tolerances, because there are two builds:
 *   POLICY_FAST_TANH=0  exact libm tanh -> must match PyTorch to float32
 *                       rounding (1e-5). Any larger error is a porting bug,
 *                       almost always row/column-major confusion.
 *   POLICY_FAST_TANH=1  rational tanh approximation for FPU-less targets ->
 *                       deviates by design. Originally bounded at 0.0113 (a
 *                       0.6 N.m torque limit, the earlier policy) over 2000
 *                       random observations, under 2%. Re-checked against the
 *                       run-7 (PWM) checkpoint's own 4 test vectors: worst
 *                       case 1.16e-02 against a +/-1.0 action limit, still
 *                       under the 2% tolerance below.
 */
#include <stdio.h>
#include <math.h>
#include "policy.h"
#include "policy_testvectors.h"

#ifndef POLICY_FAST_TANH
#define POLICY_FAST_TANH 1
#endif

#if POLICY_FAST_TANH
#  define TOL 2.0e-2f
#  define MODE "fast tanh (approximation, for STM32F103 / no FPU)"
#else
#  define TOL 1.0e-5f
#  define MODE "exact tanh (must match PyTorch bit-for-bit)"
#endif

int main(void)
{
    float worst = 0.0f;
    printf("mode: %s\ntolerance: %.1e\n\n", MODE, TOL);
    printf("%-6s %-24s %-24s %s\n", "case", "C output", "expected (PyTorch)", "max |diff|");
    for (int t = 0; t < POLICY_N_TESTS; ++t) {
        float act[POLICY_N_ACT];
        policy_infer(&policy_test_obs[t * POLICY_N_OBS], act);
        float d = 0.0f;
        for (int i = 0; i < POLICY_N_ACT; ++i) {
            float e = fabsf(act[i] - policy_test_expected[t * POLICY_N_ACT + i]);
            if (e > d) d = e;
        }
        if (d > worst) worst = d;
        printf("%-6d %+.6f %+.6f   %+.6f %+.6f   %.3e\n", t,
               act[0], act[1],
               policy_test_expected[t * POLICY_N_ACT + 0],
               policy_test_expected[t * POLICY_N_ACT + 1], d);
    }
    printf("\nworst error: %.3e   (tolerance %.1e)\n", worst, TOL);
    printf("%s\n", worst < TOL ? "PASS" : "FAIL - ports disagree");
    return worst < TOL ? 0 : 1;
}
