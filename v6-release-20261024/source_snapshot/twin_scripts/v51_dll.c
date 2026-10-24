/* v4rel_dll.c -- host-only DLL around the RELEASED firmware core
 * (e026 keil\v4_work_20261003\APP\PID\v4_core.c, = v4_release_20261003), so the
 * twin can score exactly the code that is on the car.  NOT part of the firmware.
 *
 *   gcc -O2 -shared -I<core dir> -o v4rel.dll v4rel_dll.c <core dir>/v4_core.c -lm
 *   (add -DEDGE_GRACE_N=0 -DRELATCH_GRACE=0 -DSTRUGGLE_IN_REC=1 for the
 *    "round-4 off" check build)
 */
#include "v4_core.h"

#define EXPORT __declspec(dllexport)

static V4_State st;

EXPORT void  v4_reset(void)                       { V4_Init(&st); }
EXPORT void  v4_cmd_edge(void)                    { V4_CmdEdge(&st); }
EXPORT float v4_step(float angle_deg, float gyro_lsb, int moving)
{
    return V4_Step(&st, angle_deg, gyro_lsb, moving);
}
EXPORT long  v4_rearms(void)   { return st.rearms; }
EXPORT int   v4_latched(void)  { return st.latched; }
EXPORT long  v4_downs(void)     { return st.downs; }
EXPORT long  v4_up_blocks(void) { return st.up_blocks; }
EXPORT float v4_osc(void)       { return st.osc; }
EXPORT void  v4_force(float f)  { st.f = f; st.f_latched = f; st.latched = 1; st.event_probe = 0; st.boost = 0.0f; st.ceil_n = 0; st.ceil = 1.0f; st.floor_f = 0.0f; }
EXPORT void  v4_splits(float l, float m) { v4_light_split = l; v4_mid_split = m; }
EXPORT void  v4_set_opt(int o, float wamp) { v4_opt = o; v4_wamp = wamp; }
EXPORT long  v4_small_ups(void)    { return st.small_ups; }
EXPORT long  v4_probe_aborts(void) { return st.probe_aborts; }
EXPORT int   v4_osc_on(void)       { return st.osc_on; }
EXPORT void  v4_set_swup(float t) { v4_swup = t; }
EXPORT long  v4_sway_ups(void)    { return st.sway_ups; }
EXPORT float v4_sw(void)          { return st.sw; }
/* debug snapshot for trace scripts */
EXPORT void  v4_dbg(float *o)
{
    o[0] = st.f; o[1] = st.f_latched; o[2] = st.boost; o[3] = (float)st.latched;
    o[4] = (float)st.rec; o[5] = st.osc; o[6] = st.oscf; o[7] = st.sw; o[8] = st.sw_old;
    o[9] = st.swcr * V4_FS * 0.5f; o[10] = st.floor_f; o[11] = st.ceil; o[12] = (float)st.ceil_n;
    o[13] = st.struggle; o[14] = (float)st.rearms; o[15] = (float)st.downs; o[16] = (float)st.sway_ups;
    o[17] = st.th_ref; o[18] = (float)st.event_probe;
}
