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
