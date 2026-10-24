/* rec_dll.c -- host DLL around rec_core.c for the twin.  Not firmware.
 *   gcc -O2 -shared -o rec_core.dll rec_dll.c rec_core.c */
#include "rec_core.h"
#define EXPORT __declspec(dllexport)
static REC_State s;
EXPORT void rec_init(void) { REC_Init(&s); }
EXPORT void rec_set(int mode, float on_deg) { s.mode = mode; s.on_deg = on_deg; }
EXPORT void rec_exit(float d, float w, float v) { s.exit_deg = d; s.exit_dps = w; s.exit_v = v; }
EXPORT void rec_sr(int en, int push1, int coast, int push2, float hand)
{ s.sr_en = en; s.sr_push1 = push1; s.sr_coast = coast; s.sr_push2 = push2; s.sr_hand = hand; }
EXPORT int  rec_step(float err, float gyro_lsb, int el, int er, int *u) { return REC_Step(&s, err, gyro_lsb, el, er, u); }
EXPORT void rec_stopped(void) { REC_Stopped(&s); }
EXPORT void rec_sr_rev(int r) { s.sr_rev = r; }
EXPORT int  rec_locked(void) { return s.locked; }
EXPORT long rec_entries(void) { return s.entries; }
EXPORT long rec_locks(void) { return s.locks; }
EXPORT long rec_sr_attempts(void) { return s.sr_attempts; }
EXPORT long rec_sr_ok(void) { return s.sr_ok; }
EXPORT int  rec_sr_state(void) { return s.sr_state; }
