#ifndef YB_PROTOCOL_H
#define YB_PROTOCOL_H
#include <stdint.h>
#include <float.h>
/* Version 1: AA 55 01 type seq:u16 len:u16 payload CRC:u16, little endian.
 * CRC16/CCITT-FALSE covers version through last payload byte, NOT AA 55. */
static uint16_t yb_crc16(const uint8_t *p, unsigned n)
{
    uint16_t crc=0xffffU;
    unsigned i;
    while(n--) {
        crc ^= (uint16_t)*p++ << 8;
        for(i=0; i<8; ++i)
            crc=(uint16_t)((crc & 0x8000U) ? (crc << 1)^0x1021U : crc << 1);
    }
    return crc;
}
static int yb_finite(float x) { return x==x && x<=FLT_MAX && x>=-FLT_MAX; }
/* Reject, never silently clamp. Relative bounds plus a boot-anchored envelope
 * prevent repeated 20% updates from walking to unsafe unbounded gains. */
static int yb_gains_valid(const float next[6], const float current[6],
                          const float boot[6])
{
    unsigned i;
    for(i=0; i<6; ++i) {
        if(!yb_finite(next[i]) || !yb_finite(current[i]) || !yb_finite(boot[i]) ||
           next[i]<0 || boot[i]<0 || current[i]<0) return 0;
        if(next[i] < 0.8f*current[i] || next[i] > 1.2f*current[i] ||
           next[i] < 0.5f*boot[i] || next[i] > 1.5f*boot[i]) return 0;
    }
    return 1;
}
#endif
