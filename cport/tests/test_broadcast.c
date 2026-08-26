/* Broadcast (non-ARQ) tests: the C waveform must match the fixed model
 * sample for sample, and the receive walk must reassemble the payload. */
#include <stdio.h>
#include <string.h>

#include "../src/packets.h"
#include "../src/tx.h"
#include "../src/broadcast.h"
#include "test_vectors.h"

static int g_pass, g_fail;

static void check(const char *name, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) g_pass++; else g_fail++;
}

static uint64_t fnv64(const int16_t *s, int n)
{
    uint64_t h = UINT64_C(14695981039346656037);
    int i;
    for (i = 0; i < n; i++) { h ^= (uint16_t)s[i]; h *= UINT64_C(1099511628211); }
    return h;
}

static int16_t g_air[600000];
static uint8_t g_out[4096];

int main(void)
{
    bc_stats_t st;
    int n, got;

    /* a broadcast group is tx_build_burst over PKT_TYP_BCAST frames */
    n = tx_build_burst(MODE_NORMAL, BC_BITS, BC_PKT_BITS_N, BC_GROUP,
                       PKT_TYP_BCAST, (mod_type_t)BC_MOD, (cc_rate_t)BC_SPD,
                       4, g_air);
    check("broadcast group bit-exact vs the fixed model",
          n == BC_LEN && fnv64(g_air, n) == BC_HASH);

    /* and the walk gets the payload back out of it */
    got = bc_receive(MODE_NORMAL, g_air, n, BC_GROUP, g_out,
                     (int)sizeof(g_out), &st);
    check("broadcast receive: payload byte-exact",
          got == BC_PAYLOAD_N && memcmp(g_out, BC_PAYLOAD, BC_PAYLOAD_N) == 0);
    check("broadcast receive: descriptor and EOS seen",
          st.ptype == BC_PT_TELEMETRY && st.saw_eos && st.groups == 1);
    check("broadcast receive: every frame accounted for",
          st.frames_ok == BC_GROUP && st.frames_lost == 0);
    printf("  broadcast: %d B, %d/%d frames, %d group(s), ptype %d\n",
           st.bytes_out, st.frames_ok, st.frames_ok + st.frames_lost,
           st.groups, st.ptype);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
