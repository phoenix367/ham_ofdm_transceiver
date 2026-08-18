#include <string.h>

#include "ldpc.h"
#include "rom_ldpc.h"

#define BIG ((int64_t)1 << 30)

int ldpc_cc_elements(int bits_count)
{
    return LDPC_N - (LDPC_K - bits_count);
}

void ldpc_encode(const uint8_t *bits, int bits_count, uint8_t *out)
{
    uint8_t info[LDPC_K], acc = 0;
    int c, j, k = 0;

    memset(info, 0, sizeof(info));
    for (j = 0; j < bits_count; j++)
        info[j] = bits[j] & 1;

    /* transmit order = info[:k] then all 512 accumulator parity bits */
    for (j = 0; j < bits_count; j++)
        out[k++] = info[j];
    for (c = 0; c < LDPC_M; c++) {
        uint8_t s = 0;
        for (j = 0; j < LDPC_CI_MAX; j++) {
            int v = LDPC_CHK_INFO[c * LDPC_CI_MAX + j];
            if (v >= 0)
                s ^= info[v];
        }
        acc ^= s; /* p_i = p_{i-1} XOR chk_sum_i */
        out[k++] = acc;
    }
}

/* integer normalized min-sum, standard convention inside (positive = 0) */
static void min_sum_int(const int64_t *llr, uint8_t *bits)
{
    static int64_t v2c[LDPC_M * LDPC_DC_MAX];
    static int64_t c2v[LDPC_M * LDPC_DC_MAX];
    static int64_t totals[LDPC_N];
    int c, j, it, v;

    for (c = 0; c < LDPC_M; c++)
        for (j = 0; j < LDPC_DC_MAX; j++) {
            int vv = LDPC_EV[c * LDPC_DC_MAX + j];
            v2c[c * LDPC_DC_MAX + j] = vv >= 0 ? llr[vv] : BIG;
        }

    for (v = 0; v < LDPC_N; v++)
        bits[v] = llr[v] < 0;

    for (it = 0; it < LDPC_MAX_ITER; it++) {
        int syn_any = 0;

        for (c = 0; c < LDPC_M; c++) {
            int64_t *row = &v2c[c * LDPC_DC_MAX];
            int64_t m1 = BIG, m2 = BIG;
            int idx1 = 0, row_sign = 1;
            for (j = 0; j < LDPC_DC_MAX; j++) {
                int64_t a = row[j] < 0 ? -row[j] : row[j];
                if (row[j] < 0)
                    row_sign = -row_sign;
                if (a < m1) { /* strict: first minimum, as np.argmin */
                    m2 = m1;
                    m1 = a;
                    idx1 = j;
                } else if (a < m2) {
                    m2 = a;
                }
            }
            for (j = 0; j < LDPC_DC_MAX; j++) {
                int vv = LDPC_EV[c * LDPC_DC_MAX + j];
                if (vv >= 0) {
                    int sgn = row[j] >= 0 ? 1 : -1;
                    int64_t mn = (j == idx1) ? m2 : m1;
                    int64_t scaled = mn - (mn >> 2); /* alpha = 0.75 */
                    c2v[c * LDPC_DC_MAX + j] =
                        (int64_t)(row_sign * sgn) * scaled;
                } else {
                    c2v[c * LDPC_DC_MAX + j] = 0;
                }
            }
        }

        memcpy(totals, llr, sizeof(totals));
        for (c = 0; c < LDPC_M; c++)
            for (j = 0; j < LDPC_DC_MAX; j++) {
                int vv = LDPC_EV[c * LDPC_DC_MAX + j];
                if (vv >= 0)
                    totals[vv] += c2v[c * LDPC_DC_MAX + j];
            }

        for (v = 0; v < LDPC_N; v++)
            bits[v] = totals[v] < 0;

        for (c = 0; c < LDPC_M && !syn_any; c++) {
            int s = 0;
            for (j = 0; j < LDPC_DC_MAX; j++) {
                int vv = LDPC_EV[c * LDPC_DC_MAX + j];
                if (vv >= 0)
                    s ^= bits[vv];
            }
            syn_any = s;
        }
        if (!syn_any)
            return;

        for (c = 0; c < LDPC_M; c++)
            for (j = 0; j < LDPC_DC_MAX; j++) {
                int vv = LDPC_EV[c * LDPC_DC_MAX + j];
                v2c[c * LDPC_DC_MAX + j] =
                    vv >= 0 ? totals[vv] - c2v[c * LDPC_DC_MAX + j] : BIG;
            }
    }
}

void ldpc_decode_int(const int64_t *soft, int soft_len, int bits_count,
                     uint8_t *out)
{
    static int64_t llr[LDPC_N];
    static uint8_t bits[LDPC_N];
    int n_tx = ldpc_cc_elements(bits_count);
    int64_t pin, peak = 0;
    int i;

    memset(llr, 0, sizeof(llr));
    /* transmit positions: info[:bits_count], then parity K..N-1 */
    for (i = 0; i < n_tx; i++) {
        int64_t s = i < soft_len ? soft[i] : 0;
        int pos = i < bits_count ? i : LDPC_K + (i - bits_count);
        int64_t a = s < 0 ? -s : s;
        llr[pos] = s;
        if (a > peak)
            peak = a;
    }
    pin = peak * 4;
    if (pin < 1024)
        pin = 1024;
    for (i = bits_count; i < LDPC_K; i++)
        llr[i] = -pin; /* shortened zeros (ours: negative = 0) */

    for (i = 0; i < LDPC_N; i++)
        llr[i] = -llr[i]; /* to the standard convention */

    min_sum_int(llr, bits);
    for (i = 0; i < bits_count; i++)
        out[i] = bits[i];
}
