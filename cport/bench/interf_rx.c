/* interf_rx: push an int16 raw recording through the streaming receiver
 * -- the receiver the boards run -- and print every event it raises.
 * Host twin of "what does a board hear while a third party keys over
 * the channel"; driven by experiments/interference.py.
 *
 *   interf_rx [-m MODE]... [-c CHUNK] [-k NET_KEY] FILE
 *
 * MODE 0/1/2 = NORMAL/ROBUST/EXTREME (repeatable; every instance is fed
 * the same samples, as on the boards). Output, one line per event:
 *
 *   EV <mode> <type> <start_abs> <snr_db> <nbits> <bits as 0/1>
 *   END <samples> preempts <mode>:<count>...
 *
 * type 1 = decoded (CRC ok), -1 header CRC, -2 bad ver / oversize block,
 * -3 data CRC. Failed commits are printed too: a receiver that commits
 * on a stranger's preamble and fails its header is deaf for that time,
 * and that is the quantity the experiment is after. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/rx_stream.h"
#include "../src/tx.h"
#include "../src/packets.h"

static void print_event(int mode, const rxs_event_t *ev)
{
    int i, n = ev->type == 1 ? ev->pkt_bits_n : 0;
    printf("EV %d %d %d %.2f %d ", mode, ev->type, ev->start_abs,
           ev->snr_db, n);
    for (i = 0; i < n; i++)
        putchar(ev->bits[i] ? '1' : '0');
    putchar('\n');
}

int main(int argc, char **argv)
{
    int modes[3] = { 0, 0, 0 }, n_modes = 0, chunk = 256, i, m;
    const char *path = NULL;
    rxs_t *rx[3] = { 0, 0, 0 };
    int16_t *buf;
    long n, pos;
    FILE *f;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-m") && i + 1 < argc) {
            int md = atoi(argv[++i]);
            if (md < 0 || md > 2) { fprintf(stderr, "bad mode\n"); return 2; }
            modes[md] = 1; n_modes++;
        } else if (!strcmp(argv[i], "-c") && i + 1 < argc) {
            chunk = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-k") && i + 1 < argc) {
#ifndef INTERF_OLD   /* -DINTERF_OLD: build against pre-campaign sources for A/B */
            packets_set_net_key((uint8_t)atoi(argv[++i]));  /* our net key */
#else
            ++i;
#endif
        } else {
            path = argv[i];
        }
    }
    if (!path || !n_modes) {
        fprintf(stderr, "usage: interf_rx -m MODE [-m MODE] [-c CHUNK] FILE\n");
        return 2;
    }
    f = fopen(path, "rb");
    if (!f) { perror(path); return 2; }
    fseek(f, 0, SEEK_END);
    n = ftell(f) / 2;
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t)n * sizeof *buf);
    if (!buf || fread(buf, sizeof *buf, (size_t)n, f) != (size_t)n) {
        fprintf(stderr, "read failed\n"); return 2;
    }
    fclose(f);

    for (m = 0; m < 3; m++)
        if (modes[m]) rx[m] = rxs_open((link_mode_t)m, 0);

    for (pos = 0; pos < n; pos += chunk) {
        int c = (int)(n - pos < chunk ? n - pos : chunk);
        for (m = 0; m < 3; m++) {
            rxs_event_t ev;
            if (rx[m] && rxs_push(rx[m], buf + pos, c, &ev))
                print_event(m, &ev);
        }
    }
    for (m = 0; m < 3; m++) {
        rxs_event_t ev;
        if (rx[m] && rxs_flush(rx[m], &ev))
            print_event(m, &ev);
    }
    printf("END %ld preempts", n);
#ifndef INTERF_OLD
    for (m = 0; m < 3; m++)
        if (rx[m]) printf(" %d:%lld", m, (long long)rxs_preempts(rx[m]));
    printf(" notches %d\n", rxs_notches());
#else
    printf(" 0:0 notches 0\n");
#endif
    free(buf);
    return 0;
}
