/* Feed the host-link parser pure noise in random chunk sizes. It must
 * never read or write out of bounds and must never wedge -- which is
 * only meaningful under AddressSanitizer and UBSan, so this is built by
 * `make sanitize` and not on its own. 200k chunks of up to 400 bytes,
 * ~40 MB. A healthy run accepts a handful of accidental frames and
 * reports tens of millions of resyncs; the point is the absence of any
 * sanitizer report. */
#include <stdio.h>
#include <stdlib.h>
#include "usb_proto.h"
static long g_frames;
static void sink(void *c, uint8_t t, const uint8_t *p, int n)
{ (void)c;(void)t;(void)p;(void)n; g_frames++; }
int main(void)
{
    up_parser_t par; unsigned s = 12345; long it; int i;
    static uint8_t chunk[512];
    up_parser_init(&par);
    for (it = 0; it < 200000; it++) {
        int n = 1 + (int)(s = s*1103515245u+12345u, (s >> 16) % 400u);
        for (i = 0; i < n; i++) { s = s*1103515245u+12345u; chunk[i] = (uint8_t)(s >> 20); }
        up_parser_push(&par, chunk, n, sink, 0);
    }
    printf("fuzz: 200k random chunks, %ld frames accepted, %u resyncs, no faults\n",
           g_frames, par.resyncs);
    return 0;
}
