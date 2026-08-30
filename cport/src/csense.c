#include "csense.h"

void cs_init(csense_t *c)
{
    int i;
    for (i = 0; i < CS_WIN; i++)
        c->ring[i] = 0;
    c->pos = 0;
    c->acc = 0;
    c->mean = 0;
    c->floor_ = 1e9;             /* first real evaluation snaps it down */
    c->busy_since_ms = 0;
    c->climb_ms = 0;
}

void cs_feed(csense_t *c, int16_t v)
{
    int16_t old = c->ring[c->pos];
    c->acc += (int64_t)v * v - (int64_t)old * old;
    c->ring[c->pos] = v;
    if (++c->pos >= CS_WIN)
        c->pos = 0;
    c->mean = (uint32_t)(c->acc / CS_WIN);
}

int cs_busy(csense_t *c, uint32_t now_ms)
{
    double p = (double)c->mean;
    int busy;

    if (now_ms < CS_WARMUP_MS)
        return 0;                /* no verdict, floor untouched */
    if (p < c->floor_) {
        c->floor_ = p;
    } else if ((uint32_t)(now_ms - c->climb_ms) >= CS_CLIMB_MS) {
        c->floor_ *= 1.0005;
        c->climb_ms = now_ms;
    }
    if (c->floor_ < 25.0)
        c->floor_ = 25.0;
    busy = p > CS_RATIO_SQ * c->floor_;
    if (!busy) {
        c->busy_since_ms = 0;
    } else if (c->busy_since_ms == 0) {
        c->busy_since_ms = now_ms ? now_ms : 1;
    } else if ((uint32_t)(now_ms - c->busy_since_ms) > CS_REBASE_MS) {
        /* sustained energy longer than any frame IS the new floor */
        c->floor_ = p;
        c->busy_since_ms = 0;
        busy = 0;
    }
    return busy;
}
