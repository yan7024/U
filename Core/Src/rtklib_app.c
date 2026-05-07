/*
 * MCU wiring (default RTK_DEBUG_PORT_UART1=0):
 * - Rover: F9P TX -> PA10 (USART1_RX), MCU PA9 TX -> F9P RX (binary UBX CFG from link_train).
 * - USART3 PC4/PC5: NTRIP/RTCM 输入（含 1019 星历合并进 nav）+ 调试串口；非 RTK 差分定位链路。
 * - 定位：默认仅 SPP（pntpos，PMODE_SINGLE），串口 $PRTK 输出经纬度；RTK 差分需 MCU_SOLVE_RTK=1 另行编译。
 * - 回放：-DMCU_REPLAY_RTCM_OBS=1 时 USART3 RTCM 观测拷入 rover->obs；工具见 tools/replay_rtcm_to_mcu.py。
 * - USART2 PA2/PA3: ST-Link VCP；调试文本镜像（no $PRTKMON dedicated here）。
 * - u-center: use F9P USB or second UART — not USART3（留给 MCU NTRIP）。
 */
#include "rtklib_app.h"
#include "rtklib_port.h"
#include "usart.h"
#include "rtklib.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RTK_DEBUG_ENABLE 1
#define RTK_MON_INTERVAL_MS 1000u
/* Limit solve cadence so UART RX keeps draining and $PRTKMON stays periodic */
#define RTK_SOLVE_MIN_MS 200u
#define RTK_UART_DRAIN_MAX 384u /* bytes per port per tick — avoids starving TX/$PRTKMON */
/* Rover/base epoch time alignment (s); after GPS-week fold, allow serial latency */
#define RTK_EPOCH_DT_MAX_SEC 10.0
#define GPS_HALF_WEEK_SEC 302400.0
#define GPS_FULL_WEEK_SEC 604800.0
/* NTRIP idle: no new USART3 (base) bytes → drop MSM obs so MON does not show stale basN */
#define RTK_RX2_IDLE_FLUSH_MS 2500u
/*
 * 1: USART3 上 RTCM 观测（1004/1074/1077 等 decode 后 obs.n>0）复制到 rover->obs 做 SPP 回放；
 *    断开 F9P USART1 或忽略 UBX；用于官方 RINEX→str2str→COM 验证 MCU 解算。
 *    编译加 -DMCU_REPLAY_RTCM_OBS=1
 */
#ifndef MCU_REPLAY_RTCM_OBS
#define MCU_REPLAY_RTCM_OBS 0
#endif

static raw_t *s_rover = NULL;
static rtcm_t *s_base = NULL;
static rtk_t *s_rtk = NULL;
static raw_t s_rover_storage;
static rtk_t s_rtk_storage;
/* Place rtcm_t in 32 KiB SRAM2 — keeps ~30 KiB headroom in main 96 KiB RAM for raw/rtk + heap */
#if defined(__GNUC__)
__attribute__((section(".bss_ram2")))
#endif
static rtcm_t s_base_storage;
static prcopt_t s_opt;
static prcopt_t s_opt_spp;
static uint8_t s_init_ok = 0u;
static uint8_t s_have_rover_obs = 0u;
static uint8_t s_have_base_obs = 0u;
static uint32_t s_last_status_ms = 0u;
static uint32_t s_last_sol_itow = 0xFFFFFFFFu;
static uint32_t s_rtkpos_call_total = 0u;
static uint32_t s_rtkpos_ok_total = 0u;
static uint32_t s_pntpos_call_total = 0u;
static uint32_t s_pntpos_ok_total = 0u;
static int32_t s_epoch_dt_ms = -1; /* ms between rover/base obs time; -1 if unknown */
static uint32_t s_last_solve_ms = 0u;
static uint32_t s_rx2_last_total;
static uint32_t s_rx2_last_chg_ms;
static uint8_t s_rx2_seen_once;
static uint8_t s_uart1_fallback_38400_done;
static uint32_t s_rover_ubx_idle_ticks;
static int s_mon_eph_gps = -1;
static int s_mon_eph_any_gps = -1; /* GPS eph slots filled (ignore toe vs rover time) */
static uint32_t s_ubx_eph_decode_total; /* input_ubx ret==2: RXM-SFRBX decoded GPS/QZS broadcast ephem */
static char s_rtk_fail_snip[72];

static int gtime_gpst_plausible(gtime_t t);

static int dbl_finite_small(double x)
{
    return (x == x) && (fabs(x) < 1.0e12);
}

static void refresh_mon_nav_metrics(const nav_t *nav, gtime_t tr)
{
    int k, prn, sys, c = 0, any = 0;
    double toe_win;

    if (nav == NULL || !gtime_gpst_plausible(tr)) {
        s_mon_eph_gps = -1;
        s_mon_eph_any_gps = -1;
        return;
    }
    for (k = 0; k < nav->nmax; k++) {
        if (nav->eph[k].sat <= 0) continue;
        sys = satsys(nav->eph[k].sat, &prn);
        if (sys == SYS_GPS) {
            if (prn < 1 || prn > NSATGPS) continue;
            toe_win = MAXDTOE + 3600.0;
#if NSYSGAL > 0
        } else if (sys == SYS_GAL) {
            if (prn < MINPRNGAL || prn > MAXPRNGAL) continue;
            toe_win = MAXDTOE_GAL + 3600.0;
#endif
        } else
            continue;
        any++;
        if (fabs(timediff(nav->eph[k].toe, tr)) <= toe_win) c++;
    }
    s_mon_eph_any_gps = any;
    s_mon_eph_gps = c;
}

static void snip_fail_msg(char *dst, size_t dstsz, const char *src)
{
    size_t i, j;

    if (dstsz == 0) return;
    if (src == NULL || src[0] == '\0') {
        dst[0] = '\0';
        return;
    }
    for (i = 0, j = 0; src[i] != '\0' && j + 1 < dstsz; i++) {
        unsigned char c = (unsigned char)src[i];

        if (c == '\r' || c == '\n' || c == '\t') continue;
        dst[j++] = (char)((c == ',' || c == ';') ? '|' : c);
    }
    dst[j] = '\0';
}

/*
 * RTCM type 1019 fills rtcm->nav.eph[]; rtkpos/pntpos read s_rover->nav only.
 * Many NTRIP streams carry GPS ephemeris — merge into rover nav when present.
 */
#if MCU_REPLAY_RTCM_OBS
/* Official/RINEX replay: MSM on USART3 → same rtcm_t obs buffer → copy as rover for pntpos */
static void copy_rtcm_obs_to_rover(void)
{
    int i, n;

    if (s_rover == NULL || s_base == NULL) return;
    n = s_base->obs.n;
    if (n <= 0 || n > s_rover->obs.nmax) return;

    memcpy(s_rover->obs.data, s_base->obs.data, sizeof(obsd_t) * (size_t)n);
    for (i = 0; i < n; i++) {
        s_rover->obs.data[i].rcv = 1;
    }
    s_rover->obs.n = n;
    s_rover->time = s_rover->obs.data[0].time;
    s_have_rover_obs = 1u;
}
#endif

static void merge_rtcm_brdc_eph_into_rover(void)
{
    int i;

    if (s_base == NULL || s_rover == NULL) return;
    for (i = 0; i < s_base->nav.n && i < s_rover->nav.nmax; i++) {
        int sat = s_base->nav.eph[i].sat;
        int sys;

        if (sat <= 0) continue;
        sys = satsys(sat, NULL);
        if (sys != SYS_GPS
#if NSYSGAL > 0
            && sys != SYS_GAL
#endif
            )
            continue;
        if (sat > s_rover->nav.nmax) continue;
        s_rover->nav.eph[sat - 1] = s_base->nav.eph[i];
    }
}

/*
 * RTCM 1019/1045 merge can leave eph.week far from rover RAWX GPST; pick full GPS week w
 * (1024..4200) minimizing |gpst2time(w,toes)−tr|. Galileo uses same eph_t toe/week fields.
 */
static void align_rover_nav_brdc_eph_to_obs(gtime_t tr)
{
    int k, prn, sys, wk, best_w, w_ref, w;
    double best_adt, adt, toes, dshift, tow_dummy;
    gtime_t toe_old, toe_new;
    eph_t *eph;

    if (s_rover == NULL || !gtime_gpst_plausible(tr)) return;

    (void)time2gpst(tr, &w_ref);

    for (k = 0; k < s_rover->nav.nmax; k++) {
        eph = &s_rover->nav.eph[k];
        if (eph->sat <= 0) continue;
        sys = satsys(eph->sat, &prn);
        if (sys == SYS_GPS) {
            if (prn < 1 || prn > NSATGPS) continue;
#if NSYSGAL > 0
        } else if (sys == SYS_GAL) {
            if (prn < MINPRNGAL || prn > MAXPRNGAL) continue;
#endif
        } else
            continue;

        toes = eph->toes;
        if (!(toes == toes) || toes < 0.0 || toes > 604800.0) continue;

        if (gtime_gpst_plausible(eph->toe)) {
            toe_old = eph->toe;
        } else if (eph->week >= 1024) {
            toe_old = gpst2time(eph->week, toes);
        } else {
            toe_old = gpst2time(w_ref, toes);
        }
        /*
         * decode_type1019 uses adjgpsweek(timeget()); on MCU timeget() is uptime, not UTC,
         * so eph.week/toe can land outside gtime_gpst_plausible() — do not skip alignment.
         */
        if (!gtime_gpst_plausible(toe_old)) {
            toe_old = gpst2time(w_ref, toes);
        }
        if (!gtime_gpst_plausible(toe_old)) continue;

        best_w = w_ref;
        toe_new = gpst2time(w_ref, toes);
        best_adt = gtime_gpst_plausible(toe_new) ? fabs(timediff(toe_new, tr)) : 1e300;
        for (w = 1024; w <= 4200; w++) {
            toe_new = gpst2time(w, toes);
            if (!gtime_gpst_plausible(toe_new)) continue;
            adt = fabs(timediff(toe_new, tr));
            if (adt < best_adt) {
                best_adt = adt;
                best_w = w;
            }
        }
        toe_new = gpst2time(best_w, toes);
        dshift = timediff(toe_new, toe_old);
        eph->toe = toe_new;
        eph->week = best_w;
        tow_dummy = time2gpst(eph->toe, &wk);
        (void)tow_dummy;
        eph->toes = tow_dummy;
        eph->week = wk;
        eph->toc = timeadd(eph->toc, dshift);
    }
}

/*
 * GPS week integer difference folded to about ±512 (1024-week rollover).
 * Without this, rover full week (e.g. 2296) vs RTCM-side week folded to 10-bit sense (e.g. 248)
 * gives labs(2048)>52 and process_base clears base obs every tick → basN stays 0.
 */
static int gps_week_fold_diff(int wa, int wb)
{
    long d = (long)wa - (long)wb;

    while (d > 512L)
        d -= 1024L;
    while (d < -512L)
        d += 1024L;
    return (int)d;
}

static void fold_gps_week_dt(double *dt)
{
    /*
     * Fold difference into about ±half-week (handles base/rover week ambiguity).
     * Never use unbounded while-loops: timediff garbage or +Inf makes those hang forever
     * and kills $MAIN/$PRTKMON after the first solve attempt.
     */
    double x = *dt;
    const double full = GPS_FULL_WEEK_SEC;
    const double half = GPS_HALF_WEEK_SEC;

    if (!(x == x)) /* NaN */
        return;
    x -= floor((x + half) / full) * full;
    *dt = x;
}

static void update_epoch_dt_display(double dt_abs)
{
    double ms;
    if (!dbl_finite_small(dt_abs)) {
        s_epoch_dt_ms = -2;
        return;
    }
    ms = dt_abs * 1000.0;
    /* Avoid printing INT32_MAX from boundary equality with ms == INT32_MAX exactly */
    if (!dbl_finite_small(ms) || ms >= (double)INT32_MAX) {
        s_epoch_dt_ms = -3;
        return;
    }
    s_epoch_dt_ms = (int32_t)(ms + 0.5);
}

/* GPST sanity for MCU pairing — junk/zero slots in obs buffers skew earliest_obs badly */
static int gtime_gpst_plausible(gtime_t t)
{
    int week;
    double tow = time2gpst(t, &week);

    if (week < 1800 || week > 3500) return 0;
    if (tow < -1.0 || tow > 604801.0) return 0;
    return 1;
}

/* RTCM 1005/1006 ARP ECEF (m); RTKLIB stream server would fill opt->rb on desktop */
static int base_ecef_from_rtcm_ok(const double pos[3])
{
    double r2 = pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2];
    double rm = sqrt(r2);
    if (!(rm == rm) || r2 <= 0.0) return 0;
    if (rm < 6.0e6 || rm > 7.0e6) return 0;
    return 1;
}

static int sync_base_rb_from_rtcm(void)
{
    int i;

    if (s_base == NULL || s_rtk == NULL) return 0;
    if (!base_ecef_from_rtcm_ok(s_base->sta.pos)) return 0;
    for (i = 0; i < 3; i++) {
        s_rtk->opt.rb[i] = s_base->sta.pos[i];
        s_opt.rb[i] = s_base->sta.pos[i];
    }
    return 1;
}

/* Earliest plausible sample time (read-only; avoids sortobs() reshaping obs->n each tick). */
static gtime_t earliest_plausible_obs_time(const obs_t *obs)
{
    int k;
    int ok = 0;
    gtime_t tmin;
    gtime_t z = {0};

    if (obs == NULL || obs->n <= 0) return z;
    for (k = 0; k < obs->n; k++) {
        if (!gtime_gpst_plausible(obs->data[k].time)) continue;
        if (!ok || timediff(obs->data[k].time, tmin) < 0.0) {
            tmin = obs->data[k].time;
            ok = 1;
        }
    }
    return ok ? tmin : z;
}

static void send_line(const char *line)
{
#if RTK_DEBUG_ENABLE
    if (line == NULL) return;
    rtklib_port_debug_send((const uint8_t *)line, (uint16_t)strlen(line));
#else
    (void)line;
#endif
}

static const char *sol_str(int stat)
{
    switch (stat) {
        case SOLQ_FIX: return "RTK_FIX";
        case SOLQ_FLOAT: return "RTK_FLOAT";
        case SOLQ_DGPS: return "DGPS";
        case SOLQ_SINGLE: return "SINGLE";
        default: return "NONE";
    }
}

static int carr_soln_from_stat(int stat)
{
    if (stat == SOLQ_FIX) return 2;   /* fixed integer ambiguities */
    if (stat == SOLQ_FLOAT) return 1; /* float ambiguities */
    return 0;
}

static void publish_solution(const sol_t *sol)
{
    char out[220];
    int n;
    uint32_t itow_ms;
    int week;
    double pos[3];
    int32_t lat1e7, lon1e7, hmm;
    int32_t lat_abs, lon_abs, h_abs;
    char lat_s, lon_s, h_s;

    if (sol == NULL) return;

    itow_ms = (uint32_t)(time2gpst(sol->time, &week) * 1000.0 + 0.5);
    if (itow_ms == s_last_sol_itow) return;
    s_last_sol_itow = itow_ms;

    ecef2pos(sol->rr, pos);
    lat1e7 = (int32_t)(pos[0] * R2D * 1e7);
    lon1e7 = (int32_t)(pos[1] * R2D * 1e7);
    hmm = (int32_t)(pos[2] * 1000.0);
    lat_abs = (lat1e7 < 0) ? -lat1e7 : lat1e7;
    lon_abs = (lon1e7 < 0) ? -lon1e7 : lon1e7;
    h_abs = (hmm < 0) ? -hmm : hmm;
    lat_s = (lat1e7 < 0) ? '-' : '+';
    lon_s = (lon1e7 < 0) ? '-' : '+';
    h_s = (hmm < 0) ? '-' : '+';

    n = snprintf(out, sizeof(out),
                 "$PRTK,mode=%s,q=%d,carrSoln=%d,ns=%u,lat=%c%ld.%07ld,lon=%c%ld.%07ld,h=%c%ld.%03ld,diffAge=%.2f,ratio=%.2f,itow=%lu\r\n",
                 sol_str(sol->stat),
                 sol->stat,
                 carr_soln_from_stat(sol->stat),
                 (unsigned)sol->ns,
                 lat_s, (long)(lat_abs / 10000000), (long)(lat_abs % 10000000),
                 lon_s, (long)(lon_abs / 10000000), (long)(lon_abs % 10000000),
                 h_s, (long)(h_abs / 1000), (long)(h_abs % 1000),
                 (double)sol->age,
                 (double)sol->ratio,
                 (unsigned long)itow_ms);
    if (n > 0 && n < (int)sizeof(out)) {
#if RTK_DEBUG_ENABLE
        rtklib_port_debug_send((const uint8_t *)out, (uint16_t)n);
#endif
    }
}

static void process_rover(void)
{
    uint8_t b;
    int ret;
    uint32_t drain;
    uint32_t got = 0u;

    rtklib_serial_set_read_port(1u); /* USART1: rover UBX raw */
    for (drain = 0u; drain < RTK_UART_DRAIN_MAX; drain++) {
        if (rtklib_uart_read(&b, 1) != 1) break;
        got++;
        ret = input_ubx(s_rover, b);
        if (ret == 1) {
            s_have_rover_obs = 1u;
        } else if (ret == 2) {
            s_ubx_eph_decode_total++;
        }
    }
    /*
     * If the UART goes quiet mid-frame (truncated/stale stream), input_ubx stays
     * stuck waiting for raw->len bytes — clear parser after idle so resync works.
     */
    if (got == 0u && s_rover != NULL && s_rover->nbyte > 0) {
        if (++s_rover_ubx_idle_ticks >= 150u) {
            s_rover->nbyte = 0;
            s_rover_ubx_idle_ticks = 0u;
        }
    } else {
        s_rover_ubx_idle_ticks = 0u;
    }
}

static void process_base(void)
{
    uint8_t b;
    int ret;
    uint32_t drain;

    /*
     * MCU: NO_SYSTEMTIME + rtklib_time.c stubs make timeget() ≈ uptime, not GPST.
     * RTCM adjweek() seeds rtcm->time from that when time==0 → wrong GPS week vs UBX RAWX.
     * Seed rtcm->time from the rover raw clock so MSM epochs align; drop stale base obs.
     */
    if (s_rover != NULL && s_base != NULL && s_rover->time.time != 0) {
        int wr = 0;
        int wb = 0;

        (void)time2gpst(s_rover->time, &wr);
        (void)time2gpst(s_base->time, &wb);
        if (s_base->time.time == 0 || wb < 1024 ||
            labs((long)gps_week_fold_diff(wr, wb)) > 52) {
            s_base->time = s_rover->time;
            s_base->obs.n = 0;
            s_have_base_obs = 0u;
        }
    }

    rtklib_serial_set_read_port(2u); /* logical port 2 = USART3 ring: base RTCM3 / NTRIP */
    for (drain = 0u; drain < RTK_UART_DRAIN_MAX; drain++) {
        if (rtklib_uart_read(&b, 1) != 1) break;
        ret = input_rtcm3(s_base, b);
        if (ret == 1) {
            s_have_base_obs = 1u;
#if MCU_REPLAY_RTCM_OBS
            copy_rtcm_obs_to_rover();
#endif
        }
    }
    merge_rtcm_brdc_eph_into_rover();
}

#if MCU_SOLVE_RTK
/*
 * rtkpos() counts rover/base by scanning obs from the start: all rcv==1, then all rcv==2.
 * sortobs() sorts by *time* first; if rover/base epochs differ by >DTTOL (~1 s is common),
 * the earlier receiver block comes first and nu becomes 0 → pntpos(obs,0,…) fails → rtkOk=0.
 * RTKLIB convention here: order by receiver, then satellite (time last for stability).
 */
static int cmp_obs_for_rtkpos(const void *p, const void *q)
{
    const obsd_t *a = (const obsd_t *)p;
    const obsd_t *b = (const obsd_t *)q;
    double tt;

    if (a->rcv != b->rcv) {
        return (int)a->rcv - (int)b->rcv;
    }
    if (a->sat != b->sat) {
        return (int)a->sat - (int)b->sat;
    }
    tt = timediff(a->time, b->time);
    if (fabs(tt) > DTTOL) {
        return (tt < 0.0) ? -1 : 1;
    }
    return 0;
}

/* Returns 1 if rtkpos() succeeded and published. */
static int try_solve_rtk(void)
{
    obsd_t obs_mix[MAXOBS * 2];
    int i, j, n = 0;
    double dt;
    gtime_t tr, tb;

    if (!s_have_rover_obs || !s_have_base_obs) return 0;
    if (s_rover->obs.n <= 0 || s_base->obs.n <= 0) {
        s_epoch_dt_ms = -1;
        return 0;
    }

    tr = earliest_plausible_obs_time(&s_rover->obs);
    tb = earliest_plausible_obs_time(&s_base->obs);

    if (!gtime_gpst_plausible(tr) || !gtime_gpst_plausible(tb)) {
        s_epoch_dt_ms = -8;
        return 0;
    }
    refresh_mon_nav_metrics(&s_rover->nav, tr);

    dt = timediff(tr, tb);
    fold_gps_week_dt(&dt);
    if (dt < 0.0) dt = -dt;

    update_epoch_dt_display(dt);

    if (!dbl_finite_small(dt) || dt > GPS_HALF_WEEK_SEC)
        return 0;
    if (dt > RTK_EPOCH_DT_MAX_SEC) {
        /*
         * Typical after NTRIP drop: rover epochs advance, last MSM stays fixed → dt grows.
         * Flush base obs so basN goes 0 until corrections resume (epochDtMs=-9).
         */
        if (s_base != NULL) {
            s_base->obs.n = 0;
            s_have_base_obs = 0u;
        }
        s_epoch_dt_ms = -9;
        return 0;
    }

    if (!sync_base_rb_from_rtcm()) {
        /* Need RTCM station coords (e.g. types 1005/1006); MSM alone has no ARP */
        s_epoch_dt_ms = -11;
        return 0;
    }

    for (i = 0; i < s_rover->obs.n && n < (MAXOBS * 2); i++) {
        obs_mix[n] = s_rover->obs.data[i];
        obs_mix[n].rcv = 1;
        n++;
    }
    for (i = 0; i < s_base->obs.n && n < (MAXOBS * 2); i++) {
        obs_mix[n] = s_base->obs.data[i];
        obs_mix[n].rcv = 2;
        n++;
    }
    if (n <= 0) return 0;

    qsort(obs_mix, (size_t)n, sizeof(obs_mix[0]), cmp_obs_for_rtkpos);
    for (i = 0, j = 0; i < n; i++) {
        if (obs_mix[i].sat != obs_mix[j].sat || obs_mix[i].rcv != obs_mix[j].rcv ||
            timediff(obs_mix[i].time, obs_mix[j].time) != 0.0) {
            j++;
            obs_mix[j] = obs_mix[i];
        }
    }
    n = j + 1;
    if (n <= 0) return 0;

    s_rtk->neb = 0;
    s_rtk->errbuf[0] = '\0';

    s_rtkpos_call_total++;
    if (rtkpos(s_rtk, obs_mix, n, &s_rover->nav)) {
        s_rtkpos_ok_total++;
        s_rtk_fail_snip[0] = '\0';
        publish_solution(&s_rtk->sol);
        return 1;
    }
    snip_fail_msg(s_rtk_fail_snip, sizeof(s_rtk_fail_snip), s_rtk->errbuf);
    return 0;
}
#endif /* MCU_SOLVE_RTK */

static void bump_obs0_to_plausible_gpst(obs_t *obs)
{
    int k, imin = -1;
    gtime_t tmin = {0};

    if (obs == NULL || obs->n <= 0) return;
    for (k = 0; k < obs->n; k++) {
        if (!gtime_gpst_plausible(obs->data[k].time)) continue;
        if (imin < 0 || timediff(obs->data[k].time, tmin) < 0.0) {
            imin = k;
            tmin = obs->data[k].time;
        }
    }
    if (imin > 0) {
        obsd_t tmp = obs->data[0];

        obs->data[0] = obs->data[imin];
        obs->data[imin] = tmp;
    }
}

static void try_solve_spp(void)
{
    gtime_t tr;
    sol_t sol = {{0}};
    char msg[128];

    if (!s_have_rover_obs || s_rover->obs.n <= 0) return;

    tr = earliest_plausible_obs_time(&s_rover->obs);
    if (!gtime_gpst_plausible(tr)) return;

    refresh_mon_nav_metrics(&s_rover->nav, tr);

    /* satposs() uses obs[0].time — must not be a junk slot while others are valid RAWX */
    bump_obs0_to_plausible_gpst(&s_rover->obs);

    s_pntpos_call_total++;
    if (pntpos(s_rover->obs.data, s_rover->obs.n, &s_rover->nav, &s_opt_spp, &sol, NULL, s_rtk->ssat,
               msg)) {
        s_pntpos_ok_total++;
        s_rtk_fail_snip[0] = '\0';
        publish_solution(&sol);
    } else {
        snip_fail_msg(s_rtk_fail_snip, sizeof(s_rtk_fail_snip), msg);
    }
}

static void solve_once(void)
{
    gtime_t tr_obs = {0};

    if (s_rover != NULL && s_rover->obs.n > 0) {
        tr_obs = earliest_plausible_obs_time(&s_rover->obs);
    } else if (s_rover != NULL) {
        tr_obs = s_rover->time;
    }
    if (gtime_gpst_plausible(tr_obs)) {
        align_rover_nav_brdc_eph_to_obs(tr_obs);
    }

#if MCU_SOLVE_RTK
    if (try_solve_rtk()) {
        return;
    }
#endif
    try_solve_spp();
}

void rtklib_init(void)
{
    char msg[128];
    rtklib_port_allocator_reset();

    s_rover = &s_rover_storage;
    s_rtk = &s_rtk_storage;
    s_base = &s_base_storage;
    memset(s_rover, 0, sizeof(raw_t));
    memset(s_rtk, 0, sizeof(rtk_t));
    memset(s_base, 0, sizeof(rtcm_t));

    /* init_raw() only fails on NULL in embedded build; use storage address directly */
    if (!init_raw(&s_rover_storage, STRFMT_UBX)) {
        send_line("$PRTKMON,ERR,init_raw_failed\r\n");
        return;
    }
    if (!init_rtcm(s_base)) {
        send_line("$PRTKMON,ERR,init_rtcm_failed\r\n");
        return;
    }

    s_opt = prcopt_default;
    s_opt.mode = PMODE_KINEMA;
    s_opt.refpos = POSOPT_POS; /* base ECEF in opt.rb — synced from RTCM sta.pos each epoch */
    s_opt.nf = 1; /* L476 light profile */
    s_opt.navsys = SYS_GPS
#if NSYSGAL > 0
                     | SYS_GAL
#endif
        ;
    s_opt.modear = ARMODE_CONT;
    s_opt.glomodear = 0;
    s_opt.bdsmodear = 1;
    s_opt.sateph = EPHOPT_BRDC;
    s_opt.ionoopt = IONOOPT_BRDC;
    s_opt.tropopt = TROPOPT_SAAS;
    s_opt.elmin = 10.0 * D2R;
    s_opt.dynamics = 0;
    s_opt_spp = s_opt;
    s_opt_spp.mode = PMODE_SINGLE;
    s_opt_spp.modear = ARMODE_OFF;
    /* SPP：略放宽仰角，便于弱天空下凑够 ≥4 颗参与迭代（仍受 NV/eph 限制） */
    s_opt_spp.elmin = 5.0 * D2R;
    /* 默认 maxgdop=30；星少时 GDOP 易超阈值，SPP 放宽拒绝门限（精度换可用性） */
    s_opt_spp.maxgdop = 150.0;
#if MCU_SOLVE_RTK
    rtkinit(s_rtk, &s_opt);
#else
    /* SPP-only：内部 opt 与 pntpos 一致（单点解 / 输出 $PRTK 经纬度） */
    rtkinit(s_rtk, &s_opt_spp);
#endif

    s_init_ok = 1u;
    {
        const char *dbgport =
#if RTK_DEBUG_PORT_UART1
            "UART1_PA9_TTL";
#else
            "USART3_PC4PC5_TTL";
#endif
        snprintf(msg, sizeof(msg),
#if MCU_SOLVE_RTK
                 "$PRTKMON,BOOT_OK,MCU_RTK,nf=%d,maxobs=%d,dbg=%s\r\n",
#else
                 "$PRTKMON,BOOT_OK,MCU_SPP,nf=%d,maxobs=%d,dbg=%s\r\n",
#endif
                 s_opt.nf, MAXOBS, dbgport);
    }
    send_line(msg);
}

void rtklib_rover_ubx_link_train(uint32_t link_baud)
{
    /*
     * MCU USART1 baud must match the receiver's current UART baud for CFG bytes to be understood.
     * CFG-PRT payload always sets the receiver to 115200 8N1 UBX (mode 0x8D0); then we restore MCU to 115200.
     */
    static const char *const k_ubx_tail[] = {
        "CFG-RATE 100 1 0",
        "CFG-MSG 1 7 0 1 0 0 0 0",
        "CFG-MSG 2 21 0 1 0 0 0 0",
        "CFG-MSG 2 19 0 1 0 0 0 0",
    };
    uint8_t buf[160];
    char prtline[96];
    size_t k;
    int n;

    if (!s_init_ok) {
        return;
    }

    usart1_apply_baud(link_baud);

    snprintf(prtline, sizeof(prtline), "CFG-PRT 1 0 0 2256 %lu 1 1 0 0", 115200UL);
    n = gen_ubx(prtline, buf);
    if (n > 8) {
        rtklib_port_uart1_send(buf, (uint16_t)n);
        rtklib_delay_ms(100u);
    }
    for (k = 0; k < sizeof(k_ubx_tail) / sizeof(k_ubx_tail[0]); k++) {
        n = gen_ubx(k_ubx_tail[k], buf);
        if (n > 8) {
            rtklib_port_uart1_send(buf, (uint16_t)n);
            rtklib_delay_ms(80u);
        }
    }
    rtklib_delay_ms(150u);
    usart1_apply_baud(115200u);
}

void rtklib_process(void)
{
    uint32_t now;
    char mon[512];
    int n;

    now = HAL_GetTick();

    if (!s_init_ok) {
        if ((now - s_last_status_ms) >= RTK_MON_INTERVAL_MS) {
            s_last_status_ms = now;
            send_line("$PRTKMON,ERR,init_not_ready\r\n");
        }
        return;
    }

    /* Rover stuck at factory 38400: one-shot re-train (MCU matches link, then both end at 115200) */
    if (s_uart1_fallback_38400_done == 0u && now >= 3000u &&
        rtklib_serial_get_rx1_total() == 0u) {
        s_uart1_fallback_38400_done = 1u;
        send_line("$PRTKMON,NOTE,u1_rx0_retry_link38400\r\n");
        rtklib_rover_ubx_link_train(38400u);
    }

    process_rover();
    process_base();

    now = HAL_GetTick();
    {
        uint32_t r2 = rtklib_serial_get_rx2_total();

        if (r2 != s_rx2_last_total) {
            s_rx2_last_total = r2;
            s_rx2_last_chg_ms = now;
            s_rx2_seen_once = 1u;
        }
#if !MCU_REPLAY_RTCM_OBS
        else if (s_rx2_seen_once != 0u && s_base != NULL && s_base->obs.n > 0 &&
                 (now - s_rx2_last_chg_ms) >= RTK_RX2_IDLE_FLUSH_MS) {
            s_base->obs.n = 0;
            s_have_base_obs = 0u;
            s_epoch_dt_ms = -10;
        }
#endif
    }

    if ((now - s_last_solve_ms) >= RTK_SOLVE_MIN_MS) {
        s_last_solve_ms = now;
        solve_once();
    }

    /*
     * MON after merge + align: ephG/ephA match this tick's rover nav (avoids one-cycle lie).
     */
    now = HAL_GetTick();
    if ((now - s_last_status_ms) >= RTK_MON_INTERVAL_MS) {
        s_last_status_ms = now;
        if (s_rover != NULL && s_rover->obs.n > 0) {
            gtime_t trm = earliest_plausible_obs_time(&s_rover->obs);

            if (gtime_gpst_plausible(trm)) {
                refresh_mon_nav_metrics(&s_rover->nav, trm);
            }
        }
        {
            rtklib_ubx_diag_t udx;

            rtklib_ubx_diag_snapshot(&udx);
            n = snprintf(
                mon, sizeof(mon),
                "$PRTKMON,rx1=%lu,rx2=%lu,ubxSync=%lu,rtcmP2=%lu,rtcmP1=%lu,rovN=%d,basN=%d,epochDtMs=%ld,"
                "rawx=%lu,sfrbx=%lu,sfrbxG=%lu,sfSat=%u,sfPrn=%u,sfId=%u,sfMask=%u,"
                "ckOk=%lu,ckErr=%lu,lenErr=%lu,ephDec=%lu,dFrm=%lu,sfIdErr=%lu,cNav=%lu,"
                "ephG=%d,ephA=%d,m1019=%lu,ubxEph=%lu,rtkCall=%lu,rtkOk=%lu,pntCall=%lu,pntOk=%lu,fail=%s\r\n",
                (unsigned long)rtklib_serial_get_rx1_total(),
                (unsigned long)rtklib_serial_get_rx2_total(),
                (unsigned long)rtklib_serial_get_ubx_sync_total(),
                (unsigned long)rtklib_serial_get_rtcm_preamble_total(),
                (unsigned long)rtklib_serial_get_rtcm_preamble_uart1_total(),
                s_rover->obs.n,
                s_base->obs.n,
                (long)s_epoch_dt_ms,
                (unsigned long)udx.ok_rawx,
                (unsigned long)udx.ok_sfrbx,
                (unsigned long)udx.sfrbx_gps,
                (unsigned int)udx.last_gps_sat,
                (unsigned int)udx.last_gps_prn,
                (unsigned int)udx.last_sf_id,
                (unsigned int)udx.sf_mask,
                (unsigned long)udx.frm_ck_ok,
                (unsigned long)udx.frm_ck_err,
                (unsigned long)udx.frm_len_err,
                (unsigned long)udx.eph_dec,
                (unsigned long)udx.dfrm_fail,
                (unsigned long)udx.sf_id_err,
                (unsigned long)udx.cnav_unsup,
                s_mon_eph_gps,
                s_mon_eph_any_gps,
                (unsigned long)((s_base != NULL) ? s_base->nmsg3[19] : 0u),
                (unsigned long)s_ubx_eph_decode_total,
                (unsigned long)s_rtkpos_call_total,
                (unsigned long)s_rtkpos_ok_total,
                (unsigned long)s_pntpos_call_total,
                (unsigned long)s_pntpos_ok_total,
                (s_rtk_fail_snip[0] != '\0') ? s_rtk_fail_snip : "-");
        }
        if (n > 0 && n < (int)sizeof(mon)) {
#if RTK_DEBUG_ENABLE
            rtklib_port_debug_send((const uint8_t *)mon, (uint16_t)n);
#endif
        }
    }
}

int rtklib_get_base_obs_ready(void)
{
    if (!s_init_ok || s_base == NULL) return 0;
    return (s_base->obs.n > 0) ? 1 : 0;
}
