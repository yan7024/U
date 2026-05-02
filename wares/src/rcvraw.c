#include "rtklib.h"
#include <string.h>

#if defined(RTKLIB_EMBEDDED)
static obsd_t s_emb_obs[MAXOBS];
static obsd_t s_emb_obuf[MAXOBS];
static eph_t s_emb_eph[RAW_NAV_EPH_SLOTS];
static geph_t s_emb_geph[1];
static seph_t s_emb_seph[1];
#endif

extern int init_raw(raw_t *raw, int format)
{
    obsd_t data0 = {{0}};
    eph_t eph0 = {0, -1, -1};
    geph_t geph0 = {0, -1};
    seph_t seph0 = {{0}};
    int i;

    if (raw == NULL) return 0;

    memset(raw, 0, sizeof(raw_t));
    raw->format = format;
    raw->time = gpst2time(0, 0.0);

    raw->obs.n = 0;
    raw->obs.nmax = MAXOBS;
    raw->obuf.n = 0;
    raw->obuf.nmax = MAXOBS;

#if defined(RTKLIB_EMBEDDED)
    raw->obs.data = s_emb_obs;
    raw->obuf.data = s_emb_obuf;

    raw->nav.nmax = RAW_NAV_EPH_SLOTS;
    raw->nav.ngmax = NSATGLO > 0 ? NSATGLO : 1;
    raw->nav.nsmax = NSATSBS > 0 ? NSATSBS : 1;
    raw->nav.nemax = 0;
    raw->nav.ncmax = 0;
    raw->nav.namax = 0;
    raw->nav.ntmax = 0;

    raw->nav.eph = s_emb_eph;
    raw->nav.geph = s_emb_geph;
    raw->nav.seph = s_emb_seph;
#else
    raw->obs.data = (obsd_t *)calloc((size_t)MAXOBS, sizeof(obsd_t));
    if (raw->obs.data == NULL) return 0;

    raw->obuf.data = (obsd_t *)calloc((size_t)MAXOBS, sizeof(obsd_t));
    if (raw->obuf.data == NULL) return 0;

    raw->nav.nmax = MAXSAT * 2;
    raw->nav.ngmax = NSATGLO > 0 ? NSATGLO : 1;
    raw->nav.nsmax = NSATSBS > 0 ? NSATSBS : 1;
    raw->nav.nemax = 0;
    raw->nav.ncmax = 0;
    raw->nav.namax = 0;
    raw->nav.ntmax = 0;

    raw->nav.eph = (eph_t *)calloc((size_t)raw->nav.nmax, sizeof(eph_t));
    raw->nav.geph = (geph_t *)calloc((size_t)raw->nav.ngmax, sizeof(geph_t));
    raw->nav.seph = (seph_t *)calloc((size_t)raw->nav.nsmax, sizeof(seph_t));
    if (raw->nav.eph == NULL || raw->nav.geph == NULL || raw->nav.seph == NULL) return 0;
#endif

    for (i = 0; i < MAXOBS; i++) {
        raw->obs.data[i] = data0;
        raw->obuf.data[i] = data0;
    }
    for (i = 0; i < raw->nav.nmax; i++) raw->nav.eph[i] = eph0;
    for (i = 0; i < raw->nav.ngmax; i++) raw->nav.geph[i] = geph0;
    for (i = 0; i < raw->nav.nsmax; i++) raw->nav.seph[i] = seph0;

    /* seleph()/selgeph() iterate nav->n / ng — streaming stores eph at slot sat-1 */
    raw->nav.n = raw->nav.nmax;
    raw->nav.ng = raw->nav.ngmax;
    raw->nav.ns = raw->nav.nsmax;

    return 1;
}

extern void free_raw(raw_t *raw)
{
    if (raw == NULL) return;

#if !defined(RTKLIB_EMBEDDED)
    if (raw->obs.data != NULL) free(raw->obs.data);
    if (raw->obuf.data != NULL) free(raw->obuf.data);
    if (raw->nav.eph != NULL) free(raw->nav.eph);
    if (raw->nav.geph != NULL) free(raw->nav.geph);
    if (raw->nav.seph != NULL) free(raw->nav.seph);
#endif

    memset(raw, 0, sizeof(raw_t));
}
