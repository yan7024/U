/*------------------------------------------------------------------------------
* datum.c : datum transformation
*
*          Copyright (C) 2007 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/02/08 1.0 new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define MAXPRM  400000              /* max number of parameter records */

typedef struct {                    /* datum trans parameter type */
    int code;                       /* mesh code */
    float db,dl;                    /* difference of latitude/longitude (sec) */
} tprm_t;

static tprm_t *prm=NULL;            /* datum trans parameter table */
static int n=0;                     /* datum trans parameter table size */

/* compare datum trans parameters --------------------------------------------*/
static int cmpprm(const void *p1, const void *p2)
{
    tprm_t *q1=(tprm_t *)p1,*q2=(tprm_t *)p2;
    return q1->code-q2->code;
}
/* search datum trans parameter ----------------------------------------------*/
static int searchprm(double lat, double lon)
{
    int i,j,k,n1,m1,n2,m2,code;
    
    lon-=6000.0;
    n1=(int)(lat/40.0); lat-=n1*40.0;
    m1=(int)(lon/60.0); lon-=m1*60.0;
    n2=(int)(lat/5.0); lat-=n2*5.0;
    m2=(int)(lon/7.5); lon-=m2*7.5;
    code=n1*1000000+m1*10000+n2*1000+m2*100+(int)(lat/0.5)*10+(int)(lon/0.75);
    
    for (i=0,j=n-1;i<j;) { /* binary search */
        k=(i+j)/2;
        if (prm[k].code==code) return k;
        if (prm[k].code<code) i=k+1; else j=k;
    }
    return -1;
}
/* tokyo datum to jgd2000 lat/lon corrections --------------------------------*/
static int dlatdlon(const double *post, double *dpos)
{
    double lat=post[0]*R2D*60.0,lon=post[1]*R2D*60.0; /* arcmin */
    double dlat=0.5,dlon=0.75,db[2][2],dl[2][2],a,b,c,d;
    int i,j,k;
    
    if (n==0) return -1;
    for (i=0;i<2;i++) for (j=0;j<2;j++) {
        if ((k=searchprm(lat+i*dlat,lon+j*dlon))<0) return -1;
        db[i][j]=prm[k].db; dl[i][j]=prm[k].dl;
    }
    a=lat/dlat-(int)(lat/dlat); c=1.0-a;
    b=lon/dlon-(int)(lon/dlon); d=1.0-b;
    dpos[0]=(db[0][0]*c*d+db[1][0]*a*d+db[0][1]*c*b+db[1][1]*a*b)*D2R/3600.0;
    dpos[1]=(dl[0][0]*c*d+dl[1][0]*a*d+dl[0][1]*c*b+dl[1][1]*a*b)*D2R/3600.0;
    return 0;
}
/* load datum transformation parameter -----------------------------------------
* load datum transformation parameter
* args   : char  *file      I   datum trans parameter file path
* return : status (0:ok,0>:error)
* notes  : parameters file shall comply with GSI TKY2JGD.par
*-----------------------------------------------------------------------------*/
extern int loaddatump(const char *file)
{
    FILE *fp;
    char buff[256];
    
    if (n>0) return 0; /* already loaded */
    
    if (!(fp=fopen(file,"r"))) {
        fprintf(stderr,"%s : datum prm file open error : %s\n",__FILE__,file);
        return -1;
    }
    if (!(prm=(tprm_t *)malloc(sizeof(tprm_t)*MAXPRM))) {
        fprintf(stderr,"%s : memory allocation error\n",__FILE__);
        return -1;
    }
    while (fgets(buff,sizeof(buff),fp)&&n<MAXPRM) {
        if (sscanf(buff,"%d %f %f",&prm[n].code,&prm[n].db,&prm[n].dl)>=3) n++;
    }
    fclose(fp);
    qsort(prm,n,sizeof(tprm_t),cmpprm); /* sort parameter table */
    return 0;
}
/* tokyo datum to JGD2000 datum ------------------------------------------------
* transform position in Tokyo datum to JGD2000 datum
* args   : double *pos      I   position in Tokyo datum   {lat,lon,h} (rad,m)
*                           O   position in JGD2000 datum {lat,lon,h} (rad,m)
* return : status (0:ok,0>:error,out of range)
* notes  : before calling, call loaddatump() to set parameter table
*-----------------------------------------------------------------------------*/
extern int tokyo2jgd(double *pos)
{
    double post[2],dpos[2];
    
    post[0]=pos[0];
    post[1]=pos[1];
    if (dlatdlon(post,dpos)) return -1;
    pos[0]=post[0]+dpos[0];
    pos[1]=post[1]+dpos[1];
    return 0;
}
/* JGD2000 datum to Tokyo datum ------------------------------------------------
* transform position in JGD2000 datum to Tokyo datum
* args   : double *pos      I   position in JGD2000 datum {lat,lon,h} (rad,m)
*                           O   position in Tokyo datum   {lat,lon,h} (rad,m)
* return : status (0:ok,0>:error,out of range)
* notes  : before calling, call loaddatump() to set parameter table
*-----------------------------------------------------------------------------*/
extern int jgd2tokyo(double *pos)
{
    double posj[2],dpos[2];
    int i;
    
    posj[0]=pos[0];
    posj[1]=pos[1];
    for (i=0;i<2;i++) {
        if (dlatdlon(pos,dpos)) return -1;
        pos[0]=posj[0]-dpos[0];
        pos[1]=posj[1]-dpos[1];
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * RTKLIB-Lite stubs for STM32L476 memory-constrained build.
 * These APIs are high-cost optional models (PPP/SSR/SBAS/precise products).
 * We provide safe fallbacks to keep relative RTK core linkable.
 * ---------------------------------------------------------------------------*/
extern void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav,
                      double *dant)
{
    (void)time; (void)rs; (void)sat; (void)nav;
    dant[0]=dant[1]=dant[2]=0.0;
}

extern int sbssatcorr(gtime_t time, int sat, const nav_t *nav, double *rs,
                      double *dts, double *var)
{
    (void)time; (void)sat; (void)nav; (void)rs; (void)dts;
    if (var) *var=0.0;
    return 0;
}

extern int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                    double *rs, double *dts, double *var)
{
    (void)time; (void)sat; (void)nav; (void)opt; (void)rs; (void)dts;
    if (var) *var=0.0;
    return 0;
}

extern int iontec(gtime_t time, const nav_t *nav, const double *pos,
                  const double *azel, int opt, double *delay, double *var)
{
    (void)time; (void)nav; (void)pos; (void)azel; (void)opt;
    if (delay) *delay=0.0;
    if (var) *var=0.0;
    return 0;
}

extern int sbsioncorr(gtime_t time, const nav_t *nav, const double *pos,
                      const double *azel, double *delay, double *var)
{
    (void)time; (void)nav; (void)pos; (void)azel;
    if (delay) *delay=0.0;
    if (var) *var=0.0;
    return 0;
}

extern double sbstropcorr(gtime_t time, const double *pos, const double *azel,
                          double *var)
{
    (void)time; (void)pos; (void)azel;
    if (var) *var=0.0;
    return 0.0;
}

extern void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
                     const double *odisp, double *dr)
{
    (void)tutc; (void)rr; (void)opt; (void)erp; (void)odisp;
    dr[0]=dr[1]=dr[2]=0.0;
}

extern int pppoutstat(rtk_t *rtk, char *buff)
{
    (void)rtk;
    if (buff) buff[0]='\0';
    return 0;
}

extern int pppnx(const prcopt_t *opt)
{
    (void)opt;
    return 0;
}

extern void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    (void)rtk; (void)obs; (void)n; (void)nav;
}

/* decode GPS/QZSS LNAV subframe 1 ------------------------------------------*/
static int decode_lnav_subfrm1(const uint8_t *buff, eph_t *eph)
{
    double tow, toc;
    int i = 48, week, iodc0, iodc1;

    week      = (int)getbitu(buff, i, 10); i += 10;
    eph->code = (int)getbitu(buff, i,  2); i +=  2;
    eph->sva  = (int)getbitu(buff, i,  4); i +=  4;
    eph->svh  = (int)getbitu(buff, i,  6); i +=  6;
    iodc0     = (int)getbitu(buff, i,  2); i +=  2;
    eph->flag = (int)getbitu(buff, i,  1); i += 88; /* L2 P flag + reserved */
    eph->tgd[0] = getbits(buff, i, 8) * P2_31; i += 8;
    iodc1     = (int)getbitu(buff, i,  8); i +=  8;
    toc       = getbitu(buff, i, 16) * 16.0; i += 16;
    eph->f2   = getbits(buff, i,  8) * P2_55; i +=  8;
    eph->f1   = getbits(buff, i, 16) * P2_43; i += 16;
    eph->f0   = getbits(buff, i, 22) * P2_31;

    tow = getbitu(buff, 24, 17) * 6.0;
    eph->iodc = (iodc0 << 8) | iodc1;
    eph->week = adjgpsweek(week);
    eph->toc = gpst2time(eph->week, toc);
    eph->ttr = gpst2time(eph->week, tow);

    return 1;
}

/* decode GPS/QZSS LNAV subframe 2 ------------------------------------------*/
static int decode_lnav_subfrm2(const uint8_t *buff, eph_t *eph)
{
    double sqrtA;
    int i = 48;

    eph->iode = (int)getbitu(buff, i,  8);              i +=  8;
    eph->crs  =      getbits(buff, i, 16) * P2_5;       i += 16;
    eph->deln =      getbits(buff, i, 16) * P2_43 * SC2RAD; i += 16;
    eph->M0   =      getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
    eph->cuc  =      getbits(buff, i, 16) * P2_29;      i += 16;
    eph->e    =      getbitu(buff, i, 32) * P2_33;      i += 32;
    eph->cus  =      getbits(buff, i, 16) * P2_29;      i += 16;
    sqrtA     =      getbitu(buff, i, 32) * P2_19;      i += 32;
    eph->toes =      getbitu(buff, i, 16) * 16.0;       i += 16;
    eph->fit  =      getbitu(buff, i,  1) ? 0.0 : 4.0;

    eph->A = sqrtA * sqrtA;
    eph->toe = gpst2time(eph->week, eph->toes);

    return 1;
}

/* decode GPS/QZSS LNAV subframe 3 ------------------------------------------*/
static int decode_lnav_subfrm3(const uint8_t *buff, eph_t *eph)
{
    int i = 48, iode;

    eph->cic  = getbits(buff, i, 16) * P2_29;           i += 16;
    eph->OMG0 = getbits(buff, i, 32) * P2_31 * SC2RAD;  i += 32;
    eph->cis  = getbits(buff, i, 16) * P2_29;           i += 16;
    eph->i0   = getbits(buff, i, 32) * P2_31 * SC2RAD;  i += 32;
    eph->crc  = getbits(buff, i, 16) * P2_5;            i += 16;
    eph->omg  = getbits(buff, i, 32) * P2_31 * SC2RAD;  i += 32;
    eph->OMGd = getbits(buff, i, 24) * P2_43 * SC2RAD;  i += 24;
    iode      = (int)getbitu(buff, i, 8);               i +=  8;
    eph->idot = getbits(buff, i, 14) * P2_43 * SC2RAD;

    if (eph->iode != iode) return 0;

    return 1;
}

static gtime_t time_near_ref(gtime_t t, gtime_t ref)
{
    double dt;

    if (ref.time == 0) return t;
    dt = timediff(t, ref);
    if      (dt < -302400.0) t = timeadd(t,  604800.0);
    else if (dt >= 302400.0) t = timeadd(t, -604800.0);
    return t;
}

extern int decode_frame(const uint8_t *buff, eph_t *eph, alm_t *alm,
                        double *ion, double *utc)
{
    int id1, id2, id3;

    (void)alm;

    if (buff == NULL) return 0;

    /*
     * Minimal embedded restore: GPS/QZSS LNAV ephemeris from subframes 1/2/3.
     * Other optional decode paths stay stubbed to keep the L476 build small.
     */
    if (eph == NULL) {
        if (ion) memset(ion, 0, sizeof(double) * 8u);
        if (utc) memset(utc, 0, sizeof(double) * 8u);
        return 0;
    }

    id1 = (int)getbitu(buff,      43, 3);
    id2 = (int)getbitu(buff + 30, 43, 3);
    id3 = (int)getbitu(buff + 60, 43, 3);
    if (id1 != 1 || id2 != 2 || id3 != 3) return 0;

    if (!decode_lnav_subfrm1(buff,      eph)) return 0;
    if (!decode_lnav_subfrm2(buff + 30, eph)) return 0;
    if (!decode_lnav_subfrm3(buff + 60, eph)) return 0;

    eph->toe = time_near_ref(eph->toe, eph->ttr);
    eph->toc = time_near_ref(eph->toc, eph->ttr);

    return 1;
}

extern int test_glostr(const uint8_t *buff)
{
    (void)buff;
    return 0;
}

extern int decode_glostr(const uint8_t *buff, geph_t *geph, double *utc)
{
    (void)buff; (void)geph; (void)utc;
    return 0;
}

extern int decode_bds_d1(const uint8_t *buff, eph_t *eph, double *ion,
                         double *utc)
{
    (void)buff; (void)eph; (void)ion; (void)utc;
    return 0;
}

extern int decode_bds_d2(const uint8_t *buff, eph_t *eph, double *utc)
{
    (void)buff; (void)eph; (void)utc;
    return 0;
}

extern int decode_gal_inav(const uint8_t *buff, eph_t *eph, double *ion,
                           double *utc)
{
    (void)buff; (void)eph; (void)ion; (void)utc;
    return 0;
}

extern int sbsdecodemsg(gtime_t time, int prn, const uint32_t *words,
                        sbsmsg_t *sbsmsg)
{
    (void)time; (void)prn; (void)words; (void)sbsmsg;
    return 0;
}
