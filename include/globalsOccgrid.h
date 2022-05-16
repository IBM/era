#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

/* This is OCC-GRID Internal Timing information (gathering resources) */
extern struct timeval ocgr_c2g_total_stop,ocgr_c2g_total_start;
extern uint64_t ocgr_c2g_total_sec;
extern uint64_t ocgr_c2g_total_usec;

extern struct timeval ocgr_c2g_initCM_stop, ocgr_c2g_initCM_start;
extern uint64_t ocgr_c2g_initCM_sec;
extern uint64_t ocgr_c2g_initCM_usec;

extern struct timeval ocgr_c2g_updOrig_stop, ocgr_c2g_updOrig_start;
extern uint64_t ocgr_c2g_updOrig_sec;
extern uint64_t ocgr_c2g_updOrig_usec;

extern struct timeval ocgr_c2g_updBnds_stop, ocgr_c2g_updBnds_start;
extern uint64_t ocgr_c2g_updBnds_sec;
extern uint64_t ocgr_c2g_updBnds_usec;

extern struct timeval ocgr_upBd_total_stop, ocgr_upBd_total_start;
extern uint64_t ocgr_upBd_total_sec;
extern uint64_t ocgr_upBd_total_usec;

extern struct timeval ocgr_upBd_rayFSp_stop, ocgr_upBd_rayFSp_start;
extern uint64_t ocgr_upBd_rayFSp_sec;
extern uint64_t ocgr_upBd_rayFSp_usec;

extern struct timeval ocgr_upBd_regObst_stop, ocgr_upBd_regObst_start;
extern uint64_t ocgr_upBd_regObst_sec;
extern uint64_t ocgr_upBd_regObst_usec;

