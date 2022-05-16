#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>

/* This is XMIT PIPE internal Timing information (gathering resources) */
extern struct timeval x_pipe_stop, x_pipe_start;
extern uint64_t x_pipe_sec;
extern uint64_t x_pipe_usec;

extern struct timeval x_genmacfr_stop, x_genmacfr_start;
extern uint64_t x_genmacfr_sec;
extern uint64_t x_genmacfr_usec;

extern struct timeval x_domapwk_stop, x_domapwk_start;
extern uint64_t x_domapwk_sec;
extern uint64_t x_domapwk_usec;

extern struct timeval xdmw_total_stop, xdmw_total_start;
extern uint64_t xdmw_total_sec;
extern uint64_t xdmw_total_usec;

extern struct timeval xdmw_genDF_stop, xdmw_genDF_start;
extern uint64_t xdmw_genDF_sec;
extern uint64_t xdmw_genDF_usec;

extern struct timeval xdmw_scrmbl_stop, xdmw_scrmbl_start;
extern uint64_t xdmw_scrmbl_sec;
extern uint64_t xdmw_scrmbl_usec;

extern struct timeval xdmw_cnvEnc_stop, xdmw_cnvEnc_start;
extern uint64_t xdmw_cnvEnc_sec;
extern uint64_t xdmw_cnvEnc_usec;

extern struct timeval xdmw_punct_stop, xdmw_punct_start;
extern uint64_t xdmw_punct_sec;
extern uint64_t xdmw_punct_usec;

extern struct timeval xdmw_intlv_stop, xdmw_intlv_start;
extern uint64_t xdmw_intlv_sec;
extern uint64_t xdmw_intlv_usec;

extern struct timeval xdmw_symbls_stop, xdmw_symbls_start;
extern uint64_t xdmw_symbls_sec;
extern uint64_t xdmw_symbls_usec;

extern struct timeval xdmw_mapout_stop, xdmw_mapout_start;
extern uint64_t xdmw_mapout_sec;
extern uint64_t xdmw_mapout_usec;

extern struct timeval x_phdrgen_stop, x_phdrgen_start;
extern uint64_t x_phdrgen_sec;
extern uint64_t x_phdrgen_usec;

extern struct timeval x_ck2sym_stop, x_ck2sym_start;
extern uint64_t x_ck2sym_sec;
extern uint64_t x_ck2sym_usec;

extern struct timeval x_ocaralloc_stop, x_ocaralloc_start;
extern uint64_t x_ocaralloc_sec;
extern uint64_t x_ocaralloc_usec;

extern struct timeval x_fft_stop, x_fft_start;
extern uint64_t x_fft_sec;
extern uint64_t x_fft_usec;

extern struct timeval x_ocycpref_stop, x_ocycpref_start;
extern uint64_t x_ocycpref_sec;
extern uint64_t x_ocycpref_usec;

