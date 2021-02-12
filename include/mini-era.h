#ifndef _MINI_ERA_H_
#define _MINI_ERA_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#ifndef __user
#define __user
#endif
#endif /* __KERNEL__ */

#include <esp.h>
#include <esp_accelerator.h>

struct vitdodec_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned cbps;
	unsigned ntraceback;
	unsigned data_bits;
	unsigned src_offset;
	unsigned dst_offset;
};

#define VITDODEC_IOC_ACCESS	_IOW ('S', 0, struct vitdodec_access)

typedef int8_t vitHW_token_t;


// This is for the FFT Accelerator

#if (USE_FFT_FX == 64)
//typedef unsigned long long fftHW_token_t;
typedef int64_t fftHW_token_t;
typedef double fftHW_native_t;
#define fx2float fixed64_to_double
#define float2fx double_to_fixed64
#define FX_IL 42
#elif (USE_FFT_FX == 32)
typedef int fftHW_token_t;
typedef float fftHW_native_t;
#define fx2float fixed32_to_float
#define float2fx float_to_fixed32
#define FX_IL 14
#endif /* FFT_FX_WIDTH */

/* <<--params-def-->> */
#define FFTHW_NO_INVERSE    0
#define FFTHW_NO_SHIFT      0
#define FFTHW_DO_INVERSE    1
#define FFTHW_DO_SHIFT      1

#define LOGN_SAMPLES 6
#define NUM_FFTS     1
#define DO_INVERSE   0
#define DO_SHIFT     1
#define SCALE_FACTOR 0

#define NACC 1

/* <<--params-->> */

struct fftHW_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned scale_factor;
	unsigned do_inverse;
	unsigned logn_samples;
	unsigned do_shift;
	unsigned num_ffts;
	unsigned src_offset;
	unsigned dst_offset;
};

#define FFTHW_IOC_ACCESS	_IOW ('S', 0, struct fftHW_access)

#endif /* _MINI_ERA_H_ */
