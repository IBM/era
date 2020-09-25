#ifndef INC_TYPE_H
#define INC_TYPE_H

#include <complex.h>

#define Nbit 32
#define Nbit_integer 10

#define Nbit_ext 42
#define Nbit_integer_ext 14

#define SIZE_HISTORY 64
//#define CHUNK_vett 16
//#define CHUNK 1024
//#define CHUNK 16*1024 // for orig_input_data
//#define CHUNK 41772
#define RAW_DATA_IN_MAX_SIZE  41772    // this is more than large enough (41281 from erav3 or 41692 from GR file)


#define COEFF_LENGTH          64
#define COMPLEX_COEFF_LENGTH  48

#define SYNC_LENGTH  320

typedef float complex fc_test;

// JDW : Consider adding CNL::fixed_point library for C++ fixed-point types...? 
/* typedef ap_fixed<Nbit,Nbit_integer> T_type; */
/* typedef ap_fixed<Nbit_ext,Nbit_integer_ext> T_type_ext; */
/* typedef float T_type; */
/* typedef float T_type_ext; */

typedef float complex fx_pt;
typedef float complex fx_pt_ext;
typedef float fx_pt1;
typedef float fx_pt1_ext;

typedef struct dma_info {
	unsigned index;
	unsigned length;
} dma_info_t;

#endif
