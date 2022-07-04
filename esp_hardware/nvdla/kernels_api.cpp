/*
 * Copyright 2019 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define BYPASS_KERAS_CV_CODE

#ifndef BYPASS_KERAS_CV_CODE
#include <Python.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
static unsigned DMA_WORD_PER_BEAT(unsigned _st)
{
        return (sizeof(void *) / _st);
}
#if defined(HW_VIT) || defined(HW_FFT)
 // These are includes from ESP Viterbi Butterfly2 Accelerator
 #include <fcntl.h>
 #include <pthread.h>
 #include <sys/types.h>
 #include <sys/mman.h>
 #include <sys/stat.h>
 #include <string.h>
 #include <sys/time.h>
 #include <unistd.h>

 #include "contig.h"

 // These are includes to support ESP FFT Accelerator
 //#include "libesp.h"
 // #include "fft_esp_cfg.h"
 #include "mini-era.h"

#endif

#include "kernels_api.h"

// NC add
#include "nvdla.h"
#include "miniera_nvdla.h"

#ifdef USE_SIM_ENVIRON
 #include "sim_environs.h"
#else
 #include "read_trace.h"
#endif
extern unsigned time_step;

char* lane_names[NUM_LANES] = {"LHazard", "Left", "Center", "Right", "RHazard" };
char* message_names[NUM_MESSAGES] = {"Safe_L_or_R", "Safe_R_only", "Safe_L_only", "Unsafe_L_or_R" };
char* object_names[NUM_OBJECTS] = {"Nothing", "Car", "Truck", "Person", "Bike" };


#ifdef VERBOSE
bool_t output_viz_trace = true;
#else
bool_t output_viz_trace = false;
#endif
unsigned fft_logn_samples = 14; // Defaults to 16k samples

unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')
unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N' };
float    nearest_dist[NUM_LANES] = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE };

unsigned hist_total_objs[NUM_LANES * MAX_OBJ_IN_LANE];

unsigned rand_seed = 0; // Only used if -r <N> option set

float IMPACT_DISTANCE = 50.0; // Minimum distance at which an obstacle "impacts" MyCar (collision case)


/* These are types, functions, etc. required for VITERBI */
#include "viterbi_flat.h"



#ifndef BYPASS_KERAS_CV_CODE
PyObject *pName, *pModule, *pFunc, *pFunc_load;
PyObject *pArgs, *pValue, *pretValue;
#define PY_SSIZE_T_CLEAN

char *python_module = "mio";
char *python_func = "predict";	  
char *python_func_load = "loadmodel";	  
#endif


/* These are some top-level defines needed for CV kernel */
/* #define IMAGE_SIZE  32  // What size are these? */
/* typedef struct { */
/*   unsigned int image_id; */
/*   label_t  object; */
/*   unsigned image_data[IMAGE_SIZE]; */
/* } cv_dict_entry_t; */

/** The CV kernel uses a different method to select appropriate inputs; dictionary not needed
unsigned int     num_cv_dictionary_items = 0;
cv_dict_entry_t* the_cv_object_dict;
**/
unsigned label_match[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0};  // Times CNN matched dictionary
unsigned label_lookup[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0}; // Times we used CNN for object classification
unsigned label_mismatch[NUM_OBJECTS][NUM_OBJECTS] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
  

/* These are some top-level defines needed for RADAR */
/* typedef struct { */
/*   unsigned int index; */
/*   unsigned int return_id; */
/*   float distance; */
/*   float return_data[2 * RADAR_N]; */
/* } radar_dict_entry_t; */

#define MAX_RDICT_ENTRIES   12   // This should be updated eventually...
unsigned int        num_radar_dictionary_items = 0;
radar_dict_entry_t* the_radar_return_dict;

unsigned radar_total_calc = 0;
unsigned hist_pct_errs[MAX_RDICT_ENTRIES][5];// = {0, 0, 0, 0, 0}; // One per distance, plus global?
unsigned hist_distances[MAX_RDICT_ENTRIES];
char*    hist_pct_err_label[5] = {"   0%", "<  1%", "< 10%", "<100%", ">100%"};

/* These are some top-level defines needed for VITERBI */
/* typedef struct { */
/*   unsigned int msg_num; */
/*   unsigned int msg_id; */
/*   ofdm_param   ofdm_p; */
/*   frame_param  frame_p; */
/*   uint8_t      in_bits[MAX_ENCODED_BITS]; */
/* } vit_dict_entry_t; */

uint8_t descramble[1600]; // I think this covers our max use cases
uint8_t actual_msg[1600];

unsigned int      num_viterbi_dictionary_items = 0;
vit_dict_entry_t* the_viterbi_trace_dict;

unsigned vit_msgs_size;
unsigned vit_msgs_per_step;
const char* vit_msgs_size_str[VITERBI_MSG_LENGTHS] = {"SHORT", "MEDIUM", "LONG", "MAXIMUM"};
const char* vit_msgs_per_step_str[VITERBI_MSGS_PER_STEP] = {"One message per time step",
							    "One message per obstacle per time step",
							    "One msg per obstacle + 1 per time step" };
unsigned viterbi_messages_histogram[VITERBI_MSG_LENGTHS][NUM_MESSAGES];

unsigned total_msgs = 0; // Total messages decoded during the full run
unsigned bad_decode_msgs = 0; // Total messages decoded incorrectly during the full run

#ifdef HW_VIT
// These are Viterbi Harware Accelerator Variales, etc.
#define VIT_DEVNAME	"/dev/vitdodec.0"

int vitHW_fd;
contig_handle_t vitHW_mem;
vitHW_token_t *vitHW_lmem;   // Pointer to local view of contig memory
vitHW_token_t *vitHW_li_mem; // Pointer to input memory block
vitHW_token_t *vitHW_lo_mem; // Pointer to output memory block
size_t vitHW_in_words_adj;
size_t vitHW_out_words_adj;
size_t vitHW_in_len;
size_t vitHW_out_len;
size_t vitHW_in_size;
size_t vitHW_out_size;
size_t vitHW_out_offset;
size_t vitHW_size;

struct vitdodec_access vitHW_desc;

static void init_vit_parameters()
{
	//printf("Doing init_vit_parameters\n");
	if (DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)) == 0) {
		vitHW_in_words_adj  = 24852;
		vitHW_out_words_adj = 18585;
	} else {
		vitHW_in_words_adj  = round_up(24852, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
		vitHW_out_words_adj = round_up(18585, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
	}
	vitHW_in_len = vitHW_in_words_adj;
	vitHW_out_len =  vitHW_out_words_adj;
	vitHW_in_size = vitHW_in_len * sizeof(vitHW_token_t);
	vitHW_out_size = vitHW_out_len * sizeof(vitHW_token_t);
	vitHW_out_offset = vitHW_in_len;
	vitHW_size = (vitHW_out_offset * sizeof(vitHW_token_t)) + vitHW_out_size;
}


#endif


#ifdef HW_FFT
// These are FFT Harware Accelerator Variables, etc.
#if (USE_FFT_FX == 64)
 #define FFT_DEVNAME  "/dev/fft.0"
#elif (USE_FFT_FX == 32)
 #define FFT_DEVNAME  "/dev/fft.0"
#else
 #define FFT_DEVNAME  "/dev/no-dev.0"
#endif

/* int32_t fftHW_log_len = FFTHW_LOG_LEN; */
/* int32_t fftHW_len     = FFTHW_LEN; */

int fftHW_fd;
contig_handle_t fftHW_mem;

fftHW_token_t* fftHW_lmem;  // Pointer to local version (mapping) of fftHW_mem
fftHW_token_t* fftHW_li_mem; // Pointer to input memory block
fftHW_token_t* fftHW_lo_mem; // Pointer to output memory block
size_t fftHW_in_words_adj;
size_t fftHW_out_words_adj;
size_t fftHW_in_len;
size_t fftHW_out_len;
size_t fftHW_in_size;
size_t fftHW_out_size;
size_t fftHW_out_offset;
size_t fftHW_size;

struct fftHW_access fftHW_desc;

const float FFT_ERR_TH = 0.05;

/* User-defined code */
static void init_fft_parameters()
{
	int len = 0x1<<14;
	if (DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)) == 0) {
		fftHW_in_words_adj  = 2 * len;
		fftHW_out_words_adj = 2 * len;
	} else {
		fftHW_in_words_adj = round_up(2 * len, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
		fftHW_out_words_adj = round_up(2 * len, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
	}
	fftHW_in_len = fftHW_in_words_adj;
	fftHW_out_len =  fftHW_out_words_adj;
	fftHW_in_size = fftHW_in_len * sizeof(fftHW_token_t);
	fftHW_out_size = fftHW_out_len * sizeof(fftHW_token_t);
	fftHW_out_offset = 0;
	fftHW_size = (fftHW_out_offset * sizeof(fftHW_token_t)) + fftHW_out_size;
}
#endif

extern void descrambler(uint8_t* in, int psdusize, char* out_msg, uint8_t* ref, uint8_t *msg);




status_t init_rad_kernel(char* dict_fn)
{
  DEBUG(printf("In init_rad_kernel...\n"));

  init_calculate_peak_dist(fft_logn_samples);

  // Read in the radar distances dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF)
  {
    printf("Error: unable to open dictionary file %s\n", dict_fn);
    return error;
  }
  // Read the number of definitions
  if (fscanf(dictF, "%u\n", &num_radar_dictionary_items))
    ;
  if (num_radar_dictionary_items > MAX_RDICT_ENTRIES) {
    printf("ERROR : Dictionary contains %u entries, but %u is MAX_RDICT_ENTRIES\n", num_radar_dictionary_items, MAX_RDICT_ENTRIES);
    exit(-2);
  }
  DEBUG(printf("  There are %u dictionary entries\n", num_radar_dictionary_items));
  
  the_radar_return_dict = (radar_dict_entry_t*)calloc(num_radar_dictionary_items, sizeof(radar_dict_entry_t));
  if (the_radar_return_dict == NULL) 
  {
    printf("ERROR : Cannot allocate Radar Trace Dictionary memory space");
    return error;
  }

  unsigned tot_dict_values = 0;
  for (int di = 0; di < num_radar_dictionary_items; di++) {
    unsigned entry_id;
    float entry_dist;
    unsigned entry_dict_values = 0;
    if (fscanf(dictF, "%u %f", &entry_id, &entry_dist))
      ;
    DEBUG(printf("  Reading rad dictionary entry %u : %u %f\n", di, entry_id, entry_dist));
    the_radar_return_dict[di].index = di;
    the_radar_return_dict[di].return_id = entry_id;
    the_radar_return_dict[di].distance =  entry_dist;
    DEBUG(printf("     Read rad dictionary entry %u : %u %f  : IDX %u\n", di, the_radar_return_dict[di].return_id, the_radar_return_dict[di].distance, the_radar_return_dict[di].index));
    for (int i = 0; i < 2*RADAR_N; i++) {
      float fin;
      if (fscanf(dictF, "%f", &fin))
	;
      the_radar_return_dict[di].return_data[i] = fin;
      tot_dict_values++;
      entry_dict_values++;
    }
    DEBUG(printf("    Read in dict entry %u with %u total values\n", di, entry_dict_values));
  }
  DEBUG(printf("  Read %u dict entries with %u values across them all\n", num_radar_dictionary_items, tot_dict_values));
  if (!feof(dictF)) {
    printf("NOTE: Did not hit eof on the radar dictionary file %s\n", dict_fn);
  }
  fclose(dictF);

  // Initialize hist_pct_errs values
  for (int di = 0; di < num_radar_dictionary_items; di++) {
    hist_distances[di] = 0;
    for (int i = 0; i < 5; i++) {
      hist_pct_errs[di][i] = 0;
    }
  }

#ifdef HW_FFT
  init_fft_parameters();
  printf("Open device %s\n", FFT_DEVNAME);
  #if (USE_FFT_FX == 64)
   printf(" typedef unsigned long long token_t\n");
   printf(" typedef double native_t\n");
   printf(" #define fx2float fixed64_to_double\n");
   printf(" #define float2fx double_to_fixed64\n");
  #elif (USE_FFT_FX == 32)
   printf(" typedef int token_t\n");
   printf(" typedef float native_t\n");
   printf(" #define fx2float fixed32_to_float\n");
   printf(" #define float2fx float_to_fixed32\n");
  #endif /* FFT_FX_WIDTH */
  printf(" #define FX_IL %u\n", FX_IL);

  fftHW_fd = open(FFT_DEVNAME, O_RDWR, 0);
  if (fftHW_fd < 0) {
    fprintf(stderr, "Error: cannot open %s", FFT_DEVNAME);
    exit(EXIT_FAILURE);
  }

  printf("Allocate hardware buffer of size %zu\n", fftHW_size);
  fftHW_lmem = contig_alloc(fftHW_size, &fftHW_mem);
  if (fftHW_lmem == NULL) {
    fprintf(stderr, "Error: cannot allocate %zu contig bytes", fftHW_size);
    exit(EXIT_FAILURE);
  }

  fftHW_li_mem = &(fftHW_lmem[0]);
  fftHW_lo_mem = &(fftHW_lmem[fftHW_out_offset]);
  printf("Set fftHW_li_mem = %p  AND fftHW_lo_mem = %p\n", fftHW_li_mem, fftHW_lo_mem);

  fftHW_desc.esp.run = true;
  fftHW_desc.esp.coherence = ACC_COH_NONE;
  fftHW_desc.esp.p2p_store = 0;
  fftHW_desc.esp.p2p_nsrcs = 0;
  //fftHW_desc.esp.p2p_srcs = {"", "", "", ""};
  fftHW_desc.esp.contig = contig_to_khandle(fftHW_mem);

#ifdef HW_FFT_BITREV
  fftHW_desc.do_bitrev  = FFTHW_DO_BITREV;
#else
  fftHW_desc.do_bitrev  = FFTHW_NO_BITREV;
#endif
  //fftHW_desc.len      = fftHW_len;
  fftHW_desc.log_len    = fft_logn_samples; // fftHW_log_len;
  fftHW_desc.src_offset = 0;
  fftHW_desc.dst_offset = 0;
#endif

  return success;
}


/* This is the initialization of the Viterbi dictionary data, etc.
 * The format is:
 *  <n> = number of dictionary entries (message types)
 * For each dictionary entry:
 *  n1 n2 n3 n4 n5 : OFDM parms: 
 *  m1 m2 m3 m4 m5 : FRAME parms:
 *  x1 x2 x3 ...   : The message bits (input to decode routine)
 */

status_t init_vit_kernel(char* dict_fn)
{
  DEBUG(printf("In init_vit_kernel...\n"));
  // Read in the object images dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF)
  {
    printf("Error: unable to open viterbi dictionary definition file %s\n", dict_fn);
    return error;
  }

  // Read in the trace message dictionary from the trace file
  // Read the number of messages
  if (fscanf(dictF, "%u\n", &num_viterbi_dictionary_items))
    ;
  DEBUG(printf("  There are %u dictionary entries\n", num_viterbi_dictionary_items));
  the_viterbi_trace_dict = (vit_dict_entry_t*)calloc(num_viterbi_dictionary_items, sizeof(vit_dict_entry_t));
  if (the_viterbi_trace_dict == NULL) 
  {
    printf("ERROR : Cannot allocate Viterbi Trace Dictionary memory space");
    return error;
  }

  // Read in each dictionary item
  for (int i = 0; i < num_viterbi_dictionary_items; i++) 
  {
    DEBUG(printf("  Reading vit dictionary entry %u\n", i)); //the_viterbi_trace_dict[i].msg_id));

    int mnum, mid;
    if (fscanf(dictF, "%d %d\n", &mnum, &mid))
      ;
    DEBUG(printf(" V_MSG: num %d Id %d\n", mnum, mid));
    if (mnum != i) {
      printf("ERROR : Check Viterbi Dictionary : i = %d but Mnum = %d  (Mid = %d)\n", i, mnum, mid);
      exit(-5);
    }
    //the_viterbi_trace_dict[i].msg_id = mnum;
    //the_viterbi_trace_dict[i].msg_id = mid;
	// NC add
    the_viterbi_trace_dict[i].msg_id = static_cast<message_t>(mnum);
    the_viterbi_trace_dict[i].msg_id = static_cast<message_t>(mid);

    int in_bpsc, in_cbps, in_dbps, in_encoding, in_rate; // OFDM PARMS
    if (fscanf(dictF, "%d %d %d %d %d\n", &in_bpsc, &in_cbps, &in_dbps, &in_encoding, &in_rate))
      ;
    DEBUG(printf("  OFDM: %d %d %d %d %d\n", in_bpsc, in_cbps, in_dbps, in_encoding, in_rate));
    //the_viterbi_trace_dict[i].ofdm_p.encoding   = in_encoding;
    the_viterbi_trace_dict[i].ofdm_p.encoding   = static_cast<Encoding>(in_encoding);  // NC
    the_viterbi_trace_dict[i].ofdm_p.n_bpsc     = in_bpsc;
    the_viterbi_trace_dict[i].ofdm_p.n_cbps     = in_cbps;
    the_viterbi_trace_dict[i].ofdm_p.n_dbps     = in_dbps;
    the_viterbi_trace_dict[i].ofdm_p.rate_field = in_rate;

    int in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits;
    if (fscanf(dictF, "%d %d %d %d %d\n", &in_pdsu_size, &in_sym, &in_pad, &in_encoded_bits, &in_data_bits))
      ;
    DEBUG(printf("  FRAME: %d %d %d %d %d\n", in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits));
    the_viterbi_trace_dict[i].frame_p.psdu_size      = in_pdsu_size;
    the_viterbi_trace_dict[i].frame_p.n_sym          = in_sym;
    the_viterbi_trace_dict[i].frame_p.n_pad          = in_pad;
    the_viterbi_trace_dict[i].frame_p.n_encoded_bits = in_encoded_bits;
    the_viterbi_trace_dict[i].frame_p.n_data_bits    = in_data_bits;

    int num_in_bits = in_encoded_bits + 10; // strlen(str3)+10; //additional 10 values
    DEBUG(printf("  Reading %u in_bits\n", num_in_bits));
    for (int ci = 0; ci < num_in_bits; ci++) { 
      unsigned c;
      if (fscanf(dictF, "%u ", &c))
	;
      #ifdef SUPER_VERBOSE
      printf("%u ", c);
      #endif
      the_viterbi_trace_dict[i].in_bits[ci] = (uint8_t)c;
    }
    DEBUG(printf("\n"));
  }
  fclose(dictF);

  //Clear the messages (injected) histogram
  for (int i = 0; i < VITERBI_MSG_LENGTHS; i++) {
    for (int j = 0; j < NUM_MESSAGES; j++) {
      viterbi_messages_histogram[i][j] = 0;
    }
  }

  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    hist_total_objs[i] = 0;
  }

#ifdef HW_VIT
  init_vit_parameters();
  printf("Open Vit-Do-Decode device %s\n", VIT_DEVNAME);
  vitHW_fd = open(VIT_DEVNAME, O_RDWR, 0);
  if(vitHW_fd < 0) {
	  fprintf(stderr, "Error: cannot open %s", VIT_DEVNAME);
	  exit(EXIT_FAILURE);
  }

  vitHW_lmem = contig_alloc(vitHW_size, &vitHW_mem);
  if (vitHW_lmem == NULL) {
    fprintf(stderr, "Error: cannot allocate %zu contig bytes", vitHW_size);
    exit(EXIT_FAILURE);
  }
  vitHW_li_mem = &(vitHW_lmem[0]);
  vitHW_lo_mem = &(vitHW_lmem[vitHW_out_offset]);
  printf("Set vitHW_li_mem = %p  AND vitHW_lo_mem = %p\n", vitHW_li_mem, vitHW_lo_mem);

  vitHW_desc.esp.run = true;
  vitHW_desc.esp.coherence = ACC_COH_NONE;
  vitHW_desc.esp.p2p_store = 0;
  vitHW_desc.esp.p2p_nsrcs = 0;
  vitHW_desc.esp.contig = contig_to_khandle(vitHW_mem);

#endif

  DEBUG(printf("DONE with init_vit_kernel -- returning success\n"));
  return success;
}

status_t init_cv_kernel(char* py_file, char* dict_fn)
{
  DEBUG(printf("In the init_cv_kernel routine\n"));
  /** The CV kernel uses a different method to select appropriate inputs; dictionary not needed
  // Read in the object images dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF)
  {
    printf("Error: unable to open dictionary file %s\n", dict_fn);
    return error;
  }
  // Read the number of definitions
  fscanf(dictF, "%u\n", &num_cv_dictionary_items);
  DEBUG(printf("  There are %u dictionary entries\n", num_cv_dictionary_items));
  the_cv_object_dict = (cv_dict_entry_t*)calloc(num_cv_dictionary_items, sizeof(cv_dict_entry_t));
  if (the_cv_object_dict == NULL) 
  {
    printf("ERROR : Cannot allocate Cv Trace Dictionary memory space");
    return error;
  }

  for (int di = 0; di < num_cv_dictionary_items; di++) {
    unsigned entry_id;
    unsigned object_id;
    fscanf(dictF, "%u %u", &entry_id, &object_id);
    DEBUG(printf("  Reading cv dictionary entry %u : %u %u\n", di, entry_id, object_id));
    the_cv_object_dict[di].image_id = entry_id;
    the_cv_object_dict[di].object   = object_id;
    for (int i = 0; i < IMAGE_SIZE; i++) {
      unsigned fin;
      fscanf(dictF, "%u", &fin);
      the_cv_object_dict[di].image_data[i] = fin;
    }
  }
  fclose(dictF);
  **/
  // Initialization to run Keras CNN code
  initNVDLA(); 
#ifndef BYPASS_KERAS_CV_CODE
  Py_Initialize();
  pName = PyUnicode_DecodeFSDefault(python_module);
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);

  if (pModule == NULL) {
     PyErr_Print();
     printf("Failed to load Python program, perhaps pythonpath needs to be set; export PYTHONPATH=your_mini_era_dir/cv/CNN_MIO_KERAS");
     return 1;
  } else {
    pFunc_load = PyObject_GetAttrString(pModule, python_func_load);

    if (pFunc_load && PyCallable_Check(pFunc_load)) {
       PyObject_CallObject(pFunc_load, NULL);
    }
    else {
        if (PyErr_Occurred())
        PyErr_Print();
        printf("Cannot find python function - loadmodel");
    }
    Py_XDECREF(pFunc_load);
  }
  DEBUG(printf("CV Kernel Init done\n"));
#endif  
  return success;
}




label_t run_object_classification_syscall(unsigned tr_val) 
{
  DEBUG(printf("Entered run_object_classification...\n"));
  label_t object;	
#ifdef BYPASS_KERAS_CV_CODE
  object = (label_t)tr_val;
#else
  char shell_cmd[100];
  snprintf(shell_cmd, sizeof(shell_cmd), "sh utils/cnn_shell.sh %u", tr_val);
  DEBUG(printf("  Invoking CV CNN using `%s`\n", shell_cmd));
  FILE *testing = popen(shell_cmd, "r");
  if (testing == NULL)
  {
    printf("FAIL to open CV kernel !\n");
    return 1;
  }
  char pbuffer[100];
  while (fgets(pbuffer, 100, testing) != NULL)
  {
    //printf(pbuffer);
  }
  DEBUG(printf("Label Prediction done \n"));
  DEBUG(printf("pbuffer : %s\n", pbuffer));
  int val = atoi(pbuffer);   //the last thing printed by the Keras code is the predicted label 
  object = (label_t)val;
  pclose(testing);
  DEBUG(printf("run_object_classification returning %u = %u\n", val, object));
#endif
  return object;  
}

label_t run_object_classification(unsigned tr_val) 
{
  DEBUG(printf("Entered run_object_classification... tr_val = %u\n", tr_val));
  label_t object = (label_t)tr_val;
#ifndef BYPASS_KERAS_CV_CODE
  if (pModule != NULL) {
    DEBUG(printf("  Starting call to pModule...\n"));
    pFunc = PyObject_GetAttrString(pModule, python_func);
  
    if (pFunc && PyCallable_Check(pFunc)) {
      pArgs = PyTuple_New(1);
      pValue = PyLong_FromLong(tr_val);
      if (!pValue) {
	Py_DECREF(pArgs);
	Py_DECREF(pFunc);
	Py_DECREF(pModule);
	fprintf(stderr, "Trying to run CNN kernel: Cannot convert C argument into python\n");
	return 1;
      }
      PyTuple_SetItem(pArgs, 0, pValue);
      pretValue = PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      if (pretValue != NULL) {
	DEBUG(printf("Predicted label from Python program: %ld\n", PyLong_AsLong(pretValue)));
	int val = PyLong_AsLong(pretValue);    
	object = (label_t)val;
	DEBUG(printf("run_object_classification returning %u = %u\n", val, object));
	Py_DECREF(pretValue);
      }
      else {
	Py_DECREF(pFunc);
	Py_DECREF(pModule);
	PyErr_Print();
	printf("Trying to run CNN kernel : Python function call failed\n");
	return 1;
      }
    }
    else {
      if (PyErr_Occurred())
	PyErr_Print();
      printf("Cannot find python function");
    }
    Py_XDECREF(pFunc);
    //Py_DECREF(pModule);
  }
#endif
  return object;  
}


label_t iterate_cv_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_cv_kernel\n"));

  unsigned tr_val = 0; // Default nothing
  switch(nearest_obj[vs.lane]) {
    case 'N' : tr_val = no_label; break;
    case 'B' : tr_val = bicycle; break;
    case 'C' : tr_val = car; break;
    case 'P' : tr_val = pedestrian; break;
    case 'T' : tr_val = truck; break;
    default: printf("ERROR : Unknown object type in cv trace: '%c'\n", nearest_obj[vs.lane]); exit(-2);
  }
  label_t d_object = (label_t)tr_val;

  return d_object;
}


label_t execute_cv_kernel(label_t in_tr_val)
{
  /* 2) Conduct object detection on the image frame */
  DEBUG(printf("  Calling run_object_detection with in_tr_val tr_val %u %s\n", in_tr_val, object_names[in_tr_val]));
  // Call Keras Code
  //label_t object = run_object_classification((unsigned)in_tr_val); 
  label_t object = in_tr_val;

  //runImageonNVDLA("seven.pgm");
  runImageonNVDLA("class_busimage_5489.jpg");

  DEBUG(printf("  Returning object %u %s : tr_val %u %s\n", object, object_names[object], in_tr_val, object_names[in_tr_val]));
  return object;
}

void post_execute_cv_kernel(label_t tr_val, label_t cv_object)
{
  if (cv_object == tr_val) {
    label_match[cv_object]++;
    label_match[NUM_OBJECTS]++;
  } else {
    label_mismatch[tr_val][cv_object]++;
  }
  label_lookup[NUM_OBJECTS]++;
  label_lookup[cv_object]++;
}



radar_dict_entry_t* iterate_rad_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_rad_kernel\n"));

  unsigned tr_val = nearest_dist[vs.lane] / RADAR_BUCKET_DISTANCE;  // The proper message for this time step and car-lane

  return &(the_radar_return_dict[tr_val]);
}


distance_t execute_rad_kernel(float * inputs)
{
  DEBUG(printf("In execute_rad_kernel\n"));

  /* 2) Conduct distance estimation on the waveform */
  DEBUG(printf("  Calling calculate_peak_dist_from_fmcw\n"));
  distance_t dist = calculate_peak_dist_from_fmcw(inputs);
  DEBUG(printf("  Returning distance = %.1f\n", dist));

  return dist;
}


void post_execute_rad_kernel(unsigned index, distance_t tr_dist, distance_t dist)
{
  // Get an error estimate (Root-Squared?)
  float error;
  radar_total_calc++;
  hist_distances[index]++;
  if ((tr_dist >= 500.0) && (dist > 10000.0)) {
    error = 0.0;
  } else {
    error = (tr_dist - dist);
  }
  float abs_err = fabs(error);
  float pct_err;
  if (tr_dist != 0.0) {
    pct_err = abs_err/tr_dist;
  } else {
    pct_err = abs_err;
  }
  
  DEBUG(printf("%f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", tr_dist, dist, error, abs_err, pct_err));
  //printf("IDX: %u :: %f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", index, tr_dist, dist, error, abs_err, pct_err);
  if (pct_err == 0.0) {
    hist_pct_errs[index][0]++;
  } else if (pct_err < 0.01) {
    //printf("RADAR_LT001_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err);
    hist_pct_errs[index][1]++;
  } else if (pct_err < 0.1) {
    //printf("RADAR_LT010_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err);
    hist_pct_errs[index][2]++;
  } else if (pct_err < 1.00) {
    //printf("RADAR_LT100_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err);
    hist_pct_errs[index][3]++;
  } else {
    //printf("RADAR_GT100_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err);
    hist_pct_errs[index][4]++;
  }
}


/* Each time-step of the trace, we read in the 
 * trace values for the left, middle and right lanes
 * (i.e. which message if the autonomous car is in the 
 *  left, middle or right lane).
 */
vit_dict_entry_t* iterate_vit_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_vit_kernel in lane %u = %s\n", vs.lane, lane_names[vs.lane]));
  hist_total_objs[total_obj]++;
  unsigned tr_val = 0; // set a default to avoid compiler messages
  switch (vs.lane) {
  case lhazard:
    {
      unsigned nd_1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      DEBUG(printf("  Lane %u : obj in %u is %c at %u\n", vs.lane, vs.lane+1, nearest_obj[vs.lane+1], nd_1));
      if ((nearest_obj[1] != 'N') && (nd_1 < VIT_CLEAR_THRESHOLD)) {  
	// Some object is in the left lane within threshold distance
	tr_val = 3; // Unsafe to move from lhazard lane into the left lane 
      } else {
	tr_val = 1;
      }
    }
    break;
  case left:
  case center:
  case right:
    {
      unsigned ndp1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      unsigned ndm1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      tr_val = 0;
      DEBUG(printf("  Lane %u : obj in %u is %c at %.1f : obj in %u is %c at %.1f\n", vs.lane, 
		   vs.lane-1, nearest_obj[vs.lane-1], nearest_dist[vs.lane-1],
		   vs.lane+1, nearest_obj[vs.lane+1], nearest_dist[vs.lane+1]));
      if ((nearest_obj[vs.lane-1] != 'N') && (ndm1 < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the Left lane at distance 0 or 1
	DEBUG(printf("    Marking unsafe to move left\n"));
	tr_val += 1; // Unsafe to move from this lane to the left.
      }
      if ((nearest_obj[vs.lane+1] != 'N') && (ndp1 < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the Right lane at distance 0 or 1
	DEBUG(printf("    Marking unsafe to move right\n"));
	tr_val += 2; // Unsafe to move from this lane to the right.
      }
    }
    break;
  case rhazard:
    {
      unsigned nd_3 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[3] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      DEBUG(printf("  Lane %u : obj in %u is %c at %u\n", vs.lane, vs.lane-1, nearest_obj[vs.lane-1], nd_3));
      if ((nearest_obj[3] != 'N') && (nd_3 < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the right lane within threshold distance
	tr_val = 3; // Unsafe to move from center lane to the right.
      } else {
	tr_val = 2;
      }
    }
    break;
  }

  DEBUG(printf("Viterbi final message for lane %u %s = %u\n", vs.lane, lane_names[vs.lane], tr_val));	

  vit_dict_entry_t* trace_msg; // Will hold msg input data for decode, based on trace input

  // Here we determine short or long messages, based on global vit_msgs_size; offset is into the Dictionary
  int msg_offset = vit_msgs_size * NUM_MESSAGES; // 0 = short messages, 4 = long messages

  switch(tr_val) {
  case 0: // safe_to_move_right_or_left
    trace_msg = &(the_viterbi_trace_dict[0 + msg_offset]);
    break;
  case 1: // safe_to_move_right
    trace_msg = &(the_viterbi_trace_dict[1 + msg_offset]);
    break;
  case 2: // safe_to_move_left
    trace_msg = &(the_viterbi_trace_dict[2 + msg_offset]);
    break;
  case 3: // unsafe_to_move_left_or_right
    trace_msg = &(the_viterbi_trace_dict[3 + msg_offset]);
    break;
  }
  DEBUG(printf(" VIT: Using msg %u Id %u : %s \n", trace_msg->msg_num, trace_msg->msg_id, message_names[trace_msg->msg_id]));
  return trace_msg;
}

message_t execute_vit_kernel(vit_dict_entry_t* trace_msg, int num_msgs)
{
  // Send each message (here they are all the same) through the viterbi decoder
  message_t msg = num_message_t;
  uint8_t *result;
  char     msg_text[1600]; // Big enough to hold largest message (1500?)
  for (int mi = 0; mi < num_msgs; mi++) {
    DEBUG(printf("  Calling the viterbi decode routine for message %u iter %u\n", trace_msg->msg_num, mi));
    viterbi_messages_histogram[vit_msgs_size][trace_msg->msg_id]++; 
    int n_res_char;
    result = decode(&(trace_msg->ofdm_p), &(trace_msg->frame_p), &(trace_msg->in_bits[0]), &n_res_char);
    // descramble the output - put it in result
    int psdusize = trace_msg->frame_p.psdu_size;
    DEBUG(printf("  Calling the viterbi descrambler routine\n"));
    descrambler(result, psdusize, msg_text, NULL /*descram_ref*/, NULL /*msg*/);

   #if(0)
    printf(" PSDU %u : Msg : = `", psdusize);
    for (int ci = 0; ci < (psdusize - 26); ci++) {
      printf("%c", msg_text[ci]);
    }
    printf("'\n");
   #endif
    if (mi == 0) { 
      // Here we look at the message string and select proper message_t out (just for the first message)
      switch(msg_text[3]) {
      case '0' : msg = safe_to_move_right_or_left; break;
      case '1' : msg = safe_to_move_right_only; break;
      case '2' : msg = safe_to_move_left_only; break;
      case '3' : msg = unsafe_to_move_left_or_right; break;
      default  : msg = num_message_t; break;
      }
    } // if (mi == 0)
  }
  DEBUG(printf("The execute_vit_kernel is returning msg %u\n", msg));
    
  return msg;
}

void post_execute_vit_kernel(message_t tr_msg, message_t dec_msg)
{
  total_msgs++;
  if (dec_msg != tr_msg) {
    bad_decode_msgs++;
  }
}


/* #undef DEBUG */
/* #define DEBUG(x) x */

vehicle_state_t plan_and_control(label_t label, distance_t distance, message_t message, vehicle_state_t vehicle_state)
{
  DEBUG(printf("In the plan_and_control routine : label %u %s distance %.1f (T1 %.1f T1 %.1f T3 %.1f) message %u\n", 
	       label, object_names[label], distance, THRESHOLD_1, THRESHOLD_2, THRESHOLD_3, message));
  vehicle_state_t new_vehicle_state = vehicle_state;
  if (!vehicle_state.active) {
    // Our car is broken and burning, no plan-and-control possible.
    return vehicle_state;
  }
  
  if (//(label != no_label) && // For safety, assume every return is from SOMETHING we should not hit!
      ((distance <= THRESHOLD_1)
       #ifdef USE_SIM_ENVIRON
       || ((vehicle_state.speed < car_goal_speed) && (distance <= THRESHOLD_2))
       #endif
       )) {
    if (distance <= IMPACT_DISTANCE) {
      printf("WHOOPS: We've suffered a collision on time_step %u!\n", time_step);
      //fprintf(stderr, "WHOOPS: We've suffered a collision on time_step %u!\n", time_step);
      new_vehicle_state.speed = 0.0;
      new_vehicle_state.active = false; // We should add visualizer stuff for this!
      return new_vehicle_state;
    }
    
    // Some object ahead of us that needs to be avoided.
    DEBUG(printf("  In lane %s with %c (%u) at %.1f (trace: %.1f)\n", lane_names[vehicle_state.lane], nearest_obj[vehicle_state.lane], label, distance, nearest_dist[vehicle_state.lane]));
    switch (message) {
      case safe_to_move_right_or_left   :
	/* Bias is move right, UNLESS we are in the Right lane and would then head into the RHazard Lane */
	if (vehicle_state.lane < right) { 
	  DEBUG(printf("   In %s with Safe_L_or_R : Moving Right\n", lane_names[vehicle_state.lane]));
	  //new_vehicle_state.lane += 1; //NC 
	  new_vehicle_state.lane = static_cast<lane_t>(static_cast<int>(new_vehicle_state.lane) + 1); //NC 
	} else {
	  DEBUG(printf("   In %s with Safe_L_or_R : Moving Left\n", lane_names[vehicle_state.lane]));
	  //new_vehicle_state.lane -= 1;
	  new_vehicle_state.lane = static_cast<lane_t>(static_cast<int>(new_vehicle_state.lane) - 1); //NC 
	}	  
	break; // prefer right lane
      case safe_to_move_right_only      :
	DEBUG(printf("   In %s with Safe_R_only : Moving Right\n", lane_names[vehicle_state.lane]));
	//new_vehicle_state.lane += 1;
	 new_vehicle_state.lane = static_cast<lane_t>(static_cast<int>(new_vehicle_state.lane) + 1); //NC 
	break;
      case safe_to_move_left_only       :
	DEBUG(printf("   In %s with Safe_L_Only : Moving Left\n", lane_names[vehicle_state.lane]));
	//new_vehicle_state.lane -= 1;
	new_vehicle_state.lane = static_cast<lane_t>(static_cast<int>(new_vehicle_state.lane) - 1); //NC 
	break;
      case unsafe_to_move_left_or_right :
	#ifdef USE_SIM_ENVIRON
	if (vehicle_state.speed > car_decel_rate) {
	  new_vehicle_state.speed = vehicle_state.speed - car_decel_rate; // was / 2.0;
	  DEBUG(printf("   In %s with No_Safe_Move -- SLOWING DOWN from %.2f to %.2f\n", lane_names[vehicle_state.lane], vehicle_state.speed, new_vehicle_state.speed));
	} else {
	  DEBUG(printf("   In %s with No_Safe_Move -- Going < 15.0 so STOPPING!\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.speed = 0.0;
	}
	#else
	DEBUG(printf("   In %s with No_Safe_Move : STOPPING\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.speed = 0.0;
	#endif
	break; /* Stop!!! */
    default:
      printf(" ERROR  In %s with UNDEFINED MESSAGE: %u\n", lane_names[vehicle_state.lane], message);
      //exit(-6);
    }
  } else {
    // No obstacle-inspired lane change, so try now to occupy the center lane
    switch (vehicle_state.lane) {
    case lhazard:
    case left:
      if ((message == safe_to_move_right_or_left) ||
	  (message == safe_to_move_right_only)) {
	DEBUG(printf("  In %s with Can_move_Right: Moving Right\n", lane_names[vehicle_state.lane]));
	//new_vehicle_state.lane += 1;
	new_vehicle_state.lane = static_cast<lane_t>(static_cast<int>(new_vehicle_state.lane) + 1); //NC 
      }
      break;
    case center:
      // No need to alter, already in the center
      break;
    case right:
    case rhazard:
      if ((message == safe_to_move_right_or_left) ||
	  (message == safe_to_move_left_only)) {
	DEBUG(printf("  In %s with Can_move_Left : Moving Left\n", lane_names[vehicle_state.lane]));
	//new_vehicle_state.lane -= 1;
	new_vehicle_state.lane = static_cast<lane_t>(static_cast<int>(new_vehicle_state.lane) - 1); //NC 
      }
      break;
    }
    #ifdef USE_SIM_ENVIRON
    if ((vehicle_state.speed < car_goal_speed) &&  // We are going slower than we want to, and
	//((label == no_label) ||      // There is no object ahead of us -- don't need; NOTHING is at INF_DISTANCE
	(distance >= THRESHOLD_2)) { // Any object is far enough away 
      if (vehicle_state.speed <= (car_goal_speed - car_accel_rate)) {
	new_vehicle_state.speed += 15.0;
      } else {
	new_vehicle_state.speed = car_goal_speed;
      }
      DEBUG(printf("  Going %.2f : slower than target speed %.2f : Speeding up to %.2f\n", vehicle_state.speed, 50.0, new_vehicle_state.speed));
    }
    #endif
  } // else clause


  return new_vehicle_state;
}
/* #undef DEBUG */
/* #define DEBUG(x) */


void closeout_cv_kernel()
{
  float label_correct_pctg = (100.0*label_match[NUM_OBJECTS])/(1.0*label_lookup[NUM_OBJECTS]);
  printf("\nFinal CV CNN Accuracy: %u correct of %u classifications = %.2f%%\n", label_match[NUM_OBJECTS], label_lookup[NUM_OBJECTS], label_correct_pctg);
  for (int i = 0; i < NUM_OBJECTS; i++) {
    label_correct_pctg = (100.0*label_match[i])/(1.0*label_lookup[i]);
    printf("  CV CNN Accuracy for %10s : %u correct of %u classifications = %.2f%%\n", object_names[i], label_match[i], label_lookup[i], label_correct_pctg);
  }

  unsigned errs = label_lookup[NUM_OBJECTS] - label_match[NUM_OBJECTS];
  if (errs > 0) {
    printf("\nAnalysis of the %u mis-identifications:\n", errs);
    for (int i = 0; i < NUM_OBJECTS; i++) {
      for (int j = 0; j < NUM_OBJECTS; j++) {
	if (label_mismatch[i][j] != 0) {
	  printf("  Mislabeled %10s as %10s on %u occasions\n", object_names[i], object_names[j], label_mismatch[i][j]);
	}
      }
    }
  }

#ifndef BYPASS_KERAS_CV_CODE
    Py_DECREF(pModule);
    Py_Finalize();
#endif   
}

void closeout_rad_kernel()
{
  printf("\nHistogram of Radar Distances:\n");
  for (int di = 0; di < num_radar_dictionary_items; di++) {
    printf("    %3u | %8.3f | %9u \n", di, the_radar_return_dict[di].distance, hist_distances[di]);
  }

  printf("\nHistogram of Radar Distance ABS-PCT-ERROR:\n");
  unsigned totals[] = {0, 0, 0, 0, 0};
  
  for (int di = 0; di < num_radar_dictionary_items; di++) {
    printf("    Entry %u Id %u Distance %f Occurs %u Histogram:\n", di, the_radar_return_dict[di].index, the_radar_return_dict[di].distance, hist_distances[di]);
    for (int i = 0; i < 5; i++) {
      printf("    %7s | %9u \n", hist_pct_err_label[i], hist_pct_errs[di][i]);
      totals[i] += hist_pct_errs[di][i];
    }
  }

  printf("\n  TOTALS Histogram of Radar Distance ABS-PCT-ERROR:\n");
  for (int i = 0; i < 5; i++) {
    printf("  %7s | %9u \n", hist_pct_err_label[i], totals[i]);
  }
}

void closeout_vit_kernel()
{
  // Nothing to do?

  printf("\nHistogram of Total Objects:\n");
  unsigned sum = 0;
  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    if (hist_total_objs[i] != 0) {
      printf("%3u | %9u \n", i, hist_total_objs[i]);
      sum += i*hist_total_objs[i];
    }
  }
  double avg_objs = (1.0 * sum)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  printf("There were %.3lf obstacles per time step (average)\n", avg_objs);
  double avg_msgs = (1.0 * total_msgs)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  printf("There were %.3lf messages per time step (average)\n", avg_msgs);
  printf("There were %u bad decodes of the %u messages\n", bad_decode_msgs, total_msgs);

  printf("\nHistogram of Viterbi Messages:\n");
  printf("    %3s | %3s | %9s \n", "Len", "Msg", "NumOccurs");
  for (int li = 0; li < VITERBI_MSG_LENGTHS; li++) {
    for (int mi = 0; mi < NUM_MESSAGES; mi++) {
      printf("    %3u | %3u | %9u \n", li, mi, viterbi_messages_histogram[li][mi]);
    }
  }
  printf("\n");

#ifdef HW_VIT
  contig_free(vitHW_mem);
  close(vitHW_fd);
#endif

#ifdef HW_FFT
  contig_free(fftHW_mem);
  close(fftHW_fd);
#endif

}


