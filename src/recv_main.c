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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>
#include <getopt.h>

#include "globals.h"
#include "getopt.h"
#include "debug.h"

#include "recv_pipe.h"  // IEEE 802.11p WiFi SDR Transmit Pipeline

#define TIME

#ifdef TIME
struct timeval stop_prog, start_prog;
struct timeval stop_exec_recv, start_exec_recv;
uint64_t exec_recv_sec = 0LL;
uint64_t exec_recv_usec = 0LL;
#endif // TIME


unsigned max_time_steps = 1;
unsigned time_step = 0;

#define MAX_MESSAGE_LEN     1500   // Max chars in a message (payload)
#define MAX_XMIT_OUTPUTS   41800   // Really 41782 I think

char recv_in_fname[256] = "default_recv_msg";

// Taken from the input file
uint32_t xmit_msg_len = 0;
int      xmit_num_out;
float    xmit_out_real[MAX_XMIT_OUTPUTS];
float    xmit_out_imag[MAX_XMIT_OUTPUTS];

bool show_main_output = true;
bool do_add_pre_pad = false;
bool show_recv_output = true;

// Forward Declarations
void dump_final_run_statistics();
void INThandler(int dummy);

#ifdef HW_VIT
 extern void init_VIT_HW_ACCEL();
 extern void free_VIT_HW_RESOURCES();
#endif

#ifdef RECV_HW_FFT
extern void free_RECV_FFT_HW_RESOURCES();
#endif


void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -s <N>     : Run the simulation for <N> time steps\n");
  printf("    -f <FN>    : Use file <FN> as input encoded message source\n");
  printf("    -M <0|1>   : 0=disable 1=enable output of Messages (input and output) per time step\n");
  printf("    -r <0|1>   : 0=disable 1=enable output of RECV output per time step\n");
}

void INThandler(int dummy)
{
  printf("In SIGINT INThandler -- Closing the connection and exiting\n");
  closeout_and_exit("Received a SIGINT...", -1);
}


void SIGPIPE_handler(int dummy)
{
  printf("In SIGPIPE_handler -- Closing the connection and exiting\n");
  closeout_and_exit("Received a SIGPIPE...", -1);
}


// This cleans up the state before exit
void closeout_and_exit(char* last_msg, int rval)
{
  if (time_step > 0) {
    dump_final_run_statistics();
  }
 #ifdef HW_VIT
  free_VIT_HW_RESOURCES();
 #endif // HW_VIT
 #ifdef RECV_HW_FFT
  free_RECV_FFT_HW_RESOURCES();
 #endif
  printf("%s\n", last_msg);
  exit(rval);
}

int main(int argc, char *argv[])
{
  int opt;

 #ifdef HW_VIT
  init_VIT_HW_ACCEL();
 #endif

  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  while((opt = getopt(argc, argv, ":hs:f:M:r:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 's':
      max_time_steps = atoi(optarg);
      //printf("Using %u maximum time steps\n", max_time_steps);
      break;
    case 'M':
      show_main_output = (atoi(optarg) != 0);
      break;
    case 'r':
      show_recv_output = (atoi(optarg) != 0);
      break;
    case 'f':
      snprintf(recv_in_fname, 255, "%s", optarg);
      break;
    case ':':
      printf("option needs a value\n");
      break;
    case '?':
      printf("unknown option: %c\n", optopt);
    break;
    }
  }

  // optind is for the extra arguments
  // which are not parsed
  for(; optind < argc; optind++){
    printf("extra arguments: %s\n", argv[optind]);
  }
  printf("Using the ESP_INTERFACE\n");
 #ifdef HW_VIT
  printf("Using the Viterbi Hardware Accelerator\n");
 #endif
  printf("Set show_recv_output = %u\n", show_recv_output);
  printf("Running for %u time steps\n", max_time_steps);
  printf("RECV message is taken from file %s\n", recv_in_fname);

  // Read in the encoded message data
  FILE *inF = fopen(recv_in_fname, "r");
  if (!inF) {
    printf("Error: unable to open receiver-pipeline input encoded message file %s\n", recv_in_fname);
    exit(-1);
  }

  // Read in the encoded message data
  // Read the length and number of encoded complex values
  if (fscanf(inF, "%u %d\n", &xmit_msg_len, &xmit_num_out) != 2) {
    printf("ERROR reading the encoded msg length and number of complex values\n");
    fclose(inF);
    exit(-2);
  }
  DEBUG(printf("  The message is %u bytes and %u complex encoded values\n", xmit_msg_len, xmit_num_out));
  for (int i = 0; i < xmit_num_out; i++) {
    if (fscanf(inF, "%f %f\n", &xmit_out_real[i], &xmit_out_imag[i]) != 2) {
      printf("ERROR reading the complex input %d values\n", i);
      fclose(inF);
      exit(-2);
    }
  }
  fclose(inF);

  /* Kernels initialization */
  printf("Initializing the Receive pipeline...\n");
  recv_pipe_init();
  
  /*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
  printf("Starting the main loop...\n");
  /* The input trace contains the per-epoch (time-step) input data */
 #ifdef TIME
  gettimeofday(&start_prog, NULL);
 #endif
  for (time_step = 0; time_step < max_time_steps; time_step++) {
    DEBUG(printf("Time Step %u\n", time_step));

    /* The receive pipeline does a receive of one message */
    int  recv_msg_len;
    char recv_msg[MAX_MESSAGE_LEN];
   #ifdef TIME
    gettimeofday(&start_exec_recv, NULL);
   #endif
    do_recv_pipeline(xmit_num_out, xmit_out_real, xmit_out_imag, &recv_msg_len, recv_msg);    
   #ifdef TIME
    gettimeofday(&stop_exec_recv, NULL);
    exec_recv_sec  += stop_exec_recv.tv_sec  - start_exec_recv.tv_sec;
    exec_recv_usec += stop_exec_recv.tv_usec - start_exec_recv.tv_usec;
   #endif

    if (show_main_output) {
      printf("Iteration %u : RECV_MSG:\n'%s'\n", time_step, recv_msg);
    }
  }

  dump_final_run_statistics();

 #ifdef RECV_HW_FFT
  free_RECV_FFT_HW_RESOURCES();
 #endif

  printf("\nDone.\n");
  return 0;
}



void dump_final_run_statistics()
{
  printf("\nFinal Run Stats after, %u, Time Steps\n", time_step);
  printf("Occ-Map Dimensions, %u, by, %u, grid, res, %lf, ray_r, %u\n", GRID_MAP_X_DIM, GRID_MAP_Y_DIM, GRID_MAP_RESLTN, RAYTR_RANGE);

  printf("Timing (in usec):");
 #ifdef HW_VIT
  printf(" with %u HW_VIT", 1);
 #else
  printf(" with NO HW_VIT");
 #endif
 #ifdef XMIT_HW_FFT
  printf(" and %u HW_XMIT_FFT", NUM_XMIT_FFT_ACCEL);
 #else
  printf(" and NO HW_XMIT_FFT");
 #endif
 #ifdef RECV_HW_FFT
  printf(" and %u HW_RECV_FFT", NUM_RECV_FFT_ACCEL);
 #else
  printf(" and NO HW_RECV_FFT");
 #endif
  printf("\n");

  gettimeofday(&stop_prog, NULL);
 #ifdef TIME
  uint64_t total_exec = (uint64_t) (stop_prog.tv_sec - start_prog.tv_sec) * 1000000 + (uint64_t) (stop_prog.tv_usec - start_prog.tv_usec);
  uint64_t total_recv   = (uint64_t) (exec_recv_sec) * 1000000 + (uint64_t) (exec_recv_usec);

 #ifdef INT_TIME
  // This is the recv_pipe.c breakdown
  uint64_t r_pipe     = (uint64_t)(r_pipe_sec)  * 1000000 + (uint64_t)(r_pipe_usec);
  uint64_t r_cmpcnj   = (uint64_t)(r_cmpcnj_sec)  * 1000000 + (uint64_t)(r_cmpcnj_usec);
  uint64_t r_cmpmpy   = (uint64_t)(r_cmpmpy_sec)  * 1000000 + (uint64_t)(r_cmpmpy_usec);
  uint64_t r_firc     = (uint64_t)(r_firc_sec)  * 1000000 + (uint64_t)(r_firc_usec);
  uint64_t r_cmpmag   = (uint64_t)(r_cmpmag_sec)  * 1000000 + (uint64_t)(r_cmpmag_usec);
  uint64_t r_cmpmag2  = (uint64_t)(r_cmpmag2_sec)  * 1000000 + (uint64_t)(r_cmpmag2_usec);
  uint64_t r_fir      = (uint64_t)(r_fir_sec)  * 1000000 + (uint64_t)(r_fir_usec);
  uint64_t r_div      = (uint64_t)(r_div_sec)  * 1000000 + (uint64_t)(r_div_usec);
  uint64_t r_sshort   = (uint64_t)(r_sshort_sec)  * 1000000 + (uint64_t)(r_sshort_usec);
  uint64_t r_slong    = (uint64_t)(r_slong_sec)  * 1000000 + (uint64_t)(r_slong_usec);
  uint64_t r_fft      = (uint64_t)(r_fft_sec)  * 1000000 + (uint64_t)(r_fft_usec);
  uint64_t r_eqlz     = (uint64_t)(r_eqlz_sec)  * 1000000 + (uint64_t)(r_eqlz_usec);
  uint64_t r_decsignl = (uint64_t)(r_decsignl_sec)  * 1000000 + (uint64_t)(r_decsignl_usec);
  uint64_t r_descrmbl = (uint64_t)(r_descrmbl_sec)  * 1000000 + (uint64_t)(r_descrmbl_usec);

  // This is the receiver Hardware FFT breakdown
  #ifdef RECV_HW_FFT
  uint64_t r_fHtotal   = (uint64_t)(r_fHtotal_sec)  * 1000000 + (uint64_t)(r_fHtotal_usec);
  uint64_t r_fHcvtin   = (uint64_t)(r_fHcvtin_sec)  * 1000000 + (uint64_t)(r_fHcvtin_usec);
  uint64_t r_fHcomp    = (uint64_t)(r_fHcomp_sec)  * 1000000 + (uint64_t)(r_fHcomp_usec);
  uint64_t r_fHcvtout  = (uint64_t)(r_fHcvtout_sec)  * 1000000 + (uint64_t)(r_fHcvtout_usec);
  #endif // RECV_HW_FFT

  // This is the sync_short.c "equalize" breakdown
  uint64_t rssh_total    = (uint64_t)(sysh_total_sec)     * 1000000 + (uint64_t)(sysh_total_usec);
  uint64_t rssh_search   = (uint64_t)(sysh_search_sec)    * 1000000 + (uint64_t)(sysh_search_usec);
  uint64_t rssh_frame    = (uint64_t)(sysh_frame_sec)     * 1000000 + (uint64_t)(sysh_frame_usec);

  // This is the synch_long.c "equalize" breakdown
  uint64_t rslg_total    = (uint64_t)(sylg_total_sec)    * 1000000 + (uint64_t)(sylg_total_usec);
  uint64_t rslg_firG     = (uint64_t)(sylg_firG_sec)     * 1000000 + (uint64_t)(sylg_firG_usec);
  uint64_t rslg_search   = (uint64_t)(sylg_search_sec)   * 1000000 + (uint64_t)(sylg_search_usec);
  uint64_t rslg_outgen   = (uint64_t)(sylg_outgen_sec)   * 1000000 + (uint64_t)(sylg_outgen_usec);

  // This is the gr_equalizer.c "equalize" breakdown
  uint64_t reql_total    = (uint64_t)(reql_total_sec)     * 1000000 + (uint64_t)(reql_total_usec);
  uint64_t reql_sym_set  = (uint64_t)(reql_symset_sec)    * 1000000 + (uint64_t)(reql_symset_usec);
  uint64_t reql_ls_eql   = (uint64_t)(reql_lseq_call_sec) * 1000000 + (uint64_t)(reql_lseq_call_usec);
  uint64_t reql_out_sym  = (uint64_t)(reql_outsym_sec)    * 1000000 + (uint64_t)(reql_outsym_usec);
  uint64_t reql_ds_fld   = (uint64_t)(reql_decSF_sec)     * 1000000 + (uint64_t)(reql_decSF_usec);

  // This is the ofdm.c decode-signal breakdown
  uint64_t rdec_total    = (uint64_t)(rdec_total_sec)  * 1000000 + (uint64_t)(rdec_total_usec);
  uint64_t rdec_map_bitr = (uint64_t)(rdec_map_bitr_sec)  * 1000000 + (uint64_t)(rdec_map_bitr_usec);
  uint64_t rdec_get_bits = (uint64_t)(rdec_get_bits_sec)  * 1000000 + (uint64_t)(rdec_get_bits_usec);
  uint64_t rdec_dec_call = (uint64_t)(rdec_dec_call_sec)  * 1000000 + (uint64_t)(rdec_dec_call_usec);
 #endif // INT_TIME

  printf(" Total workload main-loop : %10lu usec\n", total_exec);
  printf("   Total recv_pipe run-time : %10lu usec\n", total_recv);
 #ifdef INT_TIME
  printf("     R-Pipe Total Time        : %10lu usec\n", r_pipe);
  printf("     R-Pipe CmplCnjg Time     : %10lu usec\n", r_cmpcnj);
  printf("     R-Pipe CmplMult Time     : %10lu usec\n", r_cmpmpy);
  printf("     R-Pipe FIRC Time         : %10lu usec\n", r_firc);
  printf("     R-Pipe CmplMag Time      : %10lu usec\n", r_cmpmag);
  printf("     R-Pipe CmplMag^2 Time    : %10lu usec\n", r_cmpmag2);
  printf("     R-Pipe FIR Time          : %10lu usec\n", r_fir);
  printf("     R-Pipe DIV Time          : %10lu usec\n", r_div);
  printf("     R-Pipe SyncShort Time    : %10lu usec\n", r_sshort);
  printf("       R-SySht Total Time         : %10lu usec\n", rssh_total);
  printf("       R-SySht Search Time        : %10lu usec\n", rssh_search);
  printf("       R-SySht Frame Time         : %10lu usec\n", rssh_frame);
  printf("     R-Pipe SyncLong Time     : %10lu usec\n", r_slong);
  printf("       R-SyLng Total Time         : %10lu usec\n", rslg_total);
  printf("       R-SyLng FIR-G Time         : %10lu usec\n", rslg_firG);
  printf("       R-SyLng Search Time        : %10lu usec\n", rslg_search);
  printf("       R-SyLng OutGen Time        : %10lu usec\n", rslg_outgen);
  printf("     R-Pipe Rc-FFT Time       : %10lu usec\n", r_fft);
  #ifdef RECV_HW_FFT
  printf("       R-Pipe rHfft_total Time  : %10lu usec\n", r_fHtotal);
  printf("       R-Pipe rHfft_cvtin Time  : %10lu usec\n", r_fHcvtin);
  printf("       R-Pipe rHfft_comp  Time  : %10lu usec\n", r_fHcomp);
  printf("       R-Pipe rHfft_cvtout Time : %10lu usec\n", r_fHcvtout);
  #endif
  printf("     R-Pipe Equalize Time     :  %10lu usec\n", r_eqlz);
  printf("       R-Eql Total Time         : %10lu usec\n", reql_total);
  printf("       R-Eql Set-Symbol Time    : %10lu usec\n", reql_sym_set);
  printf("       R-Eql LS-EQ Time         : %10lu usec\n", reql_ls_eql);
  printf("       R-Eql Output-Sym Time    : %10lu usec\n", reql_out_sym);
  printf("       R-Eql DecSigFld Time     : %10lu usec\n", reql_ds_fld);
  printf("     R-Pipe DecSignal Time    : %10lu usec\n", r_decsignl);
  printf("       R-Dec Total Time         : %10lu usec\n", rdec_total);
  printf("       R-Dec Map-BitR Time      : %10lu usec\n", rdec_map_bitr);
  printf("       R-Dec Get-Bits Time      : %10lu usec\n", rdec_get_bits);
  printf("       R-Dec Decode Call        : %10lu usec\n", rdec_dec_call);
  printf("     R-Pipe DeScramble Time   : %10lu usec\n", r_descrmbl);
  printf("\n");
 #endif // INT_TIME
 #else // TIME
  printf(" NO more detailed timing information on this run...\n");
#endif // TIME else
  printf("\nDone with the run...\n");
}
