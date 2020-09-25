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
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>
#include <getopt.h>

#include "debug.h"
#include "kernels_api.h"

#define TIME

unsigned max_time_steps = 1;

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

  
void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -s <N>     : Run the simulation for <N> time steps\n");
  printf("    -f <FN>    : Use file <FN> as input encoded message source\n");
  printf("    -M <0|1>   : 0=disable 1=enable output of Messages (input and output) per time step\n");
  printf("    -r <0|1>   : 0=disable 1=enable output of RECV output per time step\n");
}


int main(int argc, char *argv[])
{
  int opt;

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

  printf("Set show_recv_output = %u\n", show_recv_output);
  printf("Running for %u time steps\n", max_time_steps);
  printf("RECV message is taken from file %s\n", recv_in_fname);

  /* Kernels initialization */
  printf("Initializing the Receive kernel...\n");
  if (!init_recv_kernel()) {
    printf("Error: the receive kernel couldn't be initialized properly.\n");
    return 1;
  }

  /*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
 #ifdef TIME
  struct timeval stop, start;
  struct timeval stop_exec_recv, start_exec_recv;
  uint64_t exec_recv_sec = 0LL;
  uint64_t exec_recv_usec = 0LL;
 #endif // TIME

  printf("Starting the main loop...\n");
  /* The input trace contains the per-epoch (time-step) input data */
  int time_step = 0;
 #ifdef TIME
  gettimeofday(&start, NULL);
 #endif
  for (time_step = 0; time_step < max_time_steps; time_step++) {
    DEBUG(printf("Time Step %u\n", time_step));

    /* The receive kernel does a receive of one message */
    int  recv_msg_len;
    char recv_msg[MAX_MESSAGE_LEN];
   #ifdef TIME
    gettimeofday(&start_exec_recv, NULL);
   #endif
    execute_recv_kernel(xmit_msg_len, xmit_num_out, xmit_out_real, xmit_out_imag, &recv_msg_len, recv_msg);    
   #ifdef TIME
    gettimeofday(&stop_exec_recv, NULL);
    exec_recv_sec  += stop_exec_recv.tv_sec  - start_exec_recv.tv_sec;
    exec_recv_usec += stop_exec_recv.tv_usec - start_exec_recv.tv_usec;
   #endif

    if (show_main_output) {
      printf("Iteration %u : RECV_MSG:\n'%s'\n", time_step, recv_msg);
    }
    /* // POST-EXECUTE each kernels to gather stats, etc. */
    /* post_execute_rad_kernel(rdentry_p->index, rdict_dist, distance); */
    /* for (int mi = 0; mi < num_vit_msgs; mi++) { */
    /*   post_execute_vit_kernel(vdentry_p->msg_id, message); */
    /* } */
  }

 #ifdef TIME
  gettimeofday(&stop, NULL);
 #endif

  /* All the traces have been fully consumed. Quitting... */
  closeout_recv_kernel();

  #ifdef TIME
  {
    uint64_t total_exec = (uint64_t) (stop.tv_sec - start.tv_sec) * 1000000 + (uint64_t) (stop.tv_usec - start.tv_usec);
    uint64_t exec_recv   = (uint64_t) (exec_recv_sec) * 1000000 + (uint64_t) (exec_recv_usec);
    printf("\nProgram total execution time     %lu usec\n", total_exec);
    printf("  execute_recv_kernel run time    %lu usec\n", exec_recv);
  }
 #endif // TIME
 #ifdef INT_TIME
  // These are timings taken from called routines...
  printf("\n");
 #endif // INT_TIME

  printf("\nDone.\n");
  return 0;
}
