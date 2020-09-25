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

#define MAX_XMIT_OUTPUTS   41800   // Really 41782 I think
#define MAX_MESSAGE_LEN     1500   // Max chars in a message (payload)

unsigned use_xmit_message = 0;
char* xmit_msg;
typedef struct msg_library_struct {
  unsigned msg_len;
  char     msg_text[1504];
} msg_library_entry_t;

#define XMIT_LIBRARY_SIZE      4
msg_library_entry_t xmit_msg_library[XMIT_LIBRARY_SIZE+1] = {
  {1500, "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"},
  {1500, "012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789"},
  {1500, "We hold these truths to be self-evident, that all men are created equal, that they are endowed by their Creator with certain unalienable Rights, that among these are Life, Liberty and the pursuit of Happiness.--That to secure these rights, Governments are instituted among Men, deriving their just powers from the consent of the governed, --That whenever any Form of Government becomes destructive of these ends, it is the Right of the People to alter or to abolish it, and to institute new Government, laying its foundation on such principles and organizing its powers in such form, as to them shall seem most likely to effect their Safety and Happiness. Prudence, indeed, will dictate that Governments long established should not be changed for light and transient causes; and accordingly all experience hath shewn, that mankind are more disposed to suffer, while evils are sufferable, than to right themselves by abolishing the forms to which they are accustomed. But when a long train of abuses and usurpations, pursuing invariably the same Object evinces a design to reduce them under absolute Despotism, it is their right, it is their duty, to throw off such Government, and to provide new Guards for their future security.--Such has been the patient sufferance of these Colonies; and such is now the necessity which constrains them to alter their former Systems of Government. The history of the present King of Great Britain is a history of repeated injuries and usurpations, all having etc."},
  { 4, "Msg0"},
  { 1500, "-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------" }
};

uint32_t xmit_msg_len = 0;

bool show_main_output = true;
bool show_xmit_output = false;
bool do_add_pre_pad = false;


void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -s <N>     : Run the simulation for <N> time steps\n");
  printf("    -m <N>     : Use message <N> (0 = 1500 'x' chars, 1 = 1500 '0123456789', 2 = quote)\n");
  printf("    -T \"S\"     : Use message S (No longer than 1500 chars)\n");
  printf("    -M <0|1>   : 0=disable 1=enable output of Messages (input and output) per time step\n");
  printf("    -x <0|1>   : 0=disable 1=enable output of XMIT output per time step\n");
  printf("    -o <FN>    : Output the encoded message data to file <FN>\n");
}


int main(int argc, char *argv[])
{
  int opt;
  int set_last_message = 0;
  char out_fname[256];

  out_fname[0] = '\0';
  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  while((opt = getopt(argc, argv, ":hs:m:M:x:r:T:o:")) != -1) {
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
    case 'x':
      show_xmit_output = (atoi(optarg) != 0);
      break;
    case 'm':
      use_xmit_message = atoi(optarg);
      //printf("Setting use_xmit_message to %u\n", use_xmit_message);
      break;
    case 'T': {
      use_xmit_message = XMIT_LIBRARY_SIZE;
      int mlen = strlen(optarg);
      xmit_msg_library[XMIT_LIBRARY_SIZE].msg_len = (mlen > 1500) ? 1500 : mlen;
      strncpy(xmit_msg_library[XMIT_LIBRARY_SIZE].msg_text, optarg, 1500);
      set_last_message = 1;
      printf("Setting to use a custom %u byte message...\n", xmit_msg_library[XMIT_LIBRARY_SIZE].msg_len);
    }
      break;
    case 'p':
      do_add_pre_pad = atoi(optarg);
      //printf("Setting do_add_pre_pad to %u\n", do_add_pre_pad);
      break;
    case 'o':
      snprintf(out_fname, 255, "%s", optarg);
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

  printf("Set show_xmit_output = %u\n", show_xmit_output);
  printf("Running for %u time steps\n", max_time_steps);
  if (use_xmit_message >= XMIT_LIBRARY_SIZE) {
    if ((set_last_message) && (use_xmit_message == XMIT_LIBRARY_SIZE)) {
      printf("Using a Custom XMIT message..\n");
    } else {
      printf("ERROR - Specified an illegal message identifier: %u (There are %u in library)\n", use_xmit_message, XMIT_LIBRARY_SIZE);
      exit(-1);
    }
  } else {
    printf("Using XMIT message %u (0 ='x', 1 = '0123456789')\n", use_xmit_message);
  }

  xmit_msg_len = xmit_msg_library[use_xmit_message].msg_len;
  xmit_msg = xmit_msg_library[use_xmit_message].msg_text;
  if (show_main_output) {
    printf("\nXMIT_MSG:\n'%s'\n\n", xmit_msg);
  }
  
  /* Kernels initialization */
  printf("Initializing the XMIT kernel...\n");
  if (!init_xmit_kernel()) {
    printf("Error: the computer vision kernel couldn't be initialized properly.\n");
    return 1;
  }

  /*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
 #ifdef TIME
  struct timeval stop, start;

  struct timeval stop_exec_xmit, start_exec_xmit;

  uint64_t exec_xmit_sec  = 0LL;

  uint64_t exec_xmit_usec  = 0LL;
 #endif // TIME

  printf("Starting the main loop...\n");
  /* The input trace contains the per-epoch (time-step) input data */
  int time_step = 0;
 #ifdef TIME
  gettimeofday(&start, NULL);
 #endif
  for (time_step = 0; time_step < max_time_steps; time_step++) {
    DEBUG(printf("Time Step %u\n", time_step));

    /* The xmit kernel does a transmission pass */
    int xmit_num_out;
    float xmit_out_real[MAX_XMIT_OUTPUTS];
    float xmit_out_imag[MAX_XMIT_OUTPUTS];
   #ifdef TIME
    gettimeofday(&start_exec_xmit, NULL);
   #endif
    execute_xmit_kernel(xmit_msg_len, xmit_msg, &xmit_num_out, xmit_out_real, xmit_out_imag);
   #ifdef TIME
    gettimeofday(&stop_exec_xmit, NULL);
    exec_xmit_sec  += stop_exec_xmit.tv_sec  - start_exec_xmit.tv_sec;
    exec_xmit_usec += stop_exec_xmit.tv_usec - start_exec_xmit.tv_usec;
   #endif
    if (show_xmit_output) {
      printf("\nXMIT Pipe Final output:\n");
      for (int i = 0; i < xmit_num_out; i++) {
        printf(" xmit_out_res %6u : %11.8f + %11.8f i\n", i, xmit_out_real[i], xmit_out_imag[i]);
      }
      printf("\n");
    }

    // Check whether we are to save this encoding into a file (for use by the receiver stand-alone)
    if ((time_step == 0) && (out_fname[0] != '\0')) {
      FILE *outF = fopen(out_fname, "w");
      if (!outF) {
	printf("Error: unable to open output encoded message file %s\n", out_fname);
	return error;
      }
      fprintf(outF, "%u %u\n", xmit_msg_len, xmit_num_out);
      for (int i = 0; i < xmit_num_out; i++) {
	fprintf(outF, "%12.8f %12.8f", xmit_out_real[i], xmit_out_imag[i]);
      }
      fclose(outF);
    }

    /* // POST-EXECUTE each kernels to gather stats, etc. */
    /* post_execute_xmit_kernel(xmit_tr_label, label); */
    /* post_execute_rad_kernel(rdentry_p->index, rdict_dist, distance); */
    /* for (int mi = 0; mi < num_vit_msgs; mi++) { */
    /*   post_execute_vit_kernel(vdentry_p->msg_id, message); */
    /* } */
  }

 #ifdef TIME
  gettimeofday(&stop, NULL);
 #endif

  
  /* All the traces have been fully consumed. Quitting... */
  closeout_xmit_kernel();

  #ifdef TIME
  {
    uint64_t total_exec = (uint64_t) (stop.tv_sec - start.tv_sec) * 1000000 + (uint64_t) (stop.tv_usec - start.tv_usec);
    uint64_t exec_xmit    = (uint64_t) (exec_xmit_sec)  * 1000000 + (uint64_t) (exec_xmit_usec);
    printf("\nProgram total execution time     %lu usec\n", total_exec);
    printf("  execute_xmit_kernel run time     %lu usec\n", exec_xmit);
  }
 #endif // TIME
 #ifdef INT_TIME
  // These are timings taken from called routines...
  printf("\n");
 #endif // INT_TIME

  printf("\nDone.\n");
  return 0;
}
