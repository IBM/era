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
#include <sys/time.h>
#include <unistd.h>
//extern "C"{
#include "kernels_api.h"
#include "sim_environs.h"
#include "getopt.h"
#include "base_types.h"
//}
#define TIME

#ifdef INT_TIME
extern uint64_t calc_sec;
extern uint64_t calc_usec;

extern uint64_t fft_sec;
extern uint64_t fft_usec;

extern uint64_t bitrev_sec;
extern uint64_t bitrev_usec;

extern uint64_t fft_br_sec;
extern uint64_t fft_br_usec;

extern uint64_t fft_cvtin_sec;
extern uint64_t fft_cvtin_usec;

extern uint64_t fft_cvtout_sec;
extern uint64_t fft_cvtout_usec;

extern uint64_t cdfmcw_sec;
extern uint64_t cdfmcw_usec;

extern uint64_t dodec_sec;
extern uint64_t dodec_usec;

extern uint64_t depunc_sec;
extern uint64_t depunc_usec;
#endif

char cv_dict[256]; 
char rad_dict[256];
char vit_dict[256];
//char * cv_dict  = "traces/objects_dictionary.dfn";
//char * rad_dict = "traces/radar_dictionary.dfn";
//char * vit_dict = "traces/vit_dictionary.dfn";

bool_t all_obstacle_lanes_mode = false;
unsigned time_step;

void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpfule usage info\n");
  printf("    -o         : print the Visualizer output traace information during the run\n");
  printf("    -R <file>  : defines the input Radar dictionary file <file> to use\n");
  printf("    -V <file>  : defines the input Viterbi dictionary file <file> to use\n");
  printf("    -C <file>  : defines the input CV/CNN dictionary file <file> to use\n");
#ifdef USE_SIM_ENVIRON
  printf("    -s <N>     : Sets the max number of time steps to simulate\n");
  printf("    -r <N>     : Sets the rand random number seed to N\n");
  printf("    -A         : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)\n");
  printf("    -W <wfile> : defines the world environment parameters description file <wfile> to use\n");
#else
  printf("    -t <trace> : defines the input trace file <trace> to use\n");
#endif
  printf("    -f <N>     : defines Log2 number of FFT samples\n");
  printf("               :      14 = 2^14 = 16k samples (default); 10 = 1k samples\n");
  printf("    -n <N>     : defines number of Viterbi messages per time step behavior:\n");
  printf("               :      0 = One message per time step\n");
  printf("               :      1 = One message per obstacle per time step\n");
  printf("               :      2 = One msg per obstacle + 1 per time step\n");
  printf("    -v <N>     : defines Viterbi message size:\n");
  printf("               :      0 = Short messages (4 characters)\n");
  printf("               :      1 = Medium messages (500 characters)\n");
  printf("               :      2 = Long messages (1000 characters)\n");
  printf("               :      3 = Max-sized messages (1500 characters)\n");
}


int main(int argc, char *argv[])
{
  vehicle_state_t vehicle_state;
  label_t label;
  distance_t distance;
  message_t message;
#ifdef USE_SIM_ENVIRON
  char* world_desc_file_name = "default_world.desc";
#else
  char* trace_file = "";
#endif
  int opt;

  rad_dict[0] = '\0';
  vit_dict[0] = '\0';
  cv_dict[0] = '\0';

  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  while((opt = getopt(argc, argv, ":hAot:v:n:s:r:W:R:V:C:f:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 'A':
      all_obstacle_lanes_mode = true;
      break;
    case 'o':
      output_viz_trace = true;
      break;
    case 'R':
      snprintf(rad_dict, 255, "%s", optarg);
      break;
    case 'C':
      snprintf(cv_dict, 255, "%s", optarg);
      break;
    case 'V':
      snprintf(vit_dict, 255, "%s", optarg);
      break;
    case 's':
#ifdef USE_SIM_ENVIRON
      max_time_steps = atoi(optarg);
      printf("Using %u maximum time steps (simulation)\n", max_time_steps);
#endif
      break;
    case 'f':
      fft_logn_samples = atoi(optarg);
      if ((fft_logn_samples == 10) || (fft_logn_samples == 14)) {
	printf("Using 2^%u = %u samples for the FFT\n", fft_logn_samples, (1<<fft_logn_samples));
      } else {
	printf("Cannot specify FFT logn samples value %u (Legal values are 10, 14)\n", fft_logn_samples);
	exit(-1);
      }
      break;
    case 'r':
#ifdef USE_SIM_ENVIRON
      rand_seed = atoi(optarg);
#endif
      break;
    case 't':
#ifndef USE_SIM_ENVIRON
      trace_file = optarg;
      printf("Using trace file: %s\n", trace_file);
#endif
      break;
    case 'v':
      vit_msgs_size = atoi(optarg);
      if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
	printf("ERROR: Specified viterbi message size (%u) is larger than max (%u) : from the -v option\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
	exit(-1);
      } else {
	printf("Using viterbi message size %u = %s\n", vit_msgs_size, vit_msgs_size_str[vit_msgs_size]);
      }
      break;
    case 'n':
      vit_msgs_per_step = atoi(optarg);
      if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
	printf("ERROR: Specified viterbi messages per time step behavior (%u) is larger than max (%u) : from the -n option\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
	exit(-1);
      } else {
	printf("Using viterbi messages per step behavior %u = %s\n", vit_msgs_per_step, vit_msgs_per_step_str[vit_msgs_per_step]);
      }
      break;
    case 'W':
#ifdef USE_SIM_ENVIRON
      world_desc_file_name = optarg;
      printf("Using world description file: %s\n", world_desc_file_name);
#endif
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


  if (rad_dict[0] == '\0') {
    sprintf(rad_dict, "traces/radar_dictionary.dfn");
  }
  if (vit_dict[0] == '\0') {
    sprintf(vit_dict, "traces/vit_dictionary.dfn");
  }
  if (cv_dict[0] == '\0') {
    sprintf(cv_dict, "traces/objects_dictionary.dfn");
  }

  printf("\nDictionaries:\n");
  printf("   CV/CNN : %s\n", cv_dict);
  printf("   Radar  : %s\n", rad_dict);
  printf("   Viterbi: %s\n", vit_dict);

  /* We plan to use three separate trace files to drive the three different kernels
   * that are part of mini-ERA (CV, radar, Viterbi). All these three trace files
   * are required to have the same base name, using the file extension to indicate
   * which kernel the trace corresponds to (cv, rad, vit).
   */
  /* if (argc != 2) */
  /* { */
  /*   printf("Usage: %s <trace_basename>\n\n", argv[0]); */
  /*   printf("Where <trace_basename> is the basename of the trace files to load:\n"); */
  /*   printf("  <trace_basename>.cv  : trace to feed the computer vision kernel\n"); */
  /*   printf("  <trace_basename>.rad : trace to feed the radar (FFT-1D) kernel\n"); */
  /*   printf("  <trace_basename>.vit : trace to feed the Viterbi decoding kernel\n"); */

  /*   return 1; */
  /* } */


  char cv_py_file[] = "../cv/keras_cnn/lenet.py";

  printf("Doing initialization tasks...\n");
#ifndef USE_SIM_ENVIRON
  /* Trace filename construction */
  /* char * trace_file = argv[1]; */
  //printf("Input trace file: %s\n", trace_file);

  /* Trace Reader initialization */
  if (!init_trace_reader(trace_file))
  {
    printf("Error: the trace reader couldn't be initialized properly.\n");
    return 1;
  }
#endif
  /* Kernels initialization */
  printf("Initializing the CV kernel...\n");
  if (!init_cv_kernel(cv_py_file, cv_dict))
  {
    printf("Error: the computer vision kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Radar kernel...\n");
  if (!init_rad_kernel(rad_dict))
  {
    printf("Error: the radar kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Viterbi kernel...\n");
  if (!init_vit_kernel(vit_dict))
  {
    printf("Error: the Viterbi decoding kernel couldn't be initialized properly.\n");
    return 1;
  }

  /* We assume the vehicle starts in the following state:
   *  - Lane: center
   *  - Speed: 50 mph
   */
  vehicle_state.active  = true;
  vehicle_state.lane    = center;
  vehicle_state.speed   = 50;
  DEBUG(printf("Vehicle starts with the following state: active: %u lane %u speed %.1f\n", vehicle_state.active, vehicle_state.lane, vehicle_state.speed));

  #ifdef USE_SIM_ENVIRON
  // In simulation mode, we could start the main car is a different state (lane, speed)
  init_sim_environs(world_desc_file_name, &vehicle_state);
  #endif

/*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
  time_step = 0;
 #ifdef TIME
  struct timeval stop, start;

  struct timeval stop_iter_rad, start_iter_rad;
  struct timeval stop_iter_vit, start_iter_vit;
  struct timeval stop_iter_cv , start_iter_cv;

  uint64_t iter_rad_sec = 0LL;
  uint64_t iter_vit_sec = 0LL;
  uint64_t iter_cv_sec  = 0LL;

  uint64_t iter_rad_usec = 0LL;
  uint64_t iter_vit_usec = 0LL;
  uint64_t iter_cv_usec  = 0LL;

  struct timeval stop_exec_rad, start_exec_rad;
  struct timeval stop_exec_vit, start_exec_vit;
  struct timeval stop_exec_cv , start_exec_cv;

  uint64_t exec_rad_sec = 0LL;
  uint64_t exec_vit_sec = 0LL;
  uint64_t exec_cv_sec  = 0LL;

  uint64_t exec_rad_usec = 0LL;
  uint64_t exec_vit_usec = 0LL;
  uint64_t exec_cv_usec  = 0LL;
  //printf("Program run time in milliseconds %f\n", (double) (stop.tv_sec - start.tv_sec) * 1000 + (double) (stop.tv_usec - start.tv_usec) / 1000);
 #endif // TIME

  printf("Starting the main loop...\n");
  fflush(stdout);
  /* The input trace contains the per-epoch (time-step) input data */
#ifdef USE_SIM_ENVIRON
  DEBUG(printf("\n\nTime Step %d\n", time_step));
  while (iterate_sim_environs(vehicle_state))
#else //TRACE DRIVEN MODE
  read_next_trace_record(vehicle_state);
  while (!eof_trace_reader())
#endif
  {
    DEBUG(printf("Vehicle_State: Lane %u %s Speed %.1f\n", vehicle_state.lane, lane_names[vehicle_state.lane], vehicle_state.speed));

    /* The computer vision kernel performs object recognition on the
     * next image, and returns the corresponding label. 
     * This process takes place locally (i.e. within this car).
     */
   #ifdef TIME
    gettimeofday(&start_iter_cv, NULL);
   #endif
    label_t cv_tr_label = iterate_cv_kernel(vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_cv, NULL);
    iter_cv_sec  += stop_iter_cv.tv_sec  - start_iter_cv.tv_sec;
    iter_cv_usec += stop_iter_cv.tv_usec - start_iter_cv.tv_usec;
   #endif

    /* The radar kernel performs distance estimation on the next radar
     * data, and returns the estimated distance to the object.
     */
   #ifdef TIME
    gettimeofday(&start_iter_rad, NULL);
   #endif
    radar_dict_entry_t* rdentry_p = iterate_rad_kernel(vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_rad, NULL);
    iter_rad_sec  += stop_iter_rad.tv_sec  - start_iter_rad.tv_sec;
    iter_rad_usec += stop_iter_rad.tv_usec - start_iter_rad.tv_usec;
   #endif
    distance_t rdict_dist = rdentry_p->distance;
    float * ref_in = rdentry_p->return_data;
    float radar_inputs[2*RADAR_N];
    SDEBUG(printf("\nCopying radar inputs...\n"));
    for (unsigned int ii = 0; ii < 2*RADAR_N; ii++) {
      radar_inputs[ii] = ref_in[ii];
      #ifdef SUPER_VERBOSE
       if (ii < 64) { printf("radar_inputs[%2u] = %f  %f\n", radar_inputs[ii], ref_in[ii]); }
      #endif
    }

    /* The Viterbi decoding kernel performs Viterbi decoding on the next
     * OFDM symbol (message), and returns the extracted message.
     * This message can come from another car (including, for example,
     * its 'pose') or from the infrastructure (like speed violation or
     * road construction warnings). For simplicity, we define a fix set
     * of message classes (e.g. car on the right, car on the left, etc.)
     */
   #ifdef TIME
    gettimeofday(&start_iter_vit, NULL);
   #endif
    vit_dict_entry_t* vdentry_p = iterate_vit_kernel(vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_vit, NULL);
    iter_vit_sec  += stop_iter_vit.tv_sec  - start_iter_vit.tv_sec;
    iter_vit_usec += stop_iter_vit.tv_usec - start_iter_vit.tv_usec;
   #endif

    // Here we will simulate multiple cases, based on global vit_msgs_behavior
    int num_vit_msgs = 1;   // the number of messages to send this time step (1 is default) 
    switch(vit_msgs_per_step) {
    case 1: num_vit_msgs = total_obj; break;
    case 2: num_vit_msgs = total_obj + 1; break;
    }

    // EXECUTE the kernels using the now known inputs 
   #ifdef TIME
    gettimeofday(&start_exec_cv, NULL);
   #endif
    label = execute_cv_kernel(cv_tr_label);
   #ifdef TIME
    gettimeofday(&stop_exec_cv, NULL);
    exec_cv_sec  += stop_exec_cv.tv_sec  - start_exec_cv.tv_sec;
    exec_cv_usec += stop_exec_cv.tv_usec - start_exec_cv.tv_usec;

    gettimeofday(&start_exec_rad, NULL);
   #endif
    distance = execute_rad_kernel(radar_inputs);
   #ifdef TIME
    gettimeofday(&stop_exec_rad, NULL);
    exec_rad_sec  += stop_exec_rad.tv_sec  - start_exec_rad.tv_sec;
    exec_rad_usec += stop_exec_rad.tv_usec - start_exec_rad.tv_usec;

    gettimeofday(&start_exec_vit, NULL);
   #endif
    message = execute_vit_kernel(vdentry_p, num_vit_msgs);
   #ifdef TIME
    gettimeofday(&stop_exec_vit, NULL);
    exec_vit_sec  += stop_exec_vit.tv_sec  - start_exec_vit.tv_sec;
    exec_vit_usec += stop_exec_vit.tv_usec - start_exec_vit.tv_usec;
   #endif

    // POST-EXECUTE each kernels to gather stats, etc.
    post_execute_cv_kernel(cv_tr_label, label);
    post_execute_rad_kernel(rdentry_p->index, rdict_dist, distance);
    for (int mi = 0; mi < num_vit_msgs; mi++) {
      post_execute_vit_kernel(vdentry_p->msg_id, message);
    }

    /* The plan_and_control() function makes planning and control decisions
     * based on the currently perceived information. It returns the new
     * vehicle state.
     */
    vehicle_state = plan_and_control(label, distance, message, vehicle_state);
    DEBUG(printf("New vehicle state: lane %u speed %.1f\n\n", vehicle_state.lane, vehicle_state.speed));

    #ifdef TIME
    time_step++;
    if (time_step == 1) {
      gettimeofday(&start, NULL);
    }
    #endif

    #ifndef USE_SIM_ENVIRON
    read_next_trace_record(vehicle_state);
    #endif
  }

  #ifdef TIME
  	gettimeofday(&stop, NULL);
  #endif

  /* All the traces have been fully consumed. Quitting... */
  closeout_cv_kernel();
  closeout_rad_kernel();
  closeout_vit_kernel();

  #ifdef TIME
  {
    uint64_t total_exec = (uint64_t) (stop.tv_sec - start.tv_sec) * 1000000 + (uint64_t) (stop.tv_usec - start.tv_usec);
    uint64_t iter_rad   = (uint64_t) (iter_rad_sec) * 1000000 + (uint64_t) (iter_rad_usec);
    uint64_t iter_vit   = (uint64_t) (iter_vit_sec) * 1000000 + (uint64_t) (iter_vit_usec);
    uint64_t iter_cv    = (uint64_t) (iter_cv_sec)  * 1000000 + (uint64_t) (iter_cv_usec);
    uint64_t exec_rad   = (uint64_t) (exec_rad_sec) * 1000000 + (uint64_t) (exec_rad_usec);
    uint64_t exec_vit   = (uint64_t) (exec_vit_sec) * 1000000 + (uint64_t) (exec_vit_usec);
    uint64_t exec_cv    = (uint64_t) (exec_cv_sec)  * 1000000 + (uint64_t) (exec_cv_usec);
    printf("\nProgram total execution time     %lu usec\n", total_exec);
    printf("  iterate_rad_kernel run time    %lu usec\n", iter_rad);
    printf("  iterate_vit_kernel run time    %lu usec\n", iter_vit);
    printf("  iterate_cv_kernel run time     %lu usec\n", iter_cv);
    printf("  execute_rad_kernel run time    %lu usec\n", exec_rad);
    printf("  execute_vit_kernel run time    %lu usec\n", exec_vit);
    printf("  execute_cv_kernel run time     %lu usec\n", exec_cv);
  }
 #endif // TIME
 #ifdef INT_TIME
  // These are timings taken from called routines...
  printf("\n");
  uint64_t fft_tot = (uint64_t) (calc_sec)  * 1000000 + (uint64_t) (calc_usec);
  printf("  fft-total   run time    %lu usec\n", fft_tot);
 #ifdef HW_FFT
  uint64_t fft_br    = (uint64_t) (fft_br_sec)  * 1000000 + (uint64_t) (fft_br_usec);
  printf("  bitrev      run time    %lu usec\n", fft_br);
 #else 
  uint64_t bitrev    = (uint64_t) (bitrev_sec)  * 1000000 + (uint64_t) (bitrev_usec);
  printf("  bit-reverse run time    %lu usec\n", bitrev);
 #endif
  uint64_t fft_cvtin    = (uint64_t) (fft_cvtin_sec)  * 1000000 + (uint64_t) (fft_cvtin_usec);
  printf("  fft_cvtin   run time    %lu usec\n", fft_cvtin);
  uint64_t fft_comp    = (uint64_t) (fft_sec)  * 1000000 + (uint64_t) (fft_usec);
  printf("  fft-comp    run time    %lu usec\n", fft_comp);
  uint64_t fft_cvtout    = (uint64_t) (fft_cvtout_sec)  * 1000000 + (uint64_t) (fft_cvtout_usec);
  printf("  fft_cvtout  run time    %lu usec\n", fft_cvtout);
  uint64_t cdfmcw    = (uint64_t) (cdfmcw_sec)  * 1000000 + (uint64_t) (cdfmcw_usec);
  printf("  calc-dist   run time    %lu usec\n", cdfmcw);

  printf("\n");
  uint64_t depunc    = (uint64_t) (depunc_sec)  * 1000000 + (uint64_t) (depunc_usec);
  printf("  depuncture  run time    %lu usec\n", depunc);
  uint64_t dodec    = (uint64_t) (dodec_sec)  * 1000000 + (uint64_t) (dodec_usec);
  printf("  do-decoding run time    %lu usec\n", dodec);
 #endif // INT_TIME

  printf("\nDone.\n");
  return 0;
}
