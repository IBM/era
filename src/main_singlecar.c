#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <signal.h>
#include <pthread.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <arpa/inet.h> // for inet_addr
#include <sys/time.h>

#include "globals.h"
#include "debug.h"
#include "getopt.h"

#include "occgrid.h"    // Occupancy Grid Map Create/Fuse
#include "lz4.h"        // LZ4 Compression/Decompression
#include "xmit_pipe.h"  // IEEE 802.11p WiFi SDR Transmit Pipeline
#include "recv_pipe.h"  // IEEE 802.11p WiFi SDR Receive Pipeline

// The PORTS are defined in the compilation process, and comforms to the
// definition in the read_bag_x.py files and wifi_comm_x.py files.

char bag_inet_addr_str[20];
char wifi_inet_addr_str[20];
char car_inet_addr_str[20];
unsigned  max_time_steps = ~1;

int bag_sock = 0;
int xmit_sock = 0;
int recv_sock = 0;
int car_sock = 0;

char *ack = "OK";

float odometry[] = {0.0, 0.0, 0.0};

// We will define 2 observations; one "current" and one that is to be constructed to be the new current.
int curr_obs = 1;
int next_obs = 0;
Observation observations[2];

// These variables capture "time" spent in various parts ofthe workload
struct timeval stop_prog, start_prog;

struct timeval stop_proc_odo, start_proc_odo;
uint64_t proc_odo_sec  = 0LL;
uint64_t proc_odo_usec = 0LL;

struct timeval stop_proc_rdbag, start_proc_rdbag;
uint64_t proc_rdbag_sec  = 0LL;
uint64_t proc_rdbag_usec = 0LL;

struct timeval stop_proc_lidar, start_proc_lidar;
uint64_t proc_lidar_sec  = 0LL;
uint64_t proc_lidar_usec = 0LL;

struct timeval stop_proc_data, start_proc_data;
uint64_t proc_data_sec  = 0LL;
uint64_t proc_data_usec = 0LL;

#ifdef INT_TIME
struct timeval stop_pd_cloud2grid, start_pd_cloud2grid;
uint64_t pd_cloud2grid_sec  = 0LL;
uint64_t pd_cloud2grid_usec = 0LL;

struct timeval stop_pd_lz4_cmp, start_pd_lz4_cmp;
uint64_t pd_lz4_cmp_sec  = 0LL;
uint64_t pd_lz4_cmp_usec = 0LL;

struct timeval stop_pd_wifi_pipe, start_pd_wifi_pipe;
uint64_t pd_wifi_pipe_sec  = 0LL;
uint64_t pd_wifi_pipe_usec = 0LL;

struct timeval stop_pd_wifi_send, start_pd_wifi_send;
uint64_t pd_wifi_send_sec  = 0LL;
uint64_t pd_wifi_send_usec = 0LL;

struct timeval stop_pd_wifi_send_rl, start_pd_wifi_send_rl;
uint64_t pd_wifi_send_rl_sec  = 0LL;
uint64_t pd_wifi_send_rl_usec = 0LL;

struct timeval stop_pd_wifi_send_im, start_pd_wifi_send_im;
uint64_t pd_wifi_send_im_sec  = 0LL;
uint64_t pd_wifi_send_im_usec = 0LL;

struct timeval stop_pd_wifi_recv_th, start_pd_wifi_recv_th;
uint64_t pd_wifi_recv_th_sec  = 0LL;
uint64_t pd_wifi_recv_th_usec = 0LL;

struct timeval stop_pd_wifi_lmap_wait, start_pd_wifi_lmap_wait;
uint64_t pd_wifi_lmap_wait_sec  = 0LL;
uint64_t pd_wifi_lmap_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_wait, start_pd_wifi_recv_wait;
uint64_t pd_wifi_recv_wait_sec  = 0LL;
uint64_t pd_wifi_recv_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_all, start_pd_wifi_recv_all;
uint64_t pd_wifi_recv_all_sec  = 0LL;
uint64_t pd_wifi_recv_all_usec = 0LL;

struct timeval stop_pd_wifi_recv_rl, start_pd_wifi_recv_rl;
uint64_t pd_wifi_recv_rl_sec  = 0LL;
uint64_t pd_wifi_recv_rl_usec = 0LL;

struct timeval stop_pd_wifi_recv_im, start_pd_wifi_recv_im;
uint64_t pd_wifi_recv_im_sec  = 0LL;
uint64_t pd_wifi_recv_im_usec = 0LL;

struct timeval stop_pd_recv_pipe, start_pd_recv_pipe;
uint64_t pd_recv_pipe_sec  = 0LL;
uint64_t pd_recv_pipe_usec = 0LL;

struct timeval stop_pd_lz4_uncmp, start_pd_lz4_uncmp;
uint64_t pd_lz4_uncmp_sec  = 0LL;
uint64_t pd_lz4_uncmp_usec = 0LL;

struct timeval stop_pd_combGrids, start_pd_combGrids;
uint64_t pd_combGrids_sec  = 0LL;
uint64_t pd_combGrids_usec = 0LL;

struct timeval stop_pd_wifi_car, start_pd_wifi_car;
uint64_t pd_wifi_car_sec  = 0LL;
uint64_t pd_wifi_car_usec = 0LL;

#endif


int counter = 0;
int ascii_counter = 0;

unsigned odo_count = 0;
unsigned lidar_count = 0;
unsigned lmap_count = 0;
unsigned xmit_count = 0;
unsigned recv_count = 0;
unsigned car_send_count = 0;

// Forward Declarations
void print_usage(char * pname);
void dump_final_run_statistics();
void INThandler(int dummy);
// in globals.h void closeout_and_exit(char* last_msg, int rval);



// Functions, code, etc.
void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -B <str>   : set the internet-address for the bagfile server to <str>\n");
  printf("    -W <str>   : set the internet-address for the WiFi server to <str>\n");
  printf("    -C <str>   : set the internet-address for the Car Map Output server to <str>\n");
  printf("    -s <Num>   : exit run after <Num> Lidar time-steps (msgs)\n");
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


#ifdef HW_VIT
 extern void init_VIT_HW_ACCEL();
 extern void free_VIT_HW_RESOURCES();
#endif
#ifdef XMIT_HW_FFT
 extern void free_XMIT_FFT_HW_RESOURCES();
#endif
#ifdef RECV_HW_FFT
 extern void free_RECV_FFT_HW_RESOURCES();
#endif

// This cleans up the state before exit
void closeout_and_exit(char* last_msg, int rval)
{
  if (lidar_count > 0) {
    dump_final_run_statistics();
  }
  printf("closeout_and_exit -- Closing the connection and exiting %d\n", rval);
  if (bag_sock != 0) {
    close(bag_sock);
  }
  if (xmit_sock != 0) {
    close(xmit_sock);
  }
  if (recv_sock != 0) {
    close(recv_sock);
  }
  if (car_sock != 0) {
    close(car_sock);
  }

 #ifdef HW_VIT
  free_VIT_HW_RESOURCES();
 #endif // HW_VIT
 #ifdef XMIT_HW_FFT
  free_XMIT_FFT_HW_RESOURCES();
 #endif
 #ifdef RECV_HW_FFT
  free_RECV_FFT_HW_RESOURCES();
 #endif
  printf("%s\n", last_msg);
  exit(rval);
}

/* 
   float bytes_to_float(unsigned char * bytes)
   {
   unsigned char b[] = {bytes[0], bytes[1], bytes[2], bytes[3]};
   float f;
   memcpy(&f, &b, sizeof(f));

   return f;
   }
*/

void write_array_to_file(unsigned char * data, long size)
{
  const int dimx = 50, dimy = 50;
  int i, j;

  char file_name[32];

  snprintf(file_name,sizeof(char)*32, "%s%04d.ppm", IMAGE_FN, counter);

  FILE *fp = fopen(file_name, "w");
  fprintf(fp, "P3 %d %d 255\n", dimx, dimy);

  for (j = 0; j < dimy*dimx; ++j) {
    fprintf(fp, " %d %d %d ", data[j], data[j], data[j]);
  }

  fclose(fp);
  counter++;
}

#define MAX_UNCOMPRESSED_DATA_SIZE  sizeof(Costmap2D) // MAX_GRID_SIZE
#define MAX_COMPRESSED_DATA_SIZE    MAX_UNCOMPRESSED_DATA_SIZE //(In case of no compression)?  

#define MAX_XMIT_OUTPUTS  41800  // Really something like 41782 I think

int read_all(int sock, char* buffer, int xfer_in_bytes)
{
  char * ptr;
  int message_size = xfer_in_bytes;
  char* message_ptr = buffer;
  int total_recvd = 0;
  while(total_recvd < message_size) {
    unsigned rem_len = (message_size - total_recvd);
    int valread = read(sock , message_ptr, rem_len);
    message_ptr = message_ptr + valread;
    total_recvd += valread;
    DBGOUT2(printf("        read %d bytes for %d total bytes of %d\n", valread, total_recvd, message_size));
    printf("        read %d bytes for %d total bytes of %d\n", valread, total_recvd, message_size);
    if (valread == 0) {
      DBGOUT(printf("  read_all got ZERO bytes -- END of TRANSFER?\n"));
      return total_recvd;
    }
  }
  return total_recvd;
}

void* receive_and_fuse_maps(void* parm_ptr)
{
  // Now we take in a received transmission with the other AV's map
  // If we receive a transmission, the process to turn it back into the gridMap is:
  int   n_recvd_in;
  float recvd_in_real[MAX_XMIT_OUTPUTS];
  float recvd_in_imag[MAX_XMIT_OUTPUTS];
 #if PARALLEL_PTHREADS
  printf("The receive_and_fuse_maps routine is started: lmap_count = %u\n", lmap_count);
 #endif
 #ifdef INT_TIME
  gettimeofday(&start_pd_wifi_recv_th, NULL);
 #endif
  while(lmap_count == 0) {
    DEBUG(printf("  lmap_count = %u\n", lmap_count));
    usleep(1);
    ; // wait for first observation (local map) to exist
  }
 #ifdef INT_TIME
  gettimeofday(&stop_pd_wifi_lmap_wait, NULL);
  pd_wifi_lmap_wait_sec   += stop_pd_wifi_lmap_wait.tv_sec  - start_pd_wifi_recv_th.tv_sec;
  pd_wifi_lmap_wait_usec  += stop_pd_wifi_lmap_wait.tv_usec - start_pd_wifi_recv_th.tv_usec;
 #endif
 #if PARALLEL_PTHREADS
  while(1) {
 #endif
    DBGOUT(printf("\nTrying to Receive data on RECV port %u socket\n", RECV_PORT));
   #ifdef INT_TIME
    gettimeofday(&start_pd_wifi_recv_wait, NULL);
   #endif
    char r_buffer[10];
    int valread = read_all(recv_sock, r_buffer, 8);
    DBGOUT(printf("  RECV got %d bytes :'%s'\n", valread, r_buffer));
    DBGOUT2(printf("  RECV msg psn %s\n", "01234567890"));
    if (valread == 8) {
     #ifdef INT_TIME
      gettimeofday(&start_pd_wifi_recv_all, NULL);
     #endif
      if(!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) {
	printf("ERROR: Unexpected message from WiFi...\n");
	closeout_and_exit("Unexpected WiFi message...", -3);
      }
      send(recv_sock, ack, 2, 0);

      char * ptr;
      unsigned xfer_in_bytes = strtol(r_buffer+1, &ptr, 10);
      n_recvd_in = xfer_in_bytes / sizeof(float);
      DBGOUT(printf("     Recv %u REAL values %u bytes from RECV port %u socket\n", n_recvd_in, xfer_in_bytes, RECV_PORT));
     #ifdef INT_TIME
      gettimeofday(&start_pd_wifi_recv_rl, NULL);
     #endif	
      valread = read_all(recv_sock, (char*)recvd_in_real, xfer_in_bytes);
      if (valread < xfer_in_bytes) {
	if (valread == 0) {
	  printf("  RECV REAL got ZERO bytes -- END of TRANSFER?\n");
	  closeout_and_exit("RECV REAL got zero bytes..", -1);
	} else {
	  printf("  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
	  closeout_and_exit("RECV REAL got too few bytes..", -1);
	}
      }
      DBGOUT2(printf("XFER %4u : Dumping RECV-PIPE REAL raw bytes\n", recv_count);
	      for (int i = 0; i < n_recvd_in; i++) {
		printf("XFER %4u REAL-byte %6u : %f\n", odo_count, i, recvd_in_real[i]);
	      }
	      printf("\n"));

     #ifdef INT_TIME
      gettimeofday(&stop_pd_wifi_recv_rl, NULL);
     #endif
      DBGOUT(printf("     Recv %u IMAG values %u bytes from RECV port %u socket\n", n_recvd_in, xfer_in_bytes, RECV_PORT));
      valread = read_all(recv_sock, (char*)recvd_in_imag, xfer_in_bytes);
      if (valread < xfer_in_bytes) {
	if (valread == 0) {
	  printf("  RECV IMAG got ZERO bytes -- END of TRANSFER?\n");
	  closeout_and_exit("RECV IMAG got zero bytes..", -1);
	} else {
	  printf("  RECV IMAG got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
	  closeout_and_exit("RECV IMAG got too few bytes..", -1);
	}
      }
      DBGOUT2(printf("XFER %4u : Dumping RECV-PIPE IMAG raw bytes\n", xmit_recv_count);
	      for (int i = 0; i < n_recvd_in; i++) {
		printf("XFER %4u IMAG-byte %6u : %f\n", odo_count, i, recvd_in_imag[i]);
	      }
	      printf("\n"));
     #ifdef INT_TIME
      gettimeofday(&stop_pd_wifi_recv_all, NULL);
      pd_wifi_recv_wait_sec  += start_pd_wifi_recv_all.tv_sec  - start_pd_wifi_recv_wait.tv_sec;
      pd_wifi_recv_wait_usec += start_pd_wifi_recv_all.tv_usec - start_pd_wifi_recv_wait.tv_usec;
      pd_wifi_recv_all_sec   += stop_pd_wifi_recv_all.tv_sec  - start_pd_wifi_recv_all.tv_sec;
      pd_wifi_recv_all_usec  += stop_pd_wifi_recv_all.tv_usec - start_pd_wifi_recv_all.tv_usec;
      pd_wifi_recv_rl_sec   += stop_pd_wifi_recv_rl.tv_sec  - start_pd_wifi_recv_rl.tv_sec;
      pd_wifi_recv_rl_usec  += stop_pd_wifi_recv_rl.tv_usec - start_pd_wifi_recv_rl.tv_usec;
      pd_wifi_recv_im_sec   += stop_pd_wifi_recv_all.tv_sec  - stop_pd_wifi_recv_rl.tv_sec;
      pd_wifi_recv_im_usec  += stop_pd_wifi_recv_all.tv_usec - stop_pd_wifi_recv_rl.tv_usec;
     #endif

      // Now we have the tranmission input data to be decoded...
      DBGOUT(printf("Calling do_recv_pipeline...\n"));
      int   recvd_msg_len;
      unsigned char recvd_msg[1500]; // MAX size of original message in bytes
      // Fake this with a "loopback" of the xmit message..
     #ifdef INT_TIME
      gettimeofday(&start_pd_recv_pipe, NULL);
     #endif	
      do_recv_pipeline(n_recvd_in, recvd_in_real, recvd_in_imag, &recvd_msg_len, recvd_msg);
     #ifdef INT_TIME
      gettimeofday(&stop_pd_recv_pipe, NULL);
      pd_recv_pipe_sec   += stop_pd_recv_pipe.tv_sec  - start_pd_recv_pipe.tv_sec;
      pd_recv_pipe_usec  += stop_pd_recv_pipe.tv_usec - start_pd_recv_pipe.tv_usec;
     #endif
      //do_recv_pipeline(n_xmit_out, xmit_out_real, xmit_out_imag, &recvd_msg_len, recvd_msg);

      // Now we decompress the grid received via transmission...
      DBGOUT(printf("Calling LZ4_decompress_default...\n"));
      unsigned char uncmp_data[MAX_UNCOMPRESSED_DATA_SIZE];
     #ifdef INT_TIME
      gettimeofday(&start_pd_lz4_uncmp, NULL);
     #endif
      DEBUG(printf("Calling LZ4_decompress_safe with %d input bytes...\n", recvd_msg_len));
      int dec_bytes = LZ4_decompress_safe((char*)recvd_msg, (char*)uncmp_data, recvd_msg_len, MAX_UNCOMPRESSED_DATA_SIZE);
      if (dec_bytes < 0) {
	printf("LZ4_decompress_safe ERROR : %d\n", dec_bytes);
      } DEBUG(else {
	  printf("LZ4_decompress_safe returned %d bytes\n", dec_bytes);
	});
     #ifdef INT_TIME
      gettimeofday(&stop_pd_lz4_uncmp, NULL);
      pd_lz4_uncmp_sec   += stop_pd_lz4_uncmp.tv_sec  - start_pd_lz4_uncmp.tv_sec;
      pd_lz4_uncmp_usec  += stop_pd_lz4_uncmp.tv_usec - start_pd_lz4_uncmp.tv_usec;
     #endif
      DEBUG(printf("Recevied %d decoded bytes from the wifi...\n", dec_bytes));
      Costmap2D* remote_map = (Costmap2D*)&(uncmp_data); // Convert "type" to Costmap2D
      DBGOUT(printf("  Back from LZ4_decompress_safe with %u decompressed bytes\n", dec_bytes);
	     printf("  Remote CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
	     printf("                : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map->cell_size, remote_map->x_dim, remote_map->y_dim);
	     print_ascii_costmap(stdout, remote_map));

      // Get the current local-map
      printf("Receive step %u : Processing fusion for curr_obs = %d\n", recv_count, curr_obs);
      Costmap2D* local_map = &(observations[curr_obs].master_costmap);

     #ifdef WRITE_ASCII_MAP
      char ascii_file_name[32];
      snprintf(ascii_file_name, sizeof(char)*32, "%s%04d.txt", ASCII_FN, ascii_counter);
      FILE *ascii_fp = fopen(ascii_file_name, "w");

      //printf("Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
      fprintf(ascii_fp, "Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
      fprintf(ascii_fp, "             : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim, local_map->y_dim);
      print_ascii_costmap(ascii_fp, local_map);

      fprintf(ascii_fp, "\n\nRemote CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
      //printf("\n\nRemote CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
      fprintf(ascii_fp, "              : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map->cell_size, remote_map->x_dim, remote_map->y_dim);
      print_ascii_costmap(ascii_fp, remote_map);
     #endif	
      // Then we should "Fuse" the received GridMap with our local one
      //  We need to "peel out" the remote odometry data from somewhere (in the message?)
      //unsigned char* combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
      DBGOUT(printf("\nCalling combineGrids...\n"));
      // Note: The direction in which this is called is slightly significant:
      //  The first map is copied into the second map, in this case remote into local,
      //  and the x_dim, et.c MUST correspond to that second map (here local)
     #ifdef INT_TIME
      gettimeofday(&start_pd_combGrids, NULL);
     #endif

      fuseIntoLocal(local_map, remote_map);
      /*combineGrids(remote_map->costmap, local_map->costmap,
	remote_map->av_x, remote_map->av_y,
	local_map->av_x, local_map->av_y,
	local_map->x_dim, local_map->y_dim, local_map->cell_size);
      */
     #ifdef INT_TIME
      gettimeofday(&stop_pd_combGrids, NULL);
      pd_combGrids_sec   += stop_pd_combGrids.tv_sec  - start_pd_combGrids.tv_sec;
      pd_combGrids_usec  += stop_pd_combGrids.tv_usec - start_pd_combGrids.tv_usec;
     #endif
      DEBUG2(printf(ascii_fp, "  Fused CostMAP : AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
	     printf(ascii_fp, "                : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim, local_map->y_dim);
	     print_ascii_costmap(stdout, local_map));
	  
     #ifdef WRITE_ASCII_MAP
      fprintf(ascii_fp, "\n\nFused CostMAP : AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
      //printf("\n\nFused CostMAP : AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
      fprintf(ascii_fp, "              : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim, local_map->y_dim);
      print_ascii_costmap(ascii_fp, local_map);
      fclose(ascii_fp);
      ascii_counter++;
     #endif
      // Write the combined map to a file
     #ifdef WRITE_FUSED_MAPS
      write_array_to_file(local_map->costmap, COST_MAP_ENTRIES);
     #endif

      // This is now the fused map that should be sent to the AV(Car)
      //  The n values of the (fused) local_map Costmap
      // Connect to the Car-Socket and send the data...
     #ifdef INT_TIME
      gettimeofday(&start_pd_wifi_send, NULL);
     #endif	
      unsigned car_bytes = sizeof(Costmap2D);
      snprintf(r_buffer, 9, "X%-6uX", car_bytes);
      DBGOUT(printf("\nCAR-OUT Sending %s on CAR port %u socket\n", r_buffer, CAR_PORT));
      send(car_sock, r_buffer, 8, 0);
      DBGOUT(printf("     Send %u bytes on CAR port %u socket\n", car_bytes, CAR_PORT));
      char * car_out_chars = (char*)&(local_map);
      DBGOUT2(printf("CAR-OUT %4u : Dumping XMIT-PIPE REAL raw bytes\n", car_send_count);
	      for (int i = 0; i < car_bytes; i++) {
		unsigned char c = car_out_chars[i];
		printf("CAR-OUT %4u REAL-byte %6u : %u\n", car_sendcount, i, c);
	      }
	      printf("\n"));
     #ifdef INT_TIME
      gettimeofday(&start_pd_wifi_car, NULL);
     #endif	
      send(car_sock, car_out_chars, car_bytes, 0);
      DBGOUT(printf("     Send %u bytes on CAR port %u socket\n", car_bytes, CAR_PORT));
     #ifdef INT_TIME
      gettimeofday(&stop_pd_wifi_car, NULL);
      pd_wifi_car_sec   += stop_pd_wifi_car.tv_sec  - start_pd_wifi_car.tv_sec;
      pd_wifi_car_usec  += stop_pd_wifi_car.tv_usec - start_pd_wifi_car.tv_usec;
     #endif
      car_send_count++;
      recv_count++;
    } else { // if (valread == 8)
     #ifdef INT_TIME
      gettimeofday(&stop_pd_wifi_recv_wait, NULL);
      pd_wifi_recv_wait_sec  += stop_pd_wifi_recv_wait.tv_sec  - start_pd_wifi_recv_wait.tv_sec;
      pd_wifi_recv_wait_usec += stop_pd_wifi_recv_wait.tv_usec - start_pd_wifi_recv_wait.tv_usec;
     #endif
     #if PARALLEL_PTHREADS
      if (valread == 0) {
    	printf("  RECV header got ZERO bytes -- END of TRANSFER?\n");
    	closeout_and_exit("RECV header got zero bytes...", -1);
      } else {
    	printf("  RECV header got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, 8);
    	closeout_and_exit("RECV header got too few bytes...", -1);
      }
     #endif
    }
   #ifdef INT_TIME
    gettimeofday(&stop_pd_wifi_recv_th, NULL);
    pd_wifi_recv_th_sec  = stop_pd_wifi_recv_th.tv_sec  - start_pd_wifi_recv_th.tv_sec;
    pd_wifi_recv_th_usec = stop_pd_wifi_recv_th.tv_usec - start_pd_wifi_recv_th.tv_usec;
   #endif
 #if PARALLEL_PTHREADS
  } // while (1)
 #endif
}

typedef struct lidar_inputs_struct {
  float odometry[3];
  int data_size;
  char data[200002];
} lidar_inputs_t;
  
void process_lidar_to_occgrid(lidar_inputs_t* lidar_inputs)
{
  DBGOUT(printf("Lidar step %u : Calling cloudToOccgrid next_obs = %d with odometry %.1f %.1f %.1f\n", lidar_count, next_obs, lidar_inputs->odometry[0], lidar_inputs->odometry[1], lidar_inputs->odometry[2]));
  printf("Lidar step %u : Calling cloudToOccgrid next_obs = %d with odometry %.1f %.1f %.1f\n", lidar_count, next_obs, lidar_inputs->odometry[0], lidar_inputs->odometry[1], lidar_inputs->odometry[2]);
  int valread = 0;
 #ifdef INT_TIME
  gettimeofday(&start_pd_cloud2grid, NULL);
 #endif	
  unsigned char * grid = cloudToOccgrid(&observations[next_obs], (float*)(lidar_inputs->data), lidar_inputs->data_size/sizeof(float), // data, data_size
					lidar_inputs->odometry[0], lidar_inputs->odometry[1], lidar_inputs->odometry[2], 1.5, // AVx, AVy , AVz, AVw
					false,  // rolling window
					0.05, 2.05, // min, max obstacle height
					RAYTR_RANGE, // raytrace_range
					GRID_MAP_X_DIM, GRID_MAP_Y_DIM, GRID_MAP_RESLTN  // size_x, size_y, resolution
					/*,CMV_NO_INFORMATION*/);
 #ifdef INT_TIME
  gettimeofday(&stop_pd_cloud2grid, NULL);
  pd_cloud2grid_sec   += stop_pd_cloud2grid.tv_sec  - start_pd_cloud2grid.tv_sec;
  pd_cloud2grid_usec  += stop_pd_cloud2grid.tv_usec - start_pd_cloud2grid.tv_usec;
 #endif
  
  // Write the read-in image to a file
  //write_array_to_file(grid, COST_MAP_ENTRIES);

  // Now we compress the grid for transmission...
  Costmap2D* local_map = &(observations[next_obs].master_costmap);
  DBGOUT(printf("Calling LZ4_compress_default...\n"));
  printf("Calling LZ4_compress_default...\n");
  DBGOUT(printf("  Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
	 printf("               : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim, local_map->y_dim);
	 print_ascii_costmap(stdout, local_map));
 #ifdef WRITE_ASCII_MAP
  char ascii_file_name[32];
  snprintf(ascii_file_name, sizeof(char)*32, "%sform_%04d.txt", ASCII_FN, ascii_counter);
  FILE *ascii_fp = fopen(ascii_file_name, "w");
  //printf("Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
  fprintf(ascii_fp, "Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
  fprintf(ascii_fp, "             : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim, local_map->y_dim);
  print_ascii_costmap(ascii_fp, local_map);
  fclose(ascii_fp);
 #endif
  // Now we update the current_observation index and the next_observation index
  curr_obs = 1-curr_obs;
  next_obs = 1-next_obs;
  lmap_count++;
  // And now we compress to encode for Wifi transmission, etc.
  unsigned char cmp_data[MAX_COMPRESSED_DATA_SIZE];
 #ifdef INT_TIME
  gettimeofday(&start_pd_lz4_cmp, NULL);
 #endif	
  int n_cmp_bytes = LZ4_compress_default((char*)local_map, (char*)cmp_data, MAX_UNCOMPRESSED_DATA_SIZE, MAX_COMPRESSED_DATA_SIZE);
 #ifdef INT_TIME
  gettimeofday(&stop_pd_lz4_cmp, NULL);
  pd_lz4_cmp_sec   += stop_pd_lz4_cmp.tv_sec  - start_pd_lz4_cmp.tv_sec;
  pd_lz4_cmp_usec  += stop_pd_lz4_cmp.tv_usec - start_pd_lz4_cmp.tv_usec;
 #endif
  DBGOUT(double c_ratio = 100*(1-((double)(n_cmp_bytes)/(double)(MAX_UNCOMPRESSED_DATA_SIZE)));
	 printf("  Back from LZ4_compress_default: %lu bytes -> %u bytes for %5.2f%%\n", MAX_UNCOMPRESSED_DATA_SIZE, n_cmp_bytes, c_ratio););

  // Now we encode and transmit the grid...
  DBGOUT(printf("Calling do_xmit_pipeline for %u compressed grid elements\n", n_cmp_bytes));
  int n_xmit_out;
  float xmit_out_real[MAX_XMIT_OUTPUTS];
  float xmit_out_imag[MAX_XMIT_OUTPUTS];
 #ifdef INT_TIME
  gettimeofday(&start_pd_wifi_pipe, NULL);
 #endif
  do_xmit_pipeline(n_cmp_bytes, cmp_data, &n_xmit_out, xmit_out_real, xmit_out_imag);
 #ifdef INT_TIME
  gettimeofday(&stop_pd_wifi_pipe, NULL);
  pd_wifi_pipe_sec   += stop_pd_wifi_pipe.tv_sec  - start_pd_wifi_pipe.tv_sec;
  pd_wifi_pipe_usec  += stop_pd_wifi_pipe.tv_usec - start_pd_wifi_pipe.tv_usec;
 #endif
  DBGOUT(printf("  Back from do_xmit_pipeline with %u xmit outputs...\n", n_xmit_out));

  // This is now the content that should be sent out via IEEE 802.11p WiFi
  //  The n_xmit_out values of xmit_out_real and xmit_out_imag
  // Connect to the Wifi-Socket and send the n_xmit_out
  char w_buffer[10];
 #ifdef INT_TIME
  gettimeofday(&start_pd_wifi_send, NULL);
 #endif	
  unsigned xfer_bytes = n_xmit_out*sizeof(float);
  snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
  DBGOUT(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
  send(xmit_sock, w_buffer, 8, 0);
  DBGOUT(printf("     Send %u REAL values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes, XMIT_PORT));
  DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE REAL raw bytes\n", xmit_count);
	  for (int i = 0; i < n_xmit_out; i++) {
	    printf("XFER %4u REAL-byte %6u : %f\n", xmit_count, i, xmit_out_real[i]);
	  }
	  printf("\n"));
 #ifdef INT_TIME
  gettimeofday(&start_pd_wifi_send_rl, NULL);
 #endif	
  send(xmit_sock, (char*)(xmit_out_real), n_xmit_out*sizeof(float), 0);
  DBGOUT(printf("     Send %u IMAG values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes, XMIT_PORT));
  DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE IMAG raw bytes\n", xmit_count);
	  for (int i = 0; i < n_xmit_out; i++) {
	    printf("XFER %4u IMAG-byte %6u : %f\n", xmit_count, i, xmit_out_imag[i]);
	  }
	  printf("\n"));
 #ifdef INT_TIME
  gettimeofday(&stop_pd_wifi_send_rl, NULL);
 #endif
  send(xmit_sock, (char*)(xmit_out_imag), n_xmit_out*sizeof(float), 0);
 #ifdef INT_TIME
  gettimeofday(&stop_pd_wifi_send, NULL);
  pd_wifi_send_sec   += stop_pd_wifi_send.tv_sec  - start_pd_wifi_send.tv_sec;
  pd_wifi_send_usec  += stop_pd_wifi_send.tv_usec - start_pd_wifi_send.tv_usec;
  pd_wifi_send_rl_sec   += stop_pd_wifi_send_rl.tv_sec  - start_pd_wifi_send_rl.tv_sec;
  pd_wifi_send_rl_usec  += stop_pd_wifi_send_rl.tv_usec - start_pd_wifi_send_rl.tv_usec;
  pd_wifi_send_im_sec   += stop_pd_wifi_send.tv_sec  - stop_pd_wifi_send_rl.tv_sec;
  pd_wifi_send_im_usec  += stop_pd_wifi_send.tv_usec - stop_pd_wifi_send_rl.tv_usec;
 #endif

  xmit_count++;
 #if PARALLEL_PTHREADS
  ; // nothing to do here...
 #else
  //receive_and_fuse_maps(NULL);
 #endif
  DBGOUT(printf("Returning from process_lidat_to_occgrid\n"); fflush(stdout));
}

int main(int argc, char *argv[])
{
  struct sockaddr_in bag_servaddr;
  struct sockaddr_in xmit_servaddr;
  struct sockaddr_in recv_servaddr;
  struct sockaddr_in car_servaddr;
  unsigned char l_buffer[20] = {0};
  //unsigned char buffer[200002] = {0};

  lidar_inputs_t lidar_inputs;
    
  snprintf(bag_inet_addr_str, 20, "127.0.0.1");
  snprintf(wifi_inet_addr_str, 20, "127.0.0.1");
  snprintf(car_inet_addr_str, 20, "127.0.0.1");

 #ifdef HW_VIT
  DEBUG(printf("Calling init_VIT_HW_ACCEL...\n"));
  init_VIT_HW_ACCEL();
 #endif
  printf("Initializing the OccGrid state...\n");
  init_occgrid_state(); // Initialize the occgrid functions, state, etc.
  printf("Initializing the Transmit pipeline...\n");
  xmit_pipe_init(); // Initialize the IEEE SDR Transmit Pipeline
  printf("Initializing the Receive pipeline...\n");
  recv_pipe_init();

  signal(SIGINT, INThandler);
  signal(SIGPIPE, SIGPIPE_handler);

  // Use getopt to read in run-time options
  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  int opt;
  while((opt = getopt(argc, argv, ":hB:W:C:s:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 'B':
      snprintf(bag_inet_addr_str, 20, "%s", optarg);
      break;
    case 'W':
      snprintf(wifi_inet_addr_str, 20, "%s", optarg);
      break;
    case 'C':
      snprintf(car_inet_addr_str, 20, "%s", optarg);
      break;
    case 's':
      max_time_steps = atoi(optarg);
      DBGOUT(printf("Set maximum time steps to %u\n", max_time_steps));
      break;

    case ':':
      printf("option needs a value\n");
      break;
    case '?':
      printf("unknown option: %c\n", optopt);
      break;
    }
  }

  if (max_time_steps != ~1) {
    printf("Limiting to %u max time steps\n", max_time_steps);
  } else {
    printf("Running the entire bag file.\n");
  }

  printf("Connecting to bag-server at IP %s PORT %u\n", bag_inet_addr_str, BAG_PORT);
  // Open and connect to the BAG_SERVER 
  if ((bag_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
    printf("BAG Socket creation failed...\n");
    exit(0);
  }
  else {
    printf("BAG Socket successfully created..\n");
  }

  bag_servaddr.sin_family = AF_INET;
  bag_servaddr.sin_addr.s_addr = inet_addr(bag_inet_addr_str);
  bag_servaddr.sin_port = htons(BAG_PORT);

  while (true) {
    if (connect(bag_sock, (struct sockaddr*)&bag_servaddr, sizeof(bag_servaddr)) != 0) {
      printf("connection with the BAG server failed...\n");
      sleep(1);
      continue;
    }
    else {
      printf("connected to the BAG server..\n");
      break;
    }
  }

  // Open and connect to the XMIT_SERVER
  /*
  printf("Connecting to xmit-server at IP %s PORT %u\n", wifi_inet_addr_str, XMIT_PORT);
  if ((xmit_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
    printf("WIFI XMIT Socket creation failed...\n");
    exit(0);
  }
  else {
    printf("WIFI XMIT Socket successfully created..\n");
  }

  xmit_servaddr.sin_family = AF_INET;
  xmit_servaddr.sin_addr.s_addr = inet_addr(wifi_inet_addr_str);
  xmit_servaddr.sin_port = htons(XMIT_PORT);

  while (true) {
    if (connect(xmit_sock, (struct sockaddr*)&xmit_servaddr, sizeof(xmit_servaddr)) != 0) {
      printf("connection with the WIFI XMIT server failed...\n");
      sleep(1);
      continue;
    }
    else {
      printf("connected to the WIFI XMIT server..\n");
      break;
    }
  }

  // Open and connect to the RECV_SERVER 
  printf("Connecting to recv-server at IP %s PORT %u\n", wifi_inet_addr_str, RECV_PORT);
  if ((recv_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
    printf("WIFI RECV Socket creation failed...\n");
    exit(0);
  }
  else {
    printf("WIFI RECV Socket successfully created..\n");
  }

  recv_servaddr.sin_family = AF_INET;
  recv_servaddr.sin_addr.s_addr = inet_addr(wifi_inet_addr_str); //"127.0.0.1");
  recv_servaddr.sin_port = htons(RECV_PORT);

  while (true) {
    if (connect(recv_sock, (struct sockaddr*)&recv_servaddr, sizeof(recv_servaddr)) != 0) {
      printf("connection with the WIFI RECV server failed...\n");
      sleep(1);
      continue;
    }
    else {
      printf("connected to the WIFI RECV server..\n");
      break;
    }
  }

  printf("Connecting to car-server at IP %s PORT %u\n", car_inet_addr_str, CAR_PORT);
  // Open and connect to the CAR_SERVER 
  if ((car_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
    printf("CAR Socket creation failed...\n");
    exit(0);
  }
  else {
    printf("CAR Socket successfully created..\n");
  }

  car_servaddr.sin_family = AF_INET;
  car_servaddr.sin_addr.s_addr = inet_addr(car_inet_addr_str);
  car_servaddr.sin_port = htons(CAR_PORT);

  while (true) {
    if (connect(car_sock, (struct sockaddr*)&car_servaddr, sizeof(car_servaddr)) != 0) {
      printf("connection with the CAR server failed...\n");
      sleep(1);
      continue;
    }
    else {
      printf("connected to the CAR server..\n");
      break;
    }
  }
  */
 #if PARALLEL_PTHREADS
  // Now set up the processing threads - 1 for Lidar input, one for WiFi RECV processing (fusion)
  pthread_attr_t  pt_attr;
  pthread_attr_init(&pt_attr);
  //pthread_t form_occmap_thread;
  pthread_t fuse_occmap_thread;
  /* int pt_ret = pthread_create(&form_occmap_thread, &pt_attr, process_lidar_to_occgrid, NULL); */
  /* if (pt_ret != 0) { */
  /*   printf("Could not start the scheduler pthread... return value %d\n", pt_ret); */
  /*   closeout_and_exit(-1); */
  /* } */
  int pt_ret = pthread_create(&fuse_occmap_thread, &pt_attr, receive_and_fuse_maps, NULL);
  if (pt_ret != 0) {
    printf("Could not start the scheduler pthread... return value %d\n", pt_ret);
    closeout_and_exit("Couldn't allocate fuse_occmap_thread...", -1);
  }
 #endif
  //#ifdef INT_TIME
  gettimeofday(&start_prog, NULL);
  //#endif
  bool hit_eof = false;
  while ((!hit_eof) && (lidar_count < max_time_steps)) {
    DBGOUT(printf("Calling read_all on the BAG socket...\n"); fflush(stdout));
    //int valread = read(bag_sock , l_buffer, 10);
    //#ifdef INT_TIME
    gettimeofday(&start_proc_rdbag, NULL);
    //#endif
    int valread = read_all(bag_sock, l_buffer, 10);
    //#ifdef INT_TIME
    gettimeofday(&stop_proc_rdbag, NULL);
    proc_rdbag_sec   += stop_proc_rdbag.tv_sec  - start_proc_rdbag.tv_sec;
    proc_rdbag_usec  += stop_proc_rdbag.tv_usec - start_proc_rdbag.tv_usec;
    //#endif
    DBGOUT(printf("Top: read %d bytes\n", valread));
    printf("Top: read %d bytes\n", valread);
    if (valread < 10) {
      if (valread == 0) {
	// Zero bytes out implies we hit EOF
	hit_eof = true;
      } else {
	printf("  TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, 10);
	closeout_and_exit("Top read got too few bytes...", -1);
      }
    }

    if(l_buffer[0] == 'L' && l_buffer[9] == 'L') {
      //#ifdef INT_TIME
      gettimeofday(&start_proc_lidar, NULL);
      //#endif
      char * ptr;
      int message_size = strtol(l_buffer+1, &ptr, 10);
      DBGOUT(printf("Lidar: expecting message size: %d\n", message_size));
      printf("Lidar: expecting message size: %d\n", message_size);
      send(bag_sock, ack, 2, 0);
      printf("Lidar: expecting message size: %d\n", message_size);
      int total_bytes_read = read_all(bag_sock, lidar_inputs.data, message_size);
      if (total_bytes_read < message_size) {
	if (total_bytes_read == 0) {
	  printf("  Lidar read got ZERO bytes -- END of TRANSFER?\n");
	  closeout_and_exit("Lidar read got zero bytes...", -1);
	} else {
	  printf("  Lidar read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", total_bytes_read, message_size);
	  closeout_and_exit("Lidar read got too few bytes...", -1);
	}
      }
      if (total_bytes_read > message_size) {
	printf("NOTE: read more total bytes than expected: %u vs %u\n", total_bytes_read, message_size);
      }
      lidar_inputs.odometry[0] = odometry[0];
      lidar_inputs.odometry[1] = odometry[1];
      lidar_inputs.odometry[2] = odometry[2];
      lidar_inputs.data_size = total_bytes_read;
      DBGOUT(printf("Calling process_lidar_to_occgrid for %d total bytes\n", total_bytes_read));
      printf("Processing Lidar msg %4u data\n", lidar_count);
      //#ifdef INT_TIME
      gettimeofday(&start_proc_data, NULL);
      //#endif
      printf("Back from process_lidar_to_occgrid for Lidar\n");
      process_lidar_to_occgrid((void*)(&lidar_inputs)); // buffer, total_bytes_read);
      DBGOUT(printf("Back from process_lidar_to_occgrid for Lidar\n"); fflush(stdout));
      printf("Back from process_lidar_to_occgrid for Lidar\n");
      lidar_count++;
      //#ifdef INT_TIME
      gettimeofday(&stop_proc_lidar, NULL);
      proc_data_sec   += stop_proc_lidar.tv_sec  - start_proc_data.tv_sec;
      proc_data_usec  += stop_proc_lidar.tv_usec - start_proc_data.tv_usec;
      proc_lidar_sec  += stop_proc_lidar.tv_sec  - start_proc_lidar.tv_sec;
      proc_lidar_usec += stop_proc_lidar.tv_usec - start_proc_lidar.tv_usec;
      //#endif
    }
    else if(l_buffer[0] == 'O' && l_buffer[9] == 'O') {
     #ifdef INT_TIME
      gettimeofday(&start_proc_odo, NULL);
     #endif
      char * ptr;
      int message_size = strtol(l_buffer+1, &ptr, 10);
      DBGOUT(printf("Odometry: expecting message size: %d\n", message_size));
      printf("AAA\n");
      send(bag_sock, ack, 2, 0);
      printf("BBB\n");

      int total_bytes_read = 0;

      //valread = read(bag_sock , buffer, 10000);
      printf("Message size: %d\n", message_size);
      valread = read_all(bag_sock , l_buffer, message_size);
      DBGOUT(printf("read %d bytes\n", valread));
      printf("CCC\n");
      if (valread < message_size) {
	if (valread == 0) {
	  printf("  Odo read got ZERO bytes -- END of TRANSFER?\n");
	  closeout_and_exit("Odometry read got zero bytes...", -1);
	} else {
	  printf("  Odo read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, message_size);
	  closeout_and_exit("Odometry read got too few bytes...", -1);
	}
      }

      odometry[0] = *((float*)(l_buffer));
      odometry[1] = *((float*)(l_buffer+4));
      odometry[2] = *((float*)(l_buffer+8));
      printf("Odometry msg %4u: %.2f %.2f %.2f\n", odo_count, odometry[0], odometry[1], odometry[2]);
      odo_count++;
     #ifdef INT_TIME
      gettimeofday(&stop_proc_odo, NULL);
      proc_odo_sec  += stop_proc_odo.tv_sec  - start_proc_odo.tv_sec;
      proc_odo_usec += stop_proc_odo.tv_usec - start_proc_odo.tv_usec;
     #endif
    } else {
      /*DBGOUT(printf("BUFFER : '");
	for (int ii = 0; ii < 8; ii++) {
	printf("%c", l_buffer[ii]);
	}
	printf("'\n");
	fflush(stdout));*/
    }

  }

  dump_final_run_statistics();
  close(bag_sock);
  close(xmit_sock);
  close(recv_sock);
}





void dump_final_run_statistics()
{
  printf("\nFinal Run Stats, %u, Odo, %u, Lidar, %u, LMAP, %u, XMIT, %u, RECV, %u, CAR-SEND\n", odo_count, lidar_count, lmap_count, xmit_count, recv_count, car_send_count);
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
  uint64_t total_exec = (uint64_t)(stop_prog.tv_sec - start_prog.tv_sec) * 1000000 + (uint64_t)(stop_prog.tv_usec - start_prog.tv_usec);
  uint64_t proc_rdbag = (uint64_t)(proc_rdbag_sec)  * 1000000 + (uint64_t)(proc_rdbag_usec);
  uint64_t proc_odo   = (uint64_t)(proc_odo_sec)  * 1000000 + (uint64_t)(proc_odo_usec);
  uint64_t proc_lidar = (uint64_t)(proc_lidar_sec)  * 1000000 + (uint64_t)(proc_lidar_usec);
  uint64_t proc_data  = (uint64_t)(proc_data_sec)  * 1000000 + (uint64_t)(proc_data_usec);

 #ifdef INT_TIME
  uint64_t pd_cloud2grid = (uint64_t)(pd_cloud2grid_sec)  * 1000000 + (uint64_t)(pd_cloud2grid_usec);
  uint64_t pd_lz4_cmp    = (uint64_t)(pd_lz4_cmp_sec)  * 1000000 + (uint64_t)(pd_lz4_cmp_usec);
  uint64_t pd_wifi_pipe  = (uint64_t)(pd_wifi_pipe_sec)  * 1000000 + (uint64_t)(pd_wifi_pipe_usec);
  uint64_t pd_wifi_send  = (uint64_t)(pd_wifi_send_sec)  * 1000000 + (uint64_t)(pd_wifi_send_usec);
  uint64_t pd_wifi_send_rl = (uint64_t)(pd_wifi_send_rl_sec) * 1000000 + (uint64_t)(pd_wifi_send_rl_usec);
  uint64_t pd_wifi_send_im = (uint64_t)(pd_wifi_send_im_sec) * 1000000 + (uint64_t)(pd_wifi_send_im_usec);
  uint64_t pd_wifi_recv_th   = (uint64_t)(pd_wifi_recv_th_sec)  * 1000000 + (uint64_t)(pd_wifi_recv_th_usec);
  uint64_t pd_wifi_lmap_wait = (uint64_t)(pd_wifi_lmap_wait_sec)  * 1000000 + (uint64_t)(pd_wifi_lmap_wait_usec);
  uint64_t pd_wifi_recv_wait = (uint64_t)(pd_wifi_recv_wait_sec)  * 1000000 + (uint64_t)(pd_wifi_recv_wait_usec);
  uint64_t pd_wifi_recv_all  = (uint64_t)(pd_wifi_recv_all_sec)  * 1000000 + (uint64_t)(pd_wifi_recv_all_usec);
  uint64_t pd_wifi_recv_rl = (uint64_t)(pd_wifi_recv_rl_sec) * 1000000 + (uint64_t)(pd_wifi_recv_rl_usec);
  uint64_t pd_wifi_recv_im = (uint64_t)(pd_wifi_recv_im_sec) * 1000000 + (uint64_t)(pd_wifi_recv_im_usec);
  uint64_t pd_recv_pipe  = (uint64_t)(pd_recv_pipe_sec)  * 1000000 + (uint64_t)(pd_recv_pipe_usec);
  uint64_t pd_lz4_uncmp  = (uint64_t)(pd_lz4_uncmp_sec)  * 1000000 + (uint64_t)(pd_lz4_uncmp_usec);
  uint64_t pd_combGrids  = (uint64_t)(pd_combGrids_sec)  * 1000000 + (uint64_t)(pd_combGrids_usec);
  uint64_t pd_carSend    = (uint64_t)(pd_wifi_car_sec)  * 1000000 + (uint64_t)(pd_wifi_car_usec);

  // This is the cloud2grid breakdown
  uint64_t ocgr_cl2g_total   = (uint64_t)(ocgr_c2g_total_sec)   * 1000000 + (uint64_t)(ocgr_c2g_total_usec);
  uint64_t ocgr_cl2g_initCM  = (uint64_t)(ocgr_c2g_initCM_sec)  * 1000000 + (uint64_t)(ocgr_c2g_initCM_usec);
  uint64_t ocgr_cl2g_updOrig = (uint64_t)(ocgr_c2g_updOrig_sec) * 1000000 + (uint64_t)(ocgr_c2g_updOrig_usec);
  uint64_t ocgr_cl2g_updBnds = (uint64_t)(ocgr_c2g_updBnds_sec) * 1000000 + (uint64_t)(ocgr_c2g_updBnds_usec);

  uint64_t ocgr_upBd_total   = (uint64_t)(ocgr_upBd_total_sec)   * 1000000 + (uint64_t)(ocgr_upBd_total_usec);
  uint64_t ocgr_upBd_rayFSp  = (uint64_t)(ocgr_upBd_rayFSp_sec)  * 1000000 + (uint64_t)(ocgr_upBd_rayFSp_usec);
  uint64_t ocgr_upBd_regObst = (uint64_t)(ocgr_upBd_regObst_sec) * 1000000 + (uint64_t)(ocgr_upBd_regObst_usec);

  /** No need to do this here -- provides no more info than exterior measure, really
      uint64_t ocgr_ryFS_total  = (uint64_t)(ocgr_ryFS_total_sec)  * 1000000 + (uint64_t)(ocgr_ryFS_total_usec);
      uint64_t ocgr_ryFS_rtLine = (uint64_t)(ocgr_ryFS_rtLine_sec) * 1000000 + (uint64_t)(ocgr_ryFS_rtLine_usec); **/

  // This is the xmit_pipe.c breakdown
  uint64_t x_pipe      = (uint64_t)(x_pipe_sec)  * 1000000 + (uint64_t)(x_pipe_usec);
  uint64_t x_genmacfr  = (uint64_t)(x_genmacfr_sec)  * 1000000 + (uint64_t)(x_genmacfr_usec);
  uint64_t x_domapwk   = (uint64_t)(x_domapwk_sec)  * 1000000 + (uint64_t)(x_domapwk_usec);
  uint64_t x_phdrgen   = (uint64_t)(x_phdrgen_sec)  * 1000000 + (uint64_t)(x_phdrgen_usec);
  uint64_t x_ck2sym    = (uint64_t)(x_ck2sym_sec)  * 1000000 + (uint64_t)(x_ck2sym_usec);
  uint64_t x_ocaralloc = (uint64_t)(x_ocaralloc_sec)  * 1000000 + (uint64_t)(x_ocaralloc_usec);
  uint64_t x_fft       = (uint64_t)(x_fft_sec)  * 1000000 + (uint64_t)(x_fft_usec);
  uint64_t x_ocycpref  = (uint64_t)(x_ocycpref_sec)  * 1000000 + (uint64_t)(x_ocycpref_usec);

#ifdef XMIT_HW_FFT
  uint64_t x_fHtotal   = (uint64_t)(x_fHtotal_sec)  * 1000000 + (uint64_t)(x_fHtotal_usec);
  uint64_t x_fHcvtin   = (uint64_t)(x_fHcvtin_sec)  * 1000000 + (uint64_t)(x_fHcvtin_usec);
  uint64_t x_fHcomp    = (uint64_t)(x_fHcomp_sec)   * 1000000 + (uint64_t)(x_fHcomp_usec);
  uint64_t x_fHcvtout  = (uint64_t)(x_fHcvtout_sec)  * 1000000 + (uint64_t)(x_fHcvtout_usec);
#endif

  // This is the Xmit doMapWork breakdown
  uint64_t xdmw_total   = (uint64_t)(xdmw_total_sec)   * 1000000 + (uint64_t)(xdmw_total_usec);
  uint64_t xdmw_cnvEnc  = (uint64_t)(xdmw_cnvEnc_sec)  * 1000000 + (uint64_t)(xdmw_cnvEnc_usec);
  uint64_t xdmw_punct   = (uint64_t)(xdmw_punct_sec)   * 1000000 + (uint64_t)(xdmw_punct_usec);
  uint64_t xdmw_intlv   = (uint64_t)(xdmw_intlv_sec)   * 1000000 + (uint64_t)(xdmw_intlv_usec);
  uint64_t xdmw_symbols = (uint64_t)(xdmw_symbls_sec)  * 1000000 + (uint64_t)(xdmw_symbls_usec);
  uint64_t xdmw_mapout  = (uint64_t)(xdmw_mapout_sec)  * 1000000 + (uint64_t)(xdmw_mapout_usec);

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
#endif

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
#endif  
  printf(" Total workload main-loop : %10lu usec\n", total_exec);
  printf("   Total proc Read-Bag      : %10lu usec\n", proc_rdbag);
  printf("   Total proc Odometry      : %10lu usec\n", proc_odo);
  printf("   Total proc Lidar         : %10lu usec\n", proc_lidar);
  printf("     Total proc Data          : %10lu usec\n", proc_data);
#ifdef INT_TIME
  printf("       Total pd cloud2grid      : %10lu usec\n", pd_cloud2grid);
  printf("         Ocgr_Cl2gr Total Time         : %10lu usec\n", ocgr_cl2g_total);
  printf("         Ocgr_Cl2gr InitCM Time        : %10lu usec\n", ocgr_cl2g_initCM);
  printf("         Ocgr_Cl2gr Upd-Origin Time    : %10lu usec\n", ocgr_cl2g_updOrig);
  printf("         Ocgr_Cl2gr Upd-Bounds Time    : %10lu usec\n", ocgr_cl2g_updBnds);
  printf("           Ocgr_UpBnds Total Time        : %10lu usec\n", ocgr_upBd_total);
  printf("           Ocgr_UpBnds Ray_FreeSp Time   : %10lu usec\n", ocgr_upBd_rayFSp);
  /** No need to do this here -- provides no more info than exterior measure, really
      printf("             Ocgr_RyFSp Total Time         : %10lu usec\n", ocgr_ryFS_total);
      printf("             Ocgr_RyFSp RayTrace-Line      : %10lu usec\n", ocgr_ryFS_rtLine); **/
  printf("       Total pd lz4_cmp         : %10lu usec\n", pd_lz4_cmp);
  printf("       Total pd xmit_pipe       : %10lu usec\n", pd_wifi_pipe);
  printf("         X-Pipe Total Time        : %10lu usec\n", x_pipe);
  printf("         X-Pipe GenMacFr Time     : %10lu usec\n", x_genmacfr);
  printf("         X-Pipe doMapWk Time      : %10lu usec\n", x_domapwk);
  printf("           XdoMW Total Time         : %10lu usec\n", xdmw_total);
  printf("           XdoMW ConvEncode Time    : %10lu usec\n", xdmw_cnvEnc);
  printf("           XdoMW Puncture Time      : %10lu usec\n", xdmw_punct);
  printf("           XdoMW Interleave Time    : %10lu usec\n", xdmw_intlv);
  printf("           XdoMW Gen-Symbols Time   : %10lu usec\n", xdmw_symbols);
  printf("           XdoMW Gen-Map-Out Time   : %10lu usec\n", xdmw_mapout);
  printf("         X-Pipe PckHdrGen Time    : %10lu usec\n", x_phdrgen);
  printf("         X-Pipe Chnk2Sym Time     : %10lu usec\n", x_ck2sym);
  printf("         X-Pipe CarAlloc Time     : %10lu usec\n", x_ocaralloc);
  printf("         X-Pipe Xm-FFT Time       : %10lu usec\n", x_fft);
#ifdef XMIT_HW_FFT
  printf("           X-Pipe xHfft_total Time  : %10lu usec\n", x_fHtotal);
  printf("           X-Pipe xHfft_cvtin Time  : %10lu usec\n", x_fHcvtin);
  printf("           X-Pipe xHfft_comp  Time  : %10lu usec\n", x_fHcomp);
  printf("           X-Pipe xHfft_cvtout Time : %10lu usec\n", x_fHcvtout);
#endif
  printf("         X-Pipe CycPrefix Time    : %10lu usec\n", x_ocycpref);
  printf("       Total pd xmit_send       : %10lu usec\n", pd_wifi_send);
  printf("         Total pd xmit_send_rl    : %10lu usec\n", pd_wifi_send_rl);
  printf("         Total pd xmit_send_im    : %10lu usec\n", pd_wifi_send_im);
  printf("       Total pd xmit_recv_th    : %10lu usec\n", pd_wifi_recv_th);
  printf("         Total pd xmit_lmap_wait  : %10lu usec\n", pd_wifi_lmap_wait);
  printf("         Total pd xmit_recv_wait  : %10lu usec\n", pd_wifi_recv_wait);
  printf("         Total pd xmit_recv_all   : %10lu usec\n", pd_wifi_recv_all);
  printf("           Total pd xmit_recv_rl    : %10lu usec\n", pd_wifi_recv_rl);
  printf("           Total pd xmit_recv_im    : %10lu usec\n", pd_wifi_recv_im);
  printf("       Total pd recv_pipe       : %10lu usec\n", pd_recv_pipe);
  printf("         R-Pipe Total Time        : %10lu usec\n", r_pipe);
  printf("         R-Pipe CmplCnjg Time     : %10lu usec\n", r_cmpcnj);
  printf("         R-Pipe CmplMult Time     : %10lu usec\n", r_cmpmpy);
  printf("         R-Pipe FIRC Time         : %10lu usec\n", r_firc);
  printf("         R-Pipe CmplMag Time      : %10lu usec\n", r_cmpmag);
  printf("         R-Pipe CmplMag^2 Time    : %10lu usec\n", r_cmpmag2);
  printf("         R-Pipe FIR Time          : %10lu usec\n", r_fir);
  printf("         R-Pipe DIV Time          : %10lu usec\n", r_div);
  printf("         R-Pipe SyncShort Time    : %10lu usec\n", r_sshort);
  printf("           R-SySht Total Time         : %10lu usec\n", rssh_total);
  printf("           R-SySht Search Time        : %10lu usec\n", rssh_search);
  printf("           R-SySht Frame Time         : %10lu usec\n", rssh_frame);
  printf("         R-Pipe SyncLong Time     : %10lu usec\n", r_slong);
  printf("           R-SyLng Total Time         : %10lu usec\n", rslg_total);
  printf("           R-SyLng FIR-G Time         : %10lu usec\n", rslg_firG);
  printf("           R-SyLng Search Time        : %10lu usec\n", rslg_search);
  printf("           R-SyLng OutGen Time        : %10lu usec\n", rslg_outgen);
  printf("         R-Pipe Rc-FFT Time       : %10lu usec\n", r_fft);
#ifdef RECV_HW_FFT
  printf("           R-Pipe rHfft_total Time  : %10lu usec\n", r_fHtotal);
  printf("           R-Pipe rHfft_cvtin Time  : %10lu usec\n", r_fHcvtin);
  printf("           R-Pipe rHfft_comp  Time  : %10lu usec\n", r_fHcomp);
  printf("           R-Pipe rHfft_cvtout Time : %10lu usec\n", r_fHcvtout);
#endif
  printf("         R-Pipe Equalize Time     :  %10lu usec\n", r_eqlz);
  printf("           R-Eql Total Time         : %10lu usec\n", reql_total);
  printf("           R-Eql Set-Symbol Time    : %10lu usec\n", reql_sym_set);
  printf("           R-Eql LS-EQ Time         : %10lu usec\n", reql_ls_eql);
  printf("           R-Eql Output-Sym Time    : %10lu usec\n", reql_out_sym);
  printf("           R-Eql DecSigFld Time     : %10lu usec\n", reql_ds_fld);
  printf("         R-Pipe DecSignal Time    : %10lu usec\n", r_decsignl);
  printf("           R-Dec Total Time         : %10lu usec\n", rdec_total);
  printf("           R-Dec Map-BitR Time      : %10lu usec\n", rdec_map_bitr);
  printf("           R-Dec Get-Bits Time      : %10lu usec\n", rdec_get_bits);
  printf("           R-Dec Decode Call        : %10lu usec\n", rdec_dec_call);
  printf("         R-Pipe DeScramble Time   : %10lu usec\n", r_descrmbl);
  printf("       Total pd lz4_uncmp       : %10lu usec\n", pd_lz4_uncmp);
  printf("       Total pd combGrids       : %10lu usec\n", pd_combGrids);
  printf("       Total pd carSend         : %10lu usec\n", pd_carSend);
  printf("\n");
 #else
  printf(" NO more detailed timing information on this run...\n");
 #endif
  printf("\nDone with the run...\n");
}
