#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>     // for open
#include <unistd.h>    // for close
#include <arpa/inet.h> // for inet_addr
#include <sys/time.h>
#include <unistd.h>

#define HPVM
// #define HPVM_CV_ROOT
#define HPVM_PROCESS_LIDAR
#define HPVM_PROCESS_LIDAR_INTERNAL
#define HPVM_RECV_PIPELINE
#define RECV_CALLER

#include "globals.h"
#include "debug.h"
#include "getopt.h"
#include "cv_toolset.h"
#include "occgrid.h"   // Occupancy Grid Map Create/Fuse
#include "lz4.h"       // LZ4 Compression/Decompression
#include "xmit_pipe.h" // IEEE 802.11p WiFi SDR Transmit Pipeline
#include "recv_pipe.h" // IEEE 802.11p WiFi SDR Receive Pipeline

#include "globalsRecv.h"
#include "globalsOccgrid.h"
#include "globalsSDRViterbi.h"

#include <stdint.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image/stb_image.h"

#if defined(HPVM)
#include "hpvm.h"
#include "hetero.h"
#endif

// #define CV_PIPELINE
// #define USE_OLD_MODEL

#define PARALLEL_PTHREADS false

#define X86
#define ERA1

#ifdef ERA1
char * IMAGE_FN = "gridimage_era1_";
#define BAG_PORT 5556
#define XMIT_PORT 5558
#define RECV_PORT 5559
#define CAR_PORT 5562
#endif

#ifdef ERA2
char * IMAGE_FN = "gridimage_era2_";
#define BAG_PORT 5557
#define XMIT_PORT 5560
#define RECV_PORT 5561
#define CAR_PORT 5563
#endif

// The PORTS are defined in the compilation process, and comforms to the
// definition in the read_bag_x.py files and wifi_comm_x.py files.

char bag_inet_addr_str[20];
char wifi_inet_addr_str[20];
char car_inet_addr_str[20];
unsigned max_time_steps = ~1;

int bag_sock = 0;
int xmit_sock = 0;
int recv_sock = 0;
int car_sock = 0;

char * ack = "OK";

float odometry[] = {
	0.0,
	0.0,
	0.0 };

// We will define 2 observations; one "current" and one that is to be constructed to be the new current.
int curr_obs = 1;
int next_obs = 0;
Observation observationsArr[2];

// These variables capture "time" spent in various parts ofthe workload
struct timeval stop_prog, start_prog;

struct timeval stop_proc_odo, start_proc_odo;
uint64_t proc_odo_sec = 0LL;
uint64_t proc_odo_usec = 0LL;

struct timeval stop_proc_rdbag, start_proc_rdbag;
uint64_t proc_rdbag_sec = 0LL;
uint64_t proc_rdbag_usec = 0LL;

struct timeval stop_proc_lidar, start_proc_lidar;
uint64_t proc_lidar_sec = 0LL;
uint64_t proc_lidar_usec = 0LL;

struct timeval stop_proc_data, start_proc_data;
uint64_t proc_data_sec = 0LL;
uint64_t proc_data_usec = 0LL;

struct timeval stop_proc_cv, start_proc_cv;
uint64_t proc_cv_sec = 0LL;
uint64_t proc_cv_usec = 0LL;

#ifdef INT_TIME
struct timeval stop_pd_cloud2grid, start_pd_cloud2grid;
uint64_t pd_cloud2grid_sec = 0LL;
uint64_t pd_cloud2grid_usec = 0LL;

struct timeval stop_pd_lz4_cmp, start_pd_lz4_cmp;
uint64_t pd_lz4_cmp_sec = 0LL;
uint64_t pd_lz4_cmp_usec = 0LL;

struct timeval stop_pd_wifi_pipe, start_pd_wifi_pipe;
uint64_t pd_wifi_pipe_sec = 0LL;
uint64_t pd_wifi_pipe_usec = 0LL;

struct timeval stop_pd_wifi_send, start_pd_wifi_send;
uint64_t pd_wifi_send_sec = 0LL;
uint64_t pd_wifi_send_usec = 0LL;

struct timeval stop_pd_wifi_send_rl, start_pd_wifi_send_rl;
uint64_t pd_wifi_send_rl_sec = 0LL;
uint64_t pd_wifi_send_rl_usec = 0LL;

struct timeval stop_pd_wifi_send_im, start_pd_wifi_send_im;
uint64_t pd_wifi_send_im_sec = 0LL;
uint64_t pd_wifi_send_im_usec = 0LL;

struct timeval stop_pd_wifi_recv_th, start_pd_wifi_recv_th;
uint64_t pd_wifi_recv_th_sec = 0LL;
uint64_t pd_wifi_recv_th_usec = 0LL;

struct timeval stop_pd_wifi_lmap_wait, start_pd_wifi_lmap_wait;
uint64_t pd_wifi_lmap_wait_sec = 0LL;
uint64_t pd_wifi_lmap_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_wait, start_pd_wifi_recv_wait;
uint64_t pd_wifi_recv_wait_sec = 0LL;
uint64_t pd_wifi_recv_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_all, start_pd_wifi_recv_all;
uint64_t pd_wifi_recv_all_sec = 0LL;
uint64_t pd_wifi_recv_all_usec = 0LL;

struct timeval stop_pd_wifi_recv_rl, start_pd_wifi_recv_rl;
uint64_t pd_wifi_recv_rl_sec = 0LL;
uint64_t pd_wifi_recv_rl_usec = 0LL;

struct timeval stop_pd_wifi_recv_im, start_pd_wifi_recv_im;
uint64_t pd_wifi_recv_im_sec = 0LL;
uint64_t pd_wifi_recv_im_usec = 0LL;

struct timeval stop_pd_recv_pipe, start_pd_recv_pipe;
uint64_t pd_recv_pipe_sec = 0LL;
uint64_t pd_recv_pipe_usec = 0LL;

struct timeval stop_pd_lz4_uncmp, start_pd_lz4_uncmp;
uint64_t pd_lz4_uncmp_sec = 0LL;
uint64_t pd_lz4_uncmp_usec = 0LL;

struct timeval stop_pd_combGrids, start_pd_combGrids;
uint64_t pd_combGrids_sec = 0LL;
uint64_t pd_combGrids_usec = 0LL;

struct timeval stop_pd_wifi_car, start_pd_wifi_car;
uint64_t pd_wifi_car_sec = 0LL;
uint64_t pd_wifi_car_usec = 0LL;
#endif

int arr_counter = 0;
int ascii_counter = 0;

unsigned odo_count = 0;
unsigned lidar_count = 0;
unsigned lmap_count = 0;
unsigned xmit_count = 0;
unsigned recv_count = 0;
unsigned car_send_count = 0;
unsigned cv_count = 0;

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

void INThandler(int dummy) {
	printf("In SIGINT INThandler -- Closing the connection and exiting\n");
	closeout_and_exit("Received a SIGINT...", -1);
}

void SIGPIPE_handler(int dummy) {
	printf("In SIGPIPE_handler -- Closing the connection and exiting\n");
	closeout_and_exit("Received a SIGPIPE...", -1);
}

#ifdef XMIT_HW_FFT
extern void free_XMIT_FFT_HW_RESOURCES();
#endif
#ifdef RECV_HW_FFT
extern void free_RECV_FFT_HW_RESOURCES();
#endif

// This cleans up the state before exit
void closeout_and_exit(char * last_msg, int rval) {
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

void write_array_to_file(unsigned char * data, long size) {
	const int dimx = 50, dimy = 50;
	int i, j;

	char file_name[32];

	snprintf(file_name, sizeof(char) * 32, "%s%04d.ppm", IMAGE_FN, arr_counter);

	FILE * fp = fopen(file_name, "w");
	fprintf(fp, "P3 %d %d 255\n", dimx, dimy);

	for (j = 0; j < dimy * dimx; ++j) {
		fprintf(fp, " %d %d %d ", data[j], data[j], data[j]);
	}

	fclose(fp);
	arr_counter++;
}

#define MAX_UNCOMPRESSED_DATA_SIZE sizeof(Costmap2D)        // MAX_GRID_SIZE
#define MAX_COMPRESSED_DATA_SIZE MAX_UNCOMPRESSED_DATA_SIZE //(In case of no compression)?

#define MAX_XMIT_OUTPUTS 41800 // Really something like 41782 I think

int read_all(int sock, char * buffer, int xfer_in_bytes) {
	char * ptr;
	int message_size = xfer_in_bytes;
	char * message_ptr = buffer;
	int total_recvd = 0;
	while (total_recvd < message_size) {
		unsigned rem_len = (message_size - total_recvd);
		int valread = read(sock, message_ptr, rem_len);
		message_ptr = message_ptr + valread;
		total_recvd += valread;
		DBGOUT2(printf("        read %d bytes for %d total bytes of %d\n", valread, total_recvd,
			message_size));
		if (valread == 0) {
			DBGOUT(printf("  read_all got ZERO bytes -- END of TRANSFER?\n"));
			return total_recvd;
		}
	}
	return total_recvd;
}

void decompress(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observations, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true 
	void * Section_Inner = __hetero_section_begin();
	void * T2 = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task");

#endif
	// Now we decompress the grid received via transmission...
	DBGOUT(printf("Calling LZ4_decompress_default...\n"));
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_RECV_PIPELINE)) 
	gettimeofday(&start_pd_lz4_uncmp, NULL);
#endif
	DEBUG(printf("Calling LZ4_decompress_safe with %d input bytes...\n", recvd_msg_len));
	*dec_bytes = LZ4_decompress_safe((char *) recvd_msg, (char *) uncmp_data, *recvd_msg_len,
		MAX_UNCOMPRESSED_DATA_SIZE);
	if (*dec_bytes < 0) {
		printf("LZ4_decompress_safe ERROR : %d\n", *dec_bytes);
	}
	DEBUG(
	else {
		printf("LZ4_decompress_safe returned %d bytes\n", *dec_bytes);
	});
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_RECV_PIPELINE)) 
	gettimeofday(&stop_pd_lz4_uncmp, NULL);
	pd_lz4_uncmp_sec += stop_pd_lz4_uncmp.tv_sec - start_pd_lz4_uncmp.tv_sec;
	pd_lz4_uncmp_usec += stop_pd_lz4_uncmp.tv_usec - start_pd_lz4_uncmp.tv_usec;
#endif


	DEBUG(printf("Recevied %d decoded bytes from the wifi...\n", *dec_bytes));
	Costmap2D * remote_map = (Costmap2D *) &(uncmp_data); // Convert "type" to Costmap2D

	DBGOUT(printf("  Back from LZ4_decompress_safe with %u decompressed bytes\n", *dec_bytes);
	printf("  Remote CostMAP: AV x %lf y %lf z %lf\n", remote_map -> av_x, remote_map -> av_y,
		remote_map -> av_z);
	printf("                : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map -> cell_size,
		remote_map -> x_dim, remote_map -> y_dim); print_ascii_costmap(stdout, remote_map));

	// Get the current local-map
	printf("Receive step %u : Processing fusion for curr_obs = %d\n", recv_count, curr_obs);
	Costmap2D * local_map = &(observations[curr_obs].master_costmap);

#ifdef WRITE_ASCII_MAP
	char ascii_file_name[32];
	snprintf(ascii_file_name, sizeof(char) * 32, "%s%04d.txt", ASCII_FN, ascii_counter);
	FILE * ascii_fp = fopen(ascii_file_name, "w");

	// printf("Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
	fprintf(ascii_fp, "Input CostMAP: AV x %lf y %lf z %lf\n", local_map -> av_x, local_map -> av_y,
		local_map -> av_z);
	fprintf(ascii_fp, "             : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map -> cell_size,
		local_map -> x_dim, local_map -> y_dim);
	print_ascii_costmap(ascii_fp, local_map);

	fprintf(ascii_fp, "\n\nRemote CostMAP: AV x %lf y %lf z %lf\n", remote_map -> av_x, remote_map -> av_y,
		remote_map -> av_z);
	// printf("\n\nRemote CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
	fprintf(ascii_fp, "              : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map -> cell_size,
		remote_map -> x_dim, remote_map -> y_dim);
	print_ascii_costmap(ascii_fp, remote_map);
	fclose(ascii_fp);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true 
	__hetero_task_end(T2);
	__hetero_section_end(Section_Inner);
#endif
}


void decompress_Wrapper2(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observation, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T2_Wrapper = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper2");
#endif

	decompress(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz, recvd_msg,
		recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER) && true 
	__hetero_task_end(T2_Wrapper);
	__hetero_section_end(Section_Caller);
#endif

}

void decompress_Wrapper3(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observation, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T2_Wrapper = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper3");
#endif

	decompress_Wrapper2(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz, recvd_msg,
		recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER) && true 
	__hetero_task_end(T2_Wrapper);
	__hetero_section_end(Section_Caller);
#endif

}

void decompress_Wrapper4(unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	Observation * observation, size_t observation_sz
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T2_Wrapper = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper4");
#endif

	decompress_Wrapper3(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz, recvd_msg,
		recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observation, observation_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && defined(RECV_CALLER) && true 
	__hetero_task_end(T2_Wrapper);
	__hetero_section_end(Section_Caller);
#endif
}

void grid_fusion(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true 
	void * Section_Inner = __hetero_section_begin();
	void * T4 = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task");
#endif

	// Then we should "Fuse" the received GridMap with our local one
	//  We need to "peel out" the remote odometry data from somewhere (in the message?)
	// unsigned char* combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
	DBGOUT(printf("\nCalling combineGrids...\n"));
	// Note: The direction in which this is called is slightly significant:
	//  The first map is copied into the second map, in this case remote into local,
	//  and the x_dim, et.c MUST correspond to that second map (here local)
#ifdef INT_TIME
	gettimeofday(&start_pd_combGrids, NULL);
#endif
	// Copied the following two variable above task; this wasn't being modified in the above task so
	// just copying it down should be safe
	Costmap2D * local_map_cp = &(observations[curr_obs].master_costmap);
	Costmap2D * remote_map_cp = (Costmap2D *) (uncmp_data); // Convert "type" to Costmap2D
	fuseIntoLocal(local_map_cp, remote_map_cp);
	/*combineGrids(remote_map->costmap, local_map->costmap,
		remote_map->av_x, remote_map->av_y,
		local_map->av_x, local_map->av_y,
		local_map->x_dim, local_map->y_dim, local_map->cell_size);
		*/
#ifdef INT_TIME
	gettimeofday(&stop_pd_combGrids, NULL);
	pd_combGrids_sec += stop_pd_combGrids.tv_sec - start_pd_combGrids.tv_sec;
	pd_combGrids_usec += stop_pd_combGrids.tv_usec - start_pd_combGrids.tv_usec;
#endif
#ifdef WRITE_ASCII_MAP
	char ascii_file_name[32];
	snprintf(ascii_file_name, sizeof(char) * 32, "%s%04d.txt", ASCII_FN, ascii_counter);
	FILE * ascii_fp = fopen(ascii_file_name, "w");
	DEBUG2(printf(ascii_fp, "  Fused CostMAP : AV x %lf y %lf z %lf\n", local_map_cp -> av_x,
		local_map_cp -> av_y, local_map_cp -> av_z); printf(ascii_fp, "                : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map_cp -> cell_size,
			local_map_cp -> x_dim, local_map_cp -> y_dim); print_ascii_costmap(stdout, local_map_cp));
#endif

#ifdef WRITE_ASCII_MAP
	fprintf(ascii_fp, "\n\nFused CostMAP : AV x %lf y %lf z %lf\n", local_map_cp -> av_x, local_map_cp -> av_y,
		local_map_cp -> av_z);
	// printf("\n\nFused CostMAP : AV x %lf y %lf z %lf\n", local_map_cp->av_x, local_map_cp->av_y, local_map_cp->av_z);
	fprintf(ascii_fp, "              : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map_cp -> cell_size,
		local_map_cp -> x_dim, local_map_cp -> y_dim);
	print_ascii_costmap(ascii_fp, local_map_cp);
	fclose(ascii_fp);
	ascii_counter++;
#endif
	// Write the combined map to a file
#ifdef WRITE_FUSED_MAPS
	write_array_to_file(local_map_cp -> costmap, COST_MAP_ENTRIES);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true 
	__hetero_task_end(T4);
	__hetero_section_end(Section_Inner);
#endif
}

void grid_fusion_Wrapper2(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T4_Caller = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_wrapper2");
#endif

	grid_fusion(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && defined(RECV_CALLER)   && true 
	__hetero_task_end(T4_Caller);
	__hetero_section_end(Section_Caller);
#endif
}

void grid_fusion_Wrapper3(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T4_Caller = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_wrapper3");
#endif

	grid_fusion_Wrapper2(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && defined(RECV_CALLER)   && true 
	__hetero_task_end(T4_Caller);
	__hetero_section_end(Section_Caller);
#endif
}

void grid_fusion_Wrapper4(Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz) {

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && defined(RECV_CALLER)  && true 
	void * Section_Caller = __hetero_section_begin();
	void * T4_Caller = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_wrapper4");
#endif

	grid_fusion_Wrapper3(observations, observations_sz, uncmp_data, uncmp_data_sz);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && defined(RECV_CALLER)   && true 
	__hetero_task_end(T4_Caller);
	__hetero_section_end(Section_Caller);
#endif
}


void fuse_maps(int n_recvd_in,
	float * recvd_in_real, size_t recvd_in_real_sz,
	float * recvd_in_imag, size_t recvd_in_imag_sz,
	int * recvd_msg_len, size_t recvd_msg_len_sz,
	unsigned char * recvd_msg, size_t recvd_msg_sz,
	Observation * observations, size_t observations_sz,
	unsigned char * uncmp_data, size_t uncmp_data_sz,
	int * dec_bytes, size_t dec_bytes_sz,
	// Start variables used by do_recv_pipeline
	//              Local variables used by do_recv_pipeline
	uint8_t * scrambled_msg, size_t scrambled_msg_sz,
	float * ss_freq_offset, size_t ss_freq_offset_sz,
	unsigned * num_sync_short_vals, size_t num_sync_short_vals_sz,
	float * sl_freq_offset, size_t sl_freq_offset_sz,
	unsigned * num_sync_long_vals, size_t num_sync_long_vals_sz,
	fx_pt1 * fft_ar_r, size_t fft_ar_r_sz,
	fx_pt1 * fft_ar_i, size_t fft_ar_i_sz,
	unsigned * num_fft_outs, size_t num_fft_outs_sz,
	fx_pt * toBeEqualized, size_t toBeEqualized_sz,
	fx_pt * equalized, size_t equalized_sz,
	unsigned * num_eq_out_bits, size_t num_eq_out_bits_sz,
	unsigned * psdu, size_t psdu_sz,
	//              Global variables used by do_recv_pipeline
	fx_pt * delay16_out_arg /*= delay16_out -> global*/, size_t delay16_out_arg_sz /*= DELAY_16_MAX_OUT_SIZE*/,
	fx_pt * input_data_arg /*= input_data -> global*/, size_t input_data_sz /*=DELAY_16_MAX_OUT_SIZE - 16*/,
	fx_pt * cmpx_conj_out_arg /*= cmpx_conj_out -> global*/, size_t cmpx_conj_out_arg_sz /*= CMP_CONJ_MAX_SIZE*/,
	fx_pt * cmpx_mult_out_arg /*= cmpx_mult_out -> global*/, size_t cmpx_mult_out_arg_sz /*= CMP_MULT_MAX_SIZE*/,
	fx_pt * correlation_complex_arg /*= correlation_complex -> global*/, size_t correlation_complex_arg_sz /*= FIRC_MAVG48_MAX_SIZE*/,
	fx_pt1 * correlation_arg /*= correlation -> global*/, size_t correlation_arg_sz /*= CMP2MAG_MAX_SIZE*/,
	fx_pt1 * signal_power_arg /*= signal_power -> global*/, size_t signal_power_arg_sz /*= CMP2MAGSQ_MAX_SIZE*/,
	fx_pt1 * avg_signal_power_arg /*= avg_signal_power -> global*/, size_t avg_signal_power_arg_sz /*= FIR_MAVG64_MAX_SIZE*/,
	fx_pt1 * the_correlation_arg /*= the_correlation -> global*/, size_t the_correlation_arg_sz /*= DIVIDE_MAX_SIZE*/,
	fx_pt * sync_short_out_frames_arg /*= sync_short_out_frames -> global*/, size_t sync_short_out_frames_arg_sz /*=320*/,
	fx_pt * d_sync_long_out_frames_arg /*= d_sync_long_out_frames -> global*/, size_t d_sync_long_out_frames_arg_sz /*= SYNC_L_OUT_MAX_SIZE*/,
	// Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        unsigned* num_fft_outs_rcv_fft, size_t num_fft_outs_rcv_fft_sz,
	// 		Local variablse used by decode_signal (task in do_recv_pipeline)
	unsigned* num_dec_bits, size_t num_dec_bits_sz /*= sizeof(unsigned)*/,
	uint8_t* bit_r, size_t bit_r_sz /*= DECODE_IN_SIZE_MAX*/,
	uint8_t* bit, size_t bit_sz /*= DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES*/,
	size_t vit_size,
	ofdm_param* ofdm, size_t ofdm_sz /*= sizeof(ofdm_param)*/,
	frame_param* frame, size_t frame_sz /*= sizeof(frame_param)*/,
	int* n_res_char, size_t n_res_char_sz /*= sizeof(int)*/,
	// 		Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline
	uint8_t* inMemory, size_t inMemory_sz /*= 24852*/,
	uint8_t* outMemory, size_t outMemory_sz /*= 18585*/,
	int* d_ntraceback_arg, size_t d_ntraceback_arg_sz /*= sizeof(int)*/
	// End variables used by do_recv_pipeline
) {
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	void * SectionLoop = __hetero_section_begin();
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	// 38 inputs, 3 outputs
	void * T1 = __hetero_task_begin(39, n_recvd_in, recvd_in_real, recvd_in_real_sz,
		recvd_msg, recvd_msg_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_in_imag, recvd_in_imag_sz,
		// Start variables used by do_recv_pipeline
		//              Local variables used by do_recv_pipeline
		scrambled_msg, scrambled_msg_sz, ss_freq_offset, ss_freq_offset_sz,
		num_sync_short_vals, num_sync_short_vals_sz,
		sl_freq_offset, sl_freq_offset_sz,
		num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz,
		fft_ar_i, fft_ar_i_sz, num_fft_outs, num_fft_outs_sz,
		toBeEqualized, toBeEqualized_sz, equalized, equalized_sz,
		num_eq_out_bits, num_eq_out_bits_sz, psdu, psdu_sz,
		//              Global variables used by do_recv_pipeline
		delay16_out_arg, delay16_out_arg_sz,
		input_data_arg, input_data_sz,
		cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
		cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
		correlation_complex_arg, correlation_complex_arg_sz,
		correlation_arg, correlation_arg_sz,
		signal_power_arg, signal_power_arg_sz,
		avg_signal_power_arg, avg_signal_power_arg_sz,
		the_correlation_arg, the_correlation_arg_sz,
		sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
		d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz,
		// Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        	num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
		// Local variables for decode_signal, a task in do_recv_pipeline
                num_dec_bits, num_dec_bits_sz,
                bit_r, bit_r_sz,
                bit, bit_sz,
                ofdm, ofdm_sz,
								vit_size,
                frame, frame_sz,
                n_res_char, n_res_char_sz,
                // Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline)
                inMemory, inMemory_sz,
                outMemory, outMemory_sz,
                d_ntraceback_arg, d_ntraceback_arg_sz,
		// End variables used by do_recv_pipeline
		3, recvd_msg_len, recvd_msg_len_sz,
		recvd_in_real, recvd_in_real_sz,
		recvd_in_imag, recvd_in_imag_sz,
		"recieve_pipeline_task");
#endif

	// Now we have the tranmission input data to be decoded...
#if !(defined(HPVM) && defined(HPVM_RECV_PIPELINE))
	DBGOUT(printf("Calling do_recv_pipeline...\n"));
#endif
	// Fake this with a "loopback" of the xmit message...

#if defined(INT_TIME) && !(defined(HPVM) ||  defined(HPVM_RECV_PIPELINE))
	gettimeofday(&start_pd_recv_pipe, NULL);
#endif

	do_recv_pipeline(n_recvd_in, recvd_in_real, recvd_in_real_sz, recvd_in_imag, recvd_in_imag_sz,
		recvd_msg_len, recvd_msg_len_sz, (char *) recvd_msg, recvd_msg_sz,
		//              Local variables used by do_recv_pipeline
		scrambled_msg, scrambled_msg_sz, ss_freq_offset, ss_freq_offset_sz,
		num_sync_short_vals, num_sync_short_vals_sz, sl_freq_offset, sl_freq_offset_sz,
		num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz,
		fft_ar_i, fft_ar_i_sz, num_fft_outs, num_fft_outs_sz,
		toBeEqualized, toBeEqualized_sz, equalized, equalized_sz,
		num_eq_out_bits, num_eq_out_bits_sz, psdu, psdu_sz,
		//              Global variables used by do_recv_pipeline
		delay16_out_arg, delay16_out_arg_sz,
		input_data_arg, input_data_sz,
		cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
		cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
		correlation_complex_arg, correlation_complex_arg_sz,
		correlation_arg, correlation_arg_sz,
		signal_power_arg, signal_power_arg_sz,
		avg_signal_power_arg, avg_signal_power_arg_sz,
		the_correlation_arg, the_correlation_arg_sz,
		sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
		d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz,
		// 		Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        	num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
		// 		Local variables for decode_signal, a task in do_recv_pipeline
                num_dec_bits, num_dec_bits_sz,
                bit_r, bit_r_sz,
                bit, bit_sz,
								vit_size,
                ofdm, ofdm_sz,
                frame, frame_sz,
                n_res_char, n_res_char_sz,
                // 		Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline)
                inMemory, inMemory_sz,
                outMemory, outMemory_sz,
                d_ntraceback_arg, d_ntraceback_arg_sz);

#if defined(INT_TIME) && !(defined(HPVM) ||  defined(HPVM_RECV_PIPELINE))
	gettimeofday(&stop_pd_recv_pipe, NULL);
	pd_recv_pipe_sec += stop_pd_recv_pipe.tv_sec - start_pd_recv_pipe.tv_sec;
	pd_recv_pipe_usec += stop_pd_recv_pipe.tv_usec - start_pd_recv_pipe.tv_usec;
#endif
	// do_recv_pipeline(n_xmit_out, xmit_out_real, xmit_out_imag, &recvd_msg_len, recvd_msg);
#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	__hetero_task_end(T1);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	void * T2_Wrapper1 = __hetero_task_begin(5, uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observations_sz,
		2, uncmp_data, uncmp_data_sz, dec_bytes, dec_bytes_sz, "decompress_task_Wrapper1");
#endif

#if defined(RECV_CALLER)
	decompress_Wrapper4(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observations_sz);
#else
	decompress(uncmp_data, uncmp_data_sz, recvd_msg_len, recvd_msg_len_sz,
		recvd_msg, recvd_msg_sz, dec_bytes, dec_bytes_sz,
		observations, observations_sz);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	__hetero_task_end(T2_Wrapper1);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	void * T4_Wrapper1 = __hetero_task_begin(2, observations, observations_sz, uncmp_data, uncmp_data_sz,
		1, observations, observations_sz, "gridFusion_task_Wrapper1");
#endif

#if defined(RECV_CALLER)
	grid_fusion_Wrapper4(observations, observations_sz, uncmp_data, uncmp_data_sz);
#else
	grid_fusion(observations, observations_sz, uncmp_data, uncmp_data_sz);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE))  && true
	__hetero_task_end(T4_Wrapper1);
#endif

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
	// End graph here as we are doing IO in the following section so it should probably not be run in
	// parallel with other sections of the code as it can create race conditions.
	__hetero_section_end(SectionLoop);
#endif
}

/*
 * This function is called by receive_and_fuse_maps after the state of lmap_count changes to 0.
 * This function excuetes all the tasks that are dependent on this change of state,
 */
void * receive_and_fuse_maps_impl(Observation * observations /*=observations -> global*/, size_t observations_sz /*=2*/) {
	// Now we take in a received transmission with the other AV's map
	// If we receive a transmission, the process to turn it back into the gridMap is:
	int n_recvd_in = 0;
	float recvd_in_real[MAX_XMIT_OUTPUTS];
	size_t recvd_in_real_sz = MAX_XMIT_OUTPUTS;
	float recvd_in_imag[MAX_XMIT_OUTPUTS];
	size_t recvd_in_imag_sz = MAX_XMIT_OUTPUTS;

#if PARALLEL_PTHREADS
	while (1) {
#endif
		DBGOUT(printf("\nTrying to Receive data on RECV port %u socket\n", RECV_PORT));
#ifdef INT_TIME
		gettimeofday(&start_pd_wifi_recv_wait, NULL);
#endif
		// The first 8-bytes of the message is a unsigned long (plus some metadata) telling the size of
		// the actual message sent. So, the size is first read into r_buffer

		char r_buffer[10];
		int valread = read_all(recv_sock, r_buffer, 8);
		DBGOUT(printf("  RECV got %d bytes :'%s'\n", valread, r_buffer));
		DBGOUT2(printf("  RECV msg psn %s\n", "01234567890"));

		if (valread == 8) { // Read in 8-bytes; so we read a unsigned long (plus metadata)

			// In the following esction we are sending message through recv_sock. Since this is an IO it most
			// likely should not go in parallel with other tasks
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_recv_all, NULL);
#endif

			if (!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) {
				printf("ERROR: Unexpected message from WiFi...\n");
				closeout_and_exit("Unexpected WiFi message...", -3);
			}
			send(recv_sock, ack, 2, 0);

			// Convert r_butter to unsigned long to find out the actual size of the message
			char * ptr;
			unsigned xfer_in_bytes = strtol(r_buffer + 1, &ptr, 10);
			n_recvd_in = xfer_in_bytes / sizeof(float);

			DBGOUT(printf("     Recv %u REAL values %u bytes from RECV port %u socket\n", n_recvd_in,
				xfer_in_bytes, RECV_PORT));
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_recv_rl, NULL);
#endif

			// Read the actual message (part 1)
			valread = read_all(recv_sock, (char *) recvd_in_real, xfer_in_bytes);
			if (valread < xfer_in_bytes) {
				if (valread == 0) {
					printf("  RECV REAL got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("RECV REAL got zero bytes..", -1);
				}
				else {
					printf("  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread,
						xfer_in_bytes);
					closeout_and_exit("RECV REAL got too few bytes..", -1);
				}
			}
			DBGOUT2(printf("XFER %4u : Dumping RECV-PIPE REAL raw bytes\n", recv_count);
			for (int i = 0; i < n_recvd_in; i++) {
				printf("XFER %4u REAL-byte %6u : %f\n", odo_count, i, recvd_in_real[i]);
			} printf("\n"));

			DBGOUT(printf("     Recv %u IMAG values %u bytes from RECV port %u socket\n", n_recvd_in,
				xfer_in_bytes, RECV_PORT));
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_recv_rl, NULL);
#endif

			// Read the actual message (part 2)
			valread = read_all(recv_sock, (char *) recvd_in_imag, xfer_in_bytes);
			if (valread < xfer_in_bytes) {
				if (valread == 0) {
					printf("  RECV IMAG got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("RECV IMAG got zero bytes..", -1);
				}
				else {
					printf("  RECV IMAG got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
					closeout_and_exit("RECV IMAG got too few bytes..", -1);
				}
			}
			DBGOUT2(printf("XFER %4u : Dumping RECV-PIPE IMAG raw bytes\n", xmit_recv_count);
			for (int i = 0; i < n_recvd_in; i++) {
				printf("XFER %4u IMAG-byte %6u : %f\n", odo_count, i, recvd_in_imag[i]);
			} printf("\n"));
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_recv_all, NULL);
			pd_wifi_recv_wait_sec += start_pd_wifi_recv_all.tv_sec - start_pd_wifi_recv_wait.tv_sec;
			pd_wifi_recv_wait_usec += start_pd_wifi_recv_all.tv_usec - start_pd_wifi_recv_wait.tv_usec;
			pd_wifi_recv_all_sec += stop_pd_wifi_recv_all.tv_sec - start_pd_wifi_recv_all.tv_sec;
			pd_wifi_recv_all_usec += stop_pd_wifi_recv_all.tv_usec - start_pd_wifi_recv_all.tv_usec;
			pd_wifi_recv_rl_sec += stop_pd_wifi_recv_rl.tv_sec - start_pd_wifi_recv_rl.tv_sec;
			pd_wifi_recv_rl_usec += stop_pd_wifi_recv_rl.tv_usec - start_pd_wifi_recv_rl.tv_usec;
			pd_wifi_recv_im_sec += stop_pd_wifi_recv_all.tv_sec - stop_pd_wifi_recv_rl.tv_sec;
			pd_wifi_recv_im_usec += stop_pd_wifi_recv_all.tv_usec - stop_pd_wifi_recv_rl.tv_usec;
#endif

			// Now we start processing all the messages we read in the previous two sections
			// Helper variables
			int recvd_msg_len = 0;
			size_t recvd_msg_len_sz = sizeof(int);
			unsigned char recvd_msg[1500]; // MAX size of original message in bytes
			size_t recvd_msg_sz = 1500;
			unsigned char uncmp_data[MAX_UNCOMPRESSED_DATA_SIZE];
			size_t uncmp_data_sz = MAX_UNCOMPRESSED_DATA_SIZE;
			int dec_bytes = 0;
			size_t dec_bytes_sz = sizeof(int);

			// Variables to be passed into do_recv_pipeline
			uint8_t scrambled_msg[MAX_ENCODED_BITS * 3 / 4];
			size_t scrambled_msg_sz = MAX_ENCODED_BITS * 3 / 4;
			float ss_freq_offset = 0;
			size_t ss_freq_offset_sz = sizeof(float);
			unsigned num_sync_short_vals = 0;
			size_t num_sync_short_vals_sz = sizeof(unsigned);
			float sl_freq_offset = 0;
			size_t sl_freq_offset_sz = sizeof(float);
			unsigned num_sync_long_vals = 0;
			size_t num_sync_long_vals_sz = sizeof(unsigned);
			fx_pt1 fft_ar_r[FRAME_EQ_IN_MAX_SIZE];
			size_t fft_ar_r_sz = FRAME_EQ_IN_MAX_SIZE * sizeof(fx_pt1);
			fx_pt1 fft_ar_i[FRAME_EQ_IN_MAX_SIZE];
			size_t fft_ar_i_sz = FRAME_EQ_IN_MAX_SIZE * sizeof(fx_pt1);
			unsigned num_fft_outs = 0;
			size_t num_fft_outs_sz = sizeof(unsigned);
			fx_pt toBeEqualized[FRAME_EQ_IN_MAX_SIZE];
			size_t toBeEqualized_sz = FRAME_EQ_IN_MAX_SIZE * sizeof(fx_pt);
			fx_pt equalized[FRAME_EQ_OUT_MAX_SIZE];
			size_t equalized_sz = FRAME_EQ_OUT_MAX_SIZE * sizeof(fx_pt);
			unsigned num_eq_out_bits = 0;
			size_t num_eq_out_bits_sz = sizeof(unsigned);
			unsigned psdu = 0;
			size_t psdu_sz = sizeof(unsigned);

			// Local variablse used by decode_signal
			unsigned num_dec_bits = 0; size_t num_dec_bits_sz = sizeof(unsigned);
			size_t bit_r_sz = DECODE_IN_SIZE_MAX;
			uint8_t bit_r[DECODE_IN_SIZE_MAX];
			size_t bit_sz = DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES;
			uint8_t bit[DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES];
			size_t vit_size = 0;
			ofdm_param ofdm; size_t ofdm_sz = sizeof(ofdm_param);
			frame_param frame; size_t frame_sz = sizeof(frame_param);
			int n_res_char = 0; size_t n_res_char_sz = sizeof(int);

			// Local variables for sdr_decode_ofdm
			uint8_t inMemory[24852]; size_t inMemory_sz = 24852;
			uint8_t outMemory[18585]; size_t outMemory_sz = 18585;
			size_t d_ntraceback_arg_sz = sizeof(int);

        		unsigned num_fft_outs_rcv_fft = 0; size_t num_fft_outs_rcv_fft_sz = sizeof(unsigned);

			printf("%s %d Calling fuse_maps", __FILE__, __LINE__);

#if (defined(HPVM) && defined(HPVM_RECV_PIPELINE)) && true
			// 42 inputs, 7 outputs
			void * LaunchInner = __hetero_launch((void *) fuse_maps, 42,
				n_recvd_in,
				recvd_in_real, recvd_in_real_sz,
				recvd_in_imag, recvd_in_imag_sz,
				&recvd_msg_len, recvd_msg_len_sz,
				recvd_msg, recvd_msg_sz,
				observations, observations_sz,
				uncmp_data, uncmp_data_sz,
				&dec_bytes, dec_bytes_sz,
				// Start variables used by do_recv_pipeline
				// 	Local variables used by do_recv_pipeline
				scrambled_msg, scrambled_msg_sz,
				&ss_freq_offset, ss_freq_offset_sz,
				&num_sync_short_vals, num_sync_short_vals_sz,
				&sl_freq_offset, sl_freq_offset_sz,
				&num_sync_long_vals, num_sync_long_vals_sz,
				fft_ar_r, fft_ar_r_sz,
				fft_ar_i, fft_ar_i_sz,
				&num_fft_outs, num_fft_outs_sz,
				toBeEqualized, toBeEqualized_sz,
				equalized, equalized_sz,
				&num_eq_out_bits, num_eq_out_bits_sz,
				&psdu, psdu_sz,
				//      Global variables used by do_recv_pipeline
				delay16_out, DELAY_16_MAX_OUT_SIZE * sizeof(fx_pt),
				input_data, (DELAY_16_MAX_OUT_SIZE - 16) * sizeof(fx_pt),
				cmpx_conj_out, CMP_CONJ_MAX_SIZE * sizeof(fx_pt),
				cmpx_mult_out, CMP_MULT_MAX_SIZE * sizeof(fx_pt),
				correlation_complex, FIRC_MAVG48_MAX_SIZE * sizeof(fx_pt),
				correlation, CMP2MAG_MAX_SIZE * sizeof(fx_pt1),
				signal_power, CMP2MAGSQ_MAX_SIZE * sizeof(fx_pt1),
				avg_signal_power, FIR_MAVG64_MAX_SIZE * sizeof(fx_pt1),
				the_correlation, DIVIDE_MAX_SIZE * sizeof(fx_pt1),
				sync_short_out_frames, 320 * sizeof(fx_pt),
				d_sync_long_out_frames, SYNC_L_OUT_MAX_SIZE * sizeof(fx_pt),
				// 	Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        			&num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
				//      Local variablse used by decode_signal (task in do_recv_pipeline)
				&num_dec_bits, num_dec_bits_sz,
				bit_r, bit_r_sz,
				bit, bit_sz,
				vit_size,
				&ofdm, ofdm_sz,
				&frame, frame_sz,
				&n_res_char, n_res_char_sz,
				//       Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline
				inMemory, inMemory_sz,
				outMemory, outMemory_sz,
				&d_ntraceback, d_ntraceback_arg_sz,
				// End variables used by do_recv_pipeline
				6,
				recvd_in_real, recvd_in_real_sz,
				recvd_in_imag, recvd_in_imag_sz,
				&recvd_msg_len, recvd_msg_len_sz,
				uncmp_data, uncmp_data_sz,
				observations, observations_sz,
				&dec_bytes, dec_bytes_sz);
			__hetero_wait(LaunchInner);
#else
			fuse_maps(n_recvd_in,
				recvd_in_real, recvd_in_real_sz, recvd_in_imag, recvd_in_imag_sz,
				&recvd_msg_len, recvd_msg_len_sz, recvd_msg, recvd_msg_sz,
				observations, observations_sz,
				uncmp_data, uncmp_data_sz, &dec_bytes, dec_bytes_sz,
				// Start variables used by do_recv_pipeline
				// 	Local variables used by do_recv_pipeline
				scrambled_msg, scrambled_msg_sz, &ss_freq_offset, ss_freq_offset_sz,
				&num_sync_short_vals, num_sync_short_vals_sz,
				&sl_freq_offset, sl_freq_offset_sz,
				&num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz,
				fft_ar_i, fft_ar_i_sz, &num_fft_outs, num_fft_outs_sz,
				toBeEqualized, toBeEqualized_sz, equalized, equalized_sz,
				&num_eq_out_bits, num_eq_out_bits_sz, &psdu, psdu_sz,
				//      Global variables used by do_recv_pipeline
				delay16_out, DELAY_16_MAX_OUT_SIZE * sizeof(fx_pt),
				input_data, (DELAY_16_MAX_OUT_SIZE - 16) * sizeof(fx_pt),
				cmpx_conj_out, CMP_CONJ_MAX_SIZE * sizeof(fx_pt),
				cmpx_mult_out, CMP_MULT_MAX_SIZE * sizeof(fx_pt),
				correlation_complex, FIRC_MAVG48_MAX_SIZE * sizeof(fx_pt),
				correlation, CMP2MAG_MAX_SIZE * sizeof(fx_pt1),
				signal_power, CMP2MAGSQ_MAX_SIZE * sizeof(fx_pt1),
				avg_signal_power, FIR_MAVG64_MAX_SIZE * sizeof(fx_pt1),
				the_correlation, DIVIDE_MAX_SIZE * sizeof(fx_pt1),
				sync_short_out_frames, 320 * sizeof(fx_pt),
				d_sync_long_out_frames, SYNC_L_OUT_MAX_SIZE * sizeof(fx_pt),
				// 	Local variable used by do_rcv_ff_work (task in do_recv_pipeline)
        			&num_fft_outs_rcv_fft, num_fft_outs_rcv_fft_sz,
				//      Local variablse used by decode_signal (task in do_recv_pipeline)
				&num_dec_bits, num_dec_bits_sz,
				bit_r, bit_r_sz,
				bit, bit_sz,
				vit_size,
				&ofdm, ofdm_sz,
				&frame, frame_sz,
				&n_res_char, n_res_char_sz,
				//       Local variables for sdr_decode_ofdm (called by decode_signal, a task in do_recv_pipeline
				inMemory, inMemory_sz,
				outMemory, outMemory_sz,
				&d_ntraceback, d_ntraceback_arg_sz);
#endif
			printf("%s %d Out of fuse_maps", __FILE__, __LINE__);

			// This is now the fused map that should be sent to the AV(Car)
			//  The n values of the (fused) local_map Costmap
			// Connect to the Car-Socket and send the data...

			Costmap2D * local_map = &(observations[curr_obs].master_costmap); // Copied from fuse_maps::T3
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_send, NULL);
#endif
			unsigned car_bytes = sizeof(Costmap2D);
			snprintf(r_buffer, 9, "X%-6uX", car_bytes);
			DBGOUT(printf("\nCAR-OUT Sending %s on CAR port %u socket\n", r_buffer, CAR_PORT));
			send(car_sock, r_buffer, 8, 0);
			DBGOUT(printf("     Send %u bytes on CAR port %u socket\n", car_bytes, CAR_PORT));
			char * car_out_chars = (char *) &(local_map);
			DBGOUT2(printf("CAR-OUT %4u : Dumping XMIT-PIPE REAL raw bytes\n", car_send_count);
			for (int i = 0; i < car_bytes; i++) {
				unsigned char c = car_out_chars[i];
				printf("CAR-OUT %4u REAL-byte %6u : %u\n", car_sendcount, i, c);
			} printf("\n"));
#ifdef INT_TIME
			gettimeofday(&start_pd_wifi_car, NULL);
#endif
			send(car_sock, car_out_chars, car_bytes, 0);
			DBGOUT(printf("     Send %u bytes on CAR port %u socket\n", car_bytes, CAR_PORT));
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_car, NULL);
			pd_wifi_car_sec += stop_pd_wifi_car.tv_sec - start_pd_wifi_car.tv_sec;
			pd_wifi_car_usec += stop_pd_wifi_car.tv_usec - start_pd_wifi_car.tv_usec;
#endif
			car_send_count++;
			recv_count++;
		}
		else { // (valread != 8)
			// I don't see the need to put the following code in a graph as there is no parallelism here.
#ifdef INT_TIME
			gettimeofday(&stop_pd_wifi_recv_wait, NULL);
			pd_wifi_recv_wait_sec += stop_pd_wifi_recv_wait.tv_sec - start_pd_wifi_recv_wait.tv_sec;
			pd_wifi_recv_wait_usec += stop_pd_wifi_recv_wait.tv_usec - start_pd_wifi_recv_wait.tv_usec;
#endif
#if PARALLEL_PTHREADS
			if (valread == 0) {
				printf("  RECV header got ZERO bytes -- END of TRANSFER?\n");
				closeout_and_exit("RECV header got zero bytes...", -1);
			}
			else {
				printf("  RECV header got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, 8);
				closeout_and_exit("RECV header got too few bytes...", -1);
			}
#endif
		}
#ifdef INT_TIME
		gettimeofday(&stop_pd_wifi_recv_th, NULL);
		pd_wifi_recv_th_sec = stop_pd_wifi_recv_th.tv_sec - start_pd_wifi_recv_th.tv_sec;
		pd_wifi_recv_th_usec = stop_pd_wifi_recv_th.tv_usec - start_pd_wifi_recv_th.tv_usec;
#endif

#if PARALLEL_PTHREADS
	} // while (1)
#endif
	return NULL;
}

void * receive_and_fuse_maps(void * parm_ptr, size_t parm_ptr_sz) {
#if PARALLEL_PTHREADS
	printf("The receive_and_fuse_maps routine is started: lmap_count = %u\n", lmap_count);
#endif
#ifdef INT_TIME
	gettimeofday(&start_pd_wifi_recv_th, NULL);
#endif
	// We are waiting on lmap_count to change state and as soon as it is done we simply
	// record the time it took.
	// All future tasks in this function can only run after the state of lmap_count changes
	while (lmap_count == 0) {
		DEBUG(printf("  lmap_count = %u\n", lmap_count));
		usleep(1); // wait for first observation (local map) to exist
	}
#ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_lmap_wait, NULL);
	pd_wifi_lmap_wait_sec += stop_pd_wifi_lmap_wait.tv_sec - start_pd_wifi_recv_th.tv_sec;
	pd_wifi_lmap_wait_usec += stop_pd_wifi_lmap_wait.tv_usec - start_pd_wifi_recv_th.tv_usec;
#endif
	receive_and_fuse_maps_impl(observationsArr, sizeof(Observation) * 2);

	return NULL;
}

/* Moved to occgrid.h
	 typedef struct lidar_inputs_struct {
	 float odometry[3];
	 int data_size;
	 char data[200002];
	 }
	 lidar_inputs_t;
	 */

	 // Note: Kindof a major change; previously the entire observation's array was being passed into the function
	 // but only of the value from the array was being used (the index for that value was given by the global
	 // next_obs).
	 // Now, that particular index (and only that index) was being used in the function. So a change was
	 // made to directly pass in the observation value that the index pointed to by next_obs.

void process_lidar_to_occgrid(lidar_inputs_t * lidar_inputs, size_t lidarin_sz /*=sizeof( * lidar_inputs)*/,
	Observation * observationVal /* observations[*next_obs_cp] -> from global array*/, size_t observations_sz /*=sizeof(Observation)*/,
	int * n_cmp_bytes /*return by arg*/, size_t n_cmp_bytes_sz /*=1*/,
	unsigned char * cmp_data /*return by arg*/, size_t cmp_data_sz /*=MAX_COMPRESSED_DATA_SIZE*/,
	// Start of global variables used internally by function
	int * curr_obs_cp /*=curr_obs -> global*/, size_t curr_obs_cp_sz /*=sizeof(int)*/,
	int * next_obs_cp /*=next_obs -> global*/, size_t next_obs_cp_sz /*=sizeof(int)*/,
	int * lidar_count_cp /*lidar_count -> global*/, size_t lidar_count_cp_sz /*=sizeof(unsigned)*/,
	int * lmap_count_cp /*=lmap_count -> global*/, size_t lmap_count_cp_sz /*=sizeof(unsigned)*/,
	// End of global variables used internally by function
	// Start of arguments to cloudToOccgrid (called by process_lidar_to_occgrid)
	double * AVxyzw, size_t AVxyzw_sz /*=sizeof(double)*/,
	bool * rolling_window, size_t rolling_window_sz /*=sizeof(bool)*/,
	double * min_obstacle_height, size_t min_obstacle_height_sz /*=sizeof(double)*/,
	double * max_obstacle_height, size_t max_obstacle_height_sz /*=sizeof(double)*/,
	double * raytrace_range, size_t raytrace_range_sz /*=sizeof(double)*/,
	unsigned int * size_x, size_t size_x_sz /*=sizeof(unsigned int)*/,
	unsigned int * size_y, size_t size_y_sz /*=sizeof(unsigned int)*/,
	unsigned int * resolution, size_t resolution_sz /*=sizeof(unsigned int)*/,
	int * timer_sequentialize, size_t timer_sequentialize_sz /*=sizeof(int) */
	// End of arguments to cloudToOccgrid (called by process_lidar_to_occgrid)
) {


#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * Section = __hetero_section_begin();
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T1_timer_start = __hetero_task_begin(4, lidar_count_cp, lidar_count_cp_sz,
		next_obs_cp, next_obs_cp_sz, lidar_inputs, lidarin_sz,
		timer_sequentialize, timer_sequentialize_sz,
		1, timer_sequentialize, timer_sequentialize_sz,
		"Logging_And_cloudToOccgrid_Timer_Start_Task");
#endif

	// There is are two places where cloudToOccgrid is being timed; one is here and another is inside
	// cloudToOccgrid itself. Not sure why two of them exist so keeping both for now.
	DBGOUT(printf("Lidar step %u : Calling cloudToOccgrid next_obs = %d with odometry %.1f %.1f %.1f\n",
		*lidar_count_cp, *next_obs_cp, lidar_inputs->odometry[0], lidar_inputs->odometry[1],
		lidar_inputs->odometry[2]));
	// int valread = 0; // This is not being used in this function
#if defined(INT_TIME)
	*timer_sequentialize = 1;
	gettimeofday(&start_pd_cloud2grid, NULL);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_task_end(T1_timer_start);
#endif

#if false

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T1 = __hetero_task_begin(14, lidar_inputs, lidarin_sz, observationVal, observations_sz,
		// Args to CloudToOccgrid
		AVxyzw, AVxyzw_sz, rolling_window, rolling_window_sz,
		min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, lidar_count_cp, lidar_count_cp_sz,
		timer_sequentialize, timer_sequentialize_sz,
		3, lidar_inputs, lidarin_sz, observationVal, observations_sz,
		timer_sequentialize, timer_sequentialize_sz,
		"cloudToOccgrid_Task");
	// observationVal is an input because cloudtoOccgrid modifies it. It's an output because
	// the next task (T2) takes it as an input and it needs to get the updated value of observations (i.e.
	// after call to cloudtoOccgrid)

	// It seems all that T1 does is update the value of observationVal to the newly recieved inputs
	// as indicated by lidar_inputs
#endif

	cloudToOccgrid(observationVal, observations_sz, lidar_inputs, lidarin_sz,
		AVxyzw /*=1.5*/, AVxyzw_sz, rolling_window /*=false*/, rolling_window_sz,
		min_obstacle_height /*=0.05*/, min_obstacle_height_sz,
		max_obstacle_height /*=2.05*/, max_obstacle_height_sz,
		raytrace_range /*=RAYTR_RANGE*/, raytrace_range_sz,
		size_x /*=GRID_MAP_X_DIM*/, size_x_sz, size_y /*=GRID_MAP_Y_DIM*/, size_y_sz,
		resolution /*=GRID_MAP_RESLTN*/, resolution_sz,
		timer_sequentialize, timer_sequentialize_sz);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_task_end(T1);
#endif

#endif

	// CloudToOccgrid
	{

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T1_cloudToOccgrid_Task = __hetero_task_begin(10, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
			max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
			size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
			timer_sequentialize, timer_sequentialize_sz,
			2, observationVal, observations_sz, timer_sequentialize, timer_sequentialize_sz,
			"initCostmap_task");
#endif
		{
			*timer_sequentialize = 1;
#ifdef INT_TIME
			// gettimeofday(&ocgr_c2g_total_start, NULL); // See note above this function for why this call was commented out
			gettimeofday(&ocgr_c2g_initCM_start, NULL);
#endif
			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];

			DBGOUT(printf("In cloudToOccgrid with Odometry %.1f %.1f %.f1\n", robot_x, robot_y, robot_z));
			initCostmap(observationVal, *rolling_window, *min_obstacle_height, *max_obstacle_height, *raytrace_range,
				*size_x, *size_y, *resolution, robot_x, robot_y, robot_z);
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_initCM_stop, NULL);
			ocgr_c2g_initCM_sec += ocgr_c2g_initCM_stop.tv_sec - ocgr_c2g_initCM_start.tv_sec;
			ocgr_c2g_initCM_usec += ocgr_c2g_initCM_stop.tv_usec - ocgr_c2g_initCM_start.tv_usec;
#endif

		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T1_cloudToOccgrid_Task);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T2_cloudToOccgrid_Task = __hetero_task_begin(2, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			1, observationVal, observations_sz, "updateOrigin_task");
#endif

		{
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updOrig_start, NULL);
#endif

			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];
			//printf("(1) Number of elements : %d ... ", data_size);
			//printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
			//MOVED to physically inlined here... updateMap(obs_ptr, data, data_size, robot_x, robot_y, robot_z, robot_yaw);
			if (observationVal->rolling_window) {
				//printf("\nUpdating Map .... \n");
				//printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, AVxyzw);
				//printf("   Master Origin = (%f, %f)\n", obs_ptr->master_origin.x, obs_ptr->master_origin.y);
				double new_origin_x = robot_x - observationVal->master_costmap.x_dim / 2;
				double new_origin_y = robot_y - observationVal->master_costmap.y_dim / 2;
				updateOrigin(observationVal, new_origin_x, new_origin_y);
			}
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updOrig_stop, NULL);
			ocgr_c2g_updOrig_sec += ocgr_c2g_updOrig_stop.tv_sec - ocgr_c2g_updOrig_start.tv_sec;
			ocgr_c2g_updOrig_usec += ocgr_c2g_updOrig_stop.tv_usec - ocgr_c2g_updOrig_start.tv_usec;
#endif
		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T2_cloudToOccgrid_Task);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		void * T3_cloudToOccgrid_Task = __hetero_task_begin(3, observationVal, observations_sz, lidar_inputs, lidarin_sz,
			AVxyzw, AVxyzw_sz, 1, observationVal, observations_sz, "updateBounds_task");
#endif
		{
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updBnds_start, NULL);
#endif
			double robot_x = lidar_inputs->odometry[0];
			double robot_y = lidar_inputs->odometry[1];
			double robot_z = lidar_inputs->odometry[2];

			float * data = (float *) (lidar_inputs->data);
			unsigned int data_size = lidar_inputs->data_size / sizeof(float);

			double min_x = 1e30;
			double min_y = 1e30;
			double max_x = -1e30;
			double max_y = -1e30;

			//printf("(1) Number of elements : %d ... ", data_size);
			//printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
			//rotating_window = true; //Comment out if not rolling window

			updateBounds(observationVal, data, data_size, robot_x, robot_y, robot_z,
				*AVxyzw, &min_x, &min_y, &max_x, &max_y);

			//printMap();
#ifdef INT_TIME
			gettimeofday(&ocgr_c2g_updBnds_stop, NULL);
			ocgr_c2g_updBnds_sec += ocgr_c2g_updBnds_stop.tv_sec - ocgr_c2g_updBnds_start.tv_sec;
			ocgr_c2g_updBnds_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_updBnds_start.tv_usec;

			// Note (located above this function) explains why the following lines were commented out
			// ocgr_c2g_total_sec  += ocgr_c2g_updBnds_stop.tv_sec  - ocgr_c2g_total_start.tv_sec;
			// ocgr_c2g_total_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
#endif
		}
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
		__hetero_task_end(T3_cloudToOccgrid_Task);
#endif
#if defined(INT_TIME) && !(defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL))
		gettimeofday(&ocgr_c2g_total_stop, NULL);
		ocgr_c2g_total_sec += ocgr_c2g_total_stop.tv_sec - ocgr_c2g_total_start.tv_sec;
		ocgr_c2g_total_usec += ocgr_c2g_total_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
#endif


	}



#if defined(INT_TIME)
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T1_timer_end = __hetero_task_begin(1, timer_sequentialize, timer_sequentialize_sz,
		1, timer_sequentialize, timer_sequentialize_sz, "cloudToOccgrid_Timer_End_Task");
#endif

	* timer_sequentialize = 3;
#ifdef INT_TIME
	gettimeofday(&stop_pd_cloud2grid, NULL);
	pd_cloud2grid_sec += stop_pd_cloud2grid.tv_sec - start_pd_cloud2grid.tv_sec;
	pd_cloud2grid_usec += stop_pd_cloud2grid.tv_usec - start_pd_cloud2grid.tv_usec;
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_task_end(T1_timer_end);
#endif
#endif // defined(INT_TIME)

	// Write the read-in image to a file
	// write_array_to_file(grid, COST_MAP_ENTRIES);
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T2 = __hetero_task_begin(6, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz, next_obs_cp, next_obs_cp_sz, curr_obs_cp, curr_obs_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		3, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		"compressMap_Task");
#endif

	printf("%s %d In T2", __FILE__, __LINE__);

	// Now we compress the grid for transmission...
	Costmap2D * local_map = &(observationVal->master_costmap);
	DBGOUT(printf("Calling LZ4_compress_default...\n"));
	DBGOUT(printf("  Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y,
		local_map->av_z);
	printf("               : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim,
		local_map->y_dim);
	print_ascii_costmap(stdout, local_map));
#ifdef WRITE_ASCII_MAP
	char ascii_file_name[32];
	snprintf(ascii_file_name, sizeof(char) * 32, "%sform_%04d.txt", ASCII_FN, ascii_counter);
	FILE * ascii_fp = fopen(ascii_file_name, "w");
	// printf("Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
	fprintf(ascii_fp, "Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y,
		local_map->av_z);
	fprintf(ascii_fp, "             : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size,
		local_map->x_dim, local_map->y_dim);
	print_ascii_costmap(ascii_fp, local_map);
	fclose(ascii_fp);
#endif
	// Now we update the current observation index and the next observation index
	//      Switch curr_obs (global referred to by curr_obs_cp) and next_obs (global referred to by next_obs_cp)
	//      between 0 and 1
	* curr_obs_cp = 1 - *curr_obs_cp;
	*next_obs_cp = 1 - *next_obs_cp;
	(*lmap_count_cp)++;
	// And now we compress to encode for Wifi transmission, etc.
#ifdef INT_TIME
	gettimeofday(&start_pd_lz4_cmp, NULL);
#endif
	* n_cmp_bytes = LZ4_compress_default((char *) local_map, (char *) cmp_data,
		MAX_UNCOMPRESSED_DATA_SIZE, MAX_COMPRESSED_DATA_SIZE);

#ifdef INT_TIME
	gettimeofday(&stop_pd_lz4_cmp, NULL);
	pd_lz4_cmp_sec += stop_pd_lz4_cmp.tv_sec - start_pd_lz4_cmp.tv_sec;
	pd_lz4_cmp_usec += stop_pd_lz4_cmp.tv_usec - start_pd_lz4_cmp.tv_usec;
#endif
	DBGOUT(double c_ratio = 100 * (1 - ((double) (*n_cmp_bytes) / (double) (MAX_UNCOMPRESSED_DATA_SIZE))); printf("  Back from LZ4_compress_default: %lu bytes -> %u bytes for %5.2f%%\n",
		MAX_UNCOMPRESSED_DATA_SIZE, *n_cmp_bytes, c_ratio););


#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_task_end(T2);

	__hetero_section_end(Section);
#endif
}

void process_lidar_to_occgrid_Wrapper(lidar_inputs_t * lidar_inputs, size_t lidarin_sz /*=sizeof( * lidar_inputs)*/,
	Observation * observationVal /* observations[*next_obs_cp] -> from global array*/, size_t observations_sz /*=sizeof(Observation)*/,
	int * n_cmp_bytes /*return by arg*/, size_t n_cmp_bytes_sz /*=1*/,
	unsigned char * cmp_data /*return by arg*/, size_t cmp_data_sz /*=MAX_COMPRESSED_DATA_SIZE*/,
	// Start of global variables used internally by function
	int * curr_obs_cp /*=curr_obs -> global*/, size_t curr_obs_cp_sz /*=sizeof(int)*/,
	int * next_obs_cp /*=next_obs -> global*/, size_t next_obs_cp_sz /*=sizeof(int)*/,
	int * lidar_count_cp /*lidar_count -> global*/, size_t lidar_count_cp_sz /*=sizeof(unsigned)*/,
	int * lmap_count_cp /*=lmap_count -> global*/, size_t lmap_count_cp_sz /*=sizeof(unsigned)*/,
	// End of global variables used internally by function
	// Start of arguments to cloudToOccgrid (called by process_lidar_to_occgrid)
	double * AVxyzw, size_t AVxyzw_sz /*=sizeof(double)*/,
	bool * rolling_window, size_t rolling_window_sz /*=sizeof(bool)*/,
	double * min_obstacle_height, size_t min_obstacle_height_sz /*=sizeof(double)*/,
	double * max_obstacle_height, size_t max_obstacle_height_sz /*=sizeof(double)*/,
	double * raytrace_range, size_t raytrace_range_sz /*=sizeof(double)*/,
	unsigned int * size_x, size_t size_x_sz /*=sizeof(unsigned int)*/,
	unsigned int * size_y, size_t size_y_sz /*=sizeof(unsigned int)*/,
	unsigned int * resolution, size_t resolution_sz /*=sizeof(unsigned int)*/,
	int * timer_sequentialize, size_t timer_sequentialize_sz /*=sizeof(int) */
	// End of arguments to cloudToOccgrid (called by process_lidar_to_occgrid)
) {


#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * Section = __hetero_section_begin();
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	void * T1 = __hetero_task_begin(17, lidar_inputs, lidarin_sz, n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz, observationVal, observations_sz, timer_sequentialize, timer_sequentialize_sz,
		// Args for cloudToOccgrid
		AVxyzw, AVxyzw_sz,
		rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		// Global vars used by process_lidar_to_occgrid
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, lidar_count_cp, lidar_count_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		// Output
		3, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		"proccess_lidar_to_occgrid_caller_task_Wrapper1");
#endif

	process_lidar_to_occgrid(lidar_inputs, lidarin_sz, observationVal, observations_sz,
		n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		// Global vars used by process_lidar_to_occgrid
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, lidar_count_cp, lidar_count_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		// Args for cloudToOccgrid
		AVxyzw, AVxyzw_sz, rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		timer_sequentialize, timer_sequentialize_sz); // buffer, total_bytes_read);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_task_end(T1);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR_INTERNAL)) && true
	__hetero_section_end(Section);
#endif
}

// Local variables for do_xmit_pipeline
#define MAX_SIZE 24600          // from xmit_pipe.c
#define ofdm_max_out_size 33280 // from xmit_pipe.c

void transmit_occgrid(int * n_cmp_bytes /*from process_lidar_to_occgrid*/, size_t n_cmp_bytes_sz /*=1*/,
	unsigned char * cmp_data /*from process_lidar_to_occgrid*/, size_t cmp_data_sz /*=MAX_COMPRESSED_DATA_SIZE*/,
	int n_xmit_out, float * xmit_out_real, size_t xmit_out_real_sz,
	float * xmit_out_imag, size_t xmit_out_imag_sz,
	int * psdu_len, size_t psdu_len_sz,
	uint8_t * pckt_hdr_out, size_t pckt_hdr_out_sz,
	int * pckt_hdr_len, size_t pckt_hdr_len_sz,
	float * msg_stream_real, size_t msg_stream_real_sz,
	float * msg_stream_imag, size_t msg_stream_imag_sz,
	float * ofdm_car_str_real, size_t ofdm_car_str_real_sz,
	float * ofdm_car_str_imag, size_t ofdm_car_str_imag_sz,
	int * ofc_res, size_t ofc_res_sz,
	float * fft_out_real, size_t fft_out_real_sz,
	float * fft_out_imag, size_t fft_out_imag_sz,
	float * cycpref_out_real, size_t cycpref_out_real_sz,
	float * cycpref_out_imag, size_t cycpref_out_imag_sz) {
	// This section has no tasks as we are doing IO; introducing tasks can lead to race conditions
	// Now we transmit the grid...

	DBGOUT(printf("  Back from do_xmit_pipeline with %u xmit outputs...\n", *n_xmit_out));

	// This is now the content that should be sent out via IEEE 802.11p WiFi
	// The n_xmit_out values of xmit_out_real and xmit_out_imag
	// Connect to the Wifi-Socket and send the n_xmit_out
	char w_buffer[10];
#ifdef INT_TIME
	gettimeofday(&start_pd_wifi_send, NULL);
#endif
	unsigned xfer_bytes = (n_xmit_out) * sizeof(float);
	snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
	DBGOUT(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
	send(xmit_sock, w_buffer, 8, 0);
	DBGOUT(printf("     Send %u REAL values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
		XMIT_PORT));
	DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE REAL raw bytes\n", xmit_count);
	for (int i = 0; i < n_xmit_out; i++) {
		printf("XFER %4u REAL-byte %6u : %f\n", xmit_count, i, xmit_out_real[i]);
	} printf("\n"));
#ifdef INT_TIME
	gettimeofday(&start_pd_wifi_send_rl, NULL);
#endif
	send(xmit_sock, (char *) (xmit_out_real), (n_xmit_out) * sizeof(float), 0);
	DBGOUT(printf("     Send %u IMAG values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
		XMIT_PORT));
	DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE IMAG raw bytes\n", xmit_count);
	for (int i = 0; i < (n_xmit_out); i++) {
		printf("XFER %4u IMAG-byte %6u : %f\n", xmit_count, i, xmit_out_imag[i]);
	} printf("\n"));
#ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_send_rl, NULL);
#endif
	send(xmit_sock, (char *) (xmit_out_imag), (n_xmit_out) * sizeof(float), 0);
#ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_send, NULL);
	pd_wifi_send_sec += stop_pd_wifi_send.tv_sec - start_pd_wifi_send.tv_sec;
	pd_wifi_send_usec += stop_pd_wifi_send.tv_usec - start_pd_wifi_send.tv_usec;
	pd_wifi_send_rl_sec += stop_pd_wifi_send_rl.tv_sec - start_pd_wifi_send_rl.tv_sec;
	pd_wifi_send_rl_usec += stop_pd_wifi_send_rl.tv_usec - start_pd_wifi_send_rl.tv_usec;
	pd_wifi_send_im_sec += stop_pd_wifi_send.tv_sec - stop_pd_wifi_send_rl.tv_sec;
	pd_wifi_send_im_usec += stop_pd_wifi_send.tv_usec - stop_pd_wifi_send_rl.tv_usec;
#endif

	xmit_count++;
}

/***********************************************************************************
	void encode_transmit_occgrid(int *n_cmp_bytes, size_t n_cmp_bytes_sz,
	unsigned char *cmp_data, size_t cmp_data_sz,
	int n_xmit_out, float *xmit_out_real, size_t xmit_out_real_sz,
	float *xmit_out_imag, size_t xmit_out_imag_sz,
	int psdu_len, size_t psdu_len_sz,
	uint8_t *pckt_hdr_out, size_t pckt_hdr_out_sz,
	int pckt_hdr_len, size_t pckt_hdr_len_sz,
	float *msg_stream_real, size_t msg_stream_real_sz,
	float *msg_stream_imag, size_t msg_stream_imag_sz,
	float *ofdm_car_str_real, size_t ofdm_car_str_real_sz,
	float *ofdm_car_str_imag, size_t ofdm_car_str_imag_sz,
	int ofc_res, size_t ofc_res_sz,
	float *fft_out_real, size_t fft_out_real_sz,
	float *fft_out_imag, size_t fft_out_imag_sz,
	float *cycpref_out_real, size_t cycpref_out_real_sz,
	float *cycpref_out_imag, size_t cycpref_out_imag_sz)
	{
// This section has no tasks as we are doing IO; introducing tasks can lead to race conditions
// Now we encode and transmit the grid...
DBGOUT(printf("Calling do_xmit_pipeline for %u compressed grid elements\n", n_cmp_bytes));

#ifdef INT_TIME
gettimeofday(&start_pd_wifi_pipe, NULL);
#endif

#if defined(HPVM) && true
void *LaunchInner = __hetero_launch((void *)do_xmit_pipeline, 17,
n_cmp_bytes, n_cmp_bytes_sz, (char *)cmp_data, cmp_data_sz,
&n_xmit_out, sizeof(int), xmit_out_real, xmit_out_real_sz,
xmit_out_imag, xmit_out_imag_sz,
// Start of local variables for do_xmit_pipeline
&psdu_len, psdu_len_sz, pckt_hdr_out, pckt_hdr_out_sz,
&pckt_hdr_len, pckt_hdr_len_sz,
msg_stream_real, msg_stream_real_sz,
msg_stream_imag, msg_stream_imag_sz,
ofdm_car_str_real, ofdm_car_str_real_sz,
ofdm_car_str_imag, ofdm_car_str_imag_sz, &ofc_res, ofc_res_sz,
fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz,
cycpref_out_real, cycpref_out_real_sz,
cycpref_out_imag, cycpref_out_imag_sz,
// End of local variables for do_xmit_pipeline
3,
&n_xmit_out, sizeof(int), xmit_out_real, xmit_out_real_sz,
xmit_out_imag, xmit_out_imag_sz);
__hetero_wait(LaunchInner);
#else
do_xmit_pipeline(n_cmp_bytes, n_cmp_bytes_sz, (char *)cmp_data, cmp_data_sz,
&n_xmit_out, sizeof(int), xmit_out_real, xmit_out_real_sz,
xmit_out_imag, xmit_out_imag_sz,
// Start of local variables for do_xmit_pipeline
&psdu_len, psdu_len_sz, pckt_hdr_out, pckt_hdr_out_sz,
&pckt_hdr_len, pckt_hdr_len_sz,
msg_stream_real, msg_stream_real_sz,
msg_stream_imag, msg_stream_imag_sz,
ofdm_car_str_real, ofdm_car_str_real_sz,
ofdm_car_str_imag, ofdm_car_str_imag_sz, &ofc_res, ofc_res_sz,
fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz,
cycpref_out_real, cycpref_out_real_sz,
cycpref_out_imag, cycpref_out_imag_sz);
#endif

transmit_occgrid(n_cmp_bytes, n_cmp_bytes_sz,
cmp_data, cmp_data_sz,
 *n_xmit_out,
 xmit_out_real, xmit_out_real_sz,
 xmit_out_imag, xmit_out_imag_sz,
 psdu_len, psdu_len_sz,
 pckt_hdr_out, pckt_hdr_out_sz,
 pckt_hdr_len, pckt_hdr_len_sz,
 msg_stream_real, msg_stream_real_sz,
msg_stream_imag, msg_stream_imag_sz,
	ofdm_car_str_real, ofdm_car_str_real_sz,
	ofdm_car_str_imag, ofdm_car_str_imag_sz,
	ofc_res, ofc_res_sz,
	fft_out_real, fft_out_real_sz,
	fft_out_imag, fft_out_imag_sz,
	cycpref_out_real, cycpref_out_real_sz,
	cycpref_out_imag, cycpref_out_imag_sz);

// #ifdef INT_TIME
//                                                              gettimeofday( & stop_pd_wifi_pipe, NULL);
//                                                              pd_wifi_pipe_sec += stop_pd_wifi_pipe.tv_sec - start_pd_wifi_pipe.tv_sec;
//                                                              pd_wifi_pipe_usec += stop_pd_wifi_pipe.tv_usec - start_pd_wifi_pipe.tv_usec;
// #endif
//                                                              DBGOUT(printf("  Back from do_xmit_pipeline with %u xmit outputs...\n", *n_xmit_out));

//                                                              // This is now the content that should be sent out via IEEE 802.11p WiFi
//                                                              // The n_xmit_out values of xmit_out_real and xmit_out_imag
//                                                              // Connect to the Wifi-Socket and send the n_xmit_out
//                                                              char w_buffer[10];
// #ifdef INT_TIME
//                                                              gettimeofday( & start_pd_wifi_send, NULL);
// #endif
//                                                              unsigned xfer_bytes = (n_xmit_out) * sizeof(float);
//                                                              snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
//                                                              DBGOUT(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
//                                                              send(xmit_sock, w_buffer, 8, 0);
//                                                              DBGOUT(printf("     Send %u REAL values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
//                                                                                                                              XMIT_PORT));
//                                                              DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE REAL raw bytes\n", xmit_count);
//                                                                                                                              for (int i = 0; i < n_xmit_out; i++) {
//                                                                                                                              printf("XFER %4u REAL-byte %6u : %f\n", xmit_count, i, xmit_out_real[i]);
//                                                                                                                              }
//                                                                                                                              printf("\n"));
// #ifdef INT_TIME
//                                                              gettimeofday( & start_pd_wifi_send_rl, NULL);
// #endif
//                                                              send(xmit_sock, (char * )(xmit_out_real), (n_xmit_out) * sizeof(float), 0);
//                                                              DBGOUT(printf("     Send %u IMAG values %u bytes on XMIT port %u socket\n", n_xmit_out, xfer_bytes,
//                                                                                                                              XMIT_PORT));
//                                                              DBGOUT2(printf("XFER %4u : Dumping XMIT-PIPE IMAG raw bytes\n", xmit_count);
//                                                                                                                              for (int i = 0; i < (n_xmit_out); i++) {
//                                                                                                                              printf("XFER %4u IMAG-byte %6u : %f\n", xmit_count, i, xmit_out_imag[i]);
//                                                                                                                              }
//                                                                                                                              printf("\n"));
// #ifdef INT_TIME
//                                                              gettimeofday( & stop_pd_wifi_send_rl, NULL);
// #endif
//                                                              send(xmit_sock, (char * )(xmit_out_imag), (n_xmit_out) * sizeof(float), 0);
// #ifdef INT_TIME
//                                                              gettimeofday( & stop_pd_wifi_send, NULL);
//                                                              pd_wifi_send_sec += stop_pd_wifi_send.tv_sec - start_pd_wifi_send.tv_sec;
//                                                              pd_wifi_send_usec += stop_pd_wifi_send.tv_usec - start_pd_wifi_send.tv_usec;
//                                                              pd_wifi_send_rl_sec += stop_pd_wifi_send_rl.tv_sec - start_pd_wifi_send_rl.tv_sec;
//                                                              pd_wifi_send_rl_usec += stop_pd_wifi_send_rl.tv_usec - start_pd_wifi_send_rl.tv_usec;
//                                                              pd_wifi_send_im_sec += stop_pd_wifi_send.tv_sec - stop_pd_wifi_send_rl.tv_sec;
//                                                              pd_wifi_send_im_usec += stop_pd_wifi_send.tv_usec - stop_pd_wifi_send_rl.tv_usec;
// #endif

//                                                              xmit_count++;
}
***********************************************************************************/

void lidar_root(lidar_inputs_t * lidar_inputs, size_t lidarin_sz /*=sizeof( * lidar_inputs)*/,
	Observation * observationVal /* observations[*next_obs_cp] -> from global array*/, size_t observations_sz /*=sizeof(Observation)*2*/,
	int * n_cmp_bytes /*return by arg*/, size_t n_cmp_bytes_sz /*=sizeof(int)*1*/,
	unsigned char * cmp_data /*return by arg*/, size_t cmp_data_sz /*=MAX_COMPRESSED_DATA_SIZE*/,
	// Start of global variables used internally by function
	int * curr_obs_cp /*=curr_obs -> global*/, size_t curr_obs_cp_sz /*=sizeof(int)*/,
	int * next_obs_cp /*=next_obs -> global*/, size_t next_obs_cp_sz /*=sizeof(int)*/,
	int * lidar_count_cp /*=lidar_count -> global*/, size_t lidar_count_cp_sz /*=sizeof(unsigned)*/,
	int * lmap_count_cp /*=lmap_count -> global*/, size_t lmap_count_cp_sz /*=sizeof(unsigned)*/,
	// End of global variables used internally by function
	// Start of arguments to cloudToOccgrid (called indirectly by lidar_root)
	double * AVxyzw, size_t AVxyzw_sz /*=sizeof(double)*/,
	bool * rolling_window, size_t rolling_window_sz /*=sizeof(bool)*/,
	double * min_obstacle_height, size_t min_obstacle_height_sz /*=sizeof(double)*/,
	double * max_obstacle_height, size_t max_obstacle_height_sz /*=sizeof(double)*/,
	double * raytrace_range, size_t raytrace_range_sz /*=sizeof(double)*/,
	unsigned int * size_x, size_t size_x_sz /*=sizeof(unsigned int)*/,
	unsigned int * size_y, size_t size_y_sz /*=sizeof(unsigned int)*/,
	unsigned int * resolution, size_t resolution_sz /*=sizeof(unsigned int)*/,
	int * timer_sequentialize, size_t timer_sequentialize_sz /*=sizeof(int) */,
	// End of arguments to cloudToOccgrid (called indirectly by lidar_root)
	// Start of arguments to encode_occgrid (called indirectly by lidar_root)
	int * n_xmit_out, size_t n_xmit_out_sz,
	float * xmit_out_real, size_t xmit_out_real_sz,
	float * xmit_out_imag, size_t xmit_out_imag_sz,
	int * psdu_len, size_t psdu_len_sz,
	uint8_t * pckt_hdr_out, size_t pckt_hdr_out_sz,
	int * pckt_hdr_len, size_t pckt_hdr_len_sz,
	float * msg_stream_real, size_t msg_stream_real_sz,
	float * msg_stream_imag, size_t msg_stream_imag_sz,
	float * ofdm_car_str_real, size_t ofdm_car_str_real_sz,
	float * ofdm_car_str_imag, size_t ofdm_car_str_imag_sz,
	int * ofc_res, size_t ofc_res_sz,
	float * fft_out_real, size_t fft_out_real_sz,
	float * fft_out_imag, size_t fft_out_imag_sz,
	float * cycpref_out_real, size_t cycpref_out_real_sz,
	float * cycpref_out_imag, size_t cycpref_out_imag_sz
	// End of arguments to encode_occgrid (called indirectly by lidar_root)
) {
#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(17, lidar_inputs, lidarin_sz, n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz, observationVal, observations_sz, timer_sequentialize, timer_sequentialize_sz,
		// Args for cloudToOccgrid
		AVxyzw, AVxyzw_sz,
		rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		// Global vars used by process_lidar_to_occgrid
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, lidar_count_cp, lidar_count_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		// Output
		3, observationVal, observations_sz, n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		"proccess_lidar_to_occgrid_caller_task");
#endif

	process_lidar_to_occgrid_Wrapper(lidar_inputs, lidarin_sz, observationVal, observations_sz,
		n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		// Global vars used by process_lidar_to_occgrid
		curr_obs_cp, curr_obs_cp_sz, next_obs_cp, next_obs_cp_sz, lidar_count_cp, lidar_count_cp_sz,
		lmap_count_cp, lmap_count_cp_sz,
		// Args for cloudToOccgrid
		AVxyzw, AVxyzw_sz, rolling_window, rolling_window_sz, min_obstacle_height, min_obstacle_height_sz,
		max_obstacle_height, max_obstacle_height_sz, raytrace_range, raytrace_range_sz,
		size_x, size_x_sz, size_y, size_y_sz, resolution, resolution_sz,
		timer_sequentialize, timer_sequentialize_sz); // buffer, total_bytes_read);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_task_end(T1);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	void * T2 = __hetero_task_begin(17,
		// Args for encode_occgrid
		n_cmp_bytes, n_cmp_bytes_sz,
		cmp_data, cmp_data_sz,
		n_xmit_out, n_xmit_out_sz,
		xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		// Start of local variables for do_xmit_pipeline
		psdu_len, psdu_len_sz,
		pckt_hdr_out, pckt_hdr_out_sz,
		pckt_hdr_len, pckt_hdr_len_sz,
		msg_stream_real, msg_stream_real_sz,
		msg_stream_imag, msg_stream_imag_sz,
		ofdm_car_str_real, ofdm_car_str_real_sz,
		ofdm_car_str_imag, ofdm_car_str_imag_sz,
		ofc_res, ofc_res_sz,
		fft_out_real, fft_out_real_sz,
		fft_out_imag, fft_out_imag_sz,
		cycpref_out_real, cycpref_out_real_sz,
		cycpref_out_imag, cycpref_out_imag_sz,
		// End of local variables for do_xmit_pipeline
		3, n_xmit_out, n_xmit_out_sz,
		xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		"TX_task");
#endif

	do_xmit_pipeline(n_cmp_bytes, n_cmp_bytes_sz, cmp_data, cmp_data_sz,
		n_xmit_out, n_xmit_out_sz, xmit_out_real, xmit_out_real_sz,
		xmit_out_imag, xmit_out_imag_sz,
		// Start of local variables for do_xmit_pipeline
		psdu_len, psdu_len_sz, pckt_hdr_out, pckt_hdr_out_sz,
		pckt_hdr_len, pckt_hdr_len_sz,
		msg_stream_real, msg_stream_real_sz,
		msg_stream_imag, msg_stream_imag_sz,
		ofdm_car_str_real, ofdm_car_str_real_sz,
		ofdm_car_str_imag, ofdm_car_str_imag_sz, ofc_res, ofc_res_sz,
		fft_out_real, fft_out_real_sz, fft_out_imag, fft_out_imag_sz,
		cycpref_out_real, cycpref_out_real_sz,
		cycpref_out_imag, cycpref_out_imag_sz);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_task_end(T2);
#endif

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
	__hetero_section_end(Section);
#endif
}

#ifdef CV_PIPELINE
#ifdef USE_OLD_MODEL

void cv_root(unsigned tr_val, label_t * out_label, size_t outlabel_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT))
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(2, tr_val, out_label, outlabel_sz, 1, out_label, outlabel_sz, "cv_root_task");
#ifdef GPU
	__hpvm__hint(GPU_TARGET);
#endif
#endif
	* out_label = run_object_classification(tr_val);

#if (defined(HPVM) && defined(HPVM_CV_ROOT))
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

void cv_root_wrapper(unsigned tr_val, label_t * out_label, size_t outlabel_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(2, tr_val, out_label, outlabel_sz, 1, out_label, outlabel_sz, "cv_root_wrapper_task");
#ifdef GPU
	__hpvm__hint(GPU_TARGET);
#endif
#endif

	cv_root(tr_val, out_label, outlabel_sz);

#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

#else

void cv_root(uint8_t * rgb_image, size_t rgb_image_sz, dim_t * dimensions, size_t dimensions_sz,
	char * filename, size_t filename_sz, int * nboxes, size_t nboxes_sz,
	detection_t * dets, size_t dets_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(5, rgb_image, rgb_image_sz, dimensions, dimensions_sz, filename, filename_sz,
		nboxes, nboxes_sz, dets, dets_sz, 1, dets, dets_sz, "cv_root_task");
#endif
	dets = run_object_classification(rgb_image, *dimensions, filename, nboxes); 

#if  (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

void cv_root_wrapper(uint8_t * rgb_image, size_t rgb_image_sz, dim_t * dimensions, size_t dimensions_sz,
	char * filename, size_t filename_sz, int * nboxes, size_t nboxes_sz,
	detection_t * dets, size_t dets_sz) {
#if (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	void * Section = __hetero_section_begin();
	void * T1 = __hetero_task_begin(5, rgb_image, rgb_image_sz, dimensions, dimensions_sz, filename, filename_sz,
		nboxes, nboxes_sz, dets, dets_sz, 1, dets, dets_sz, "cv_root_wrapper_task");
#endif

	cv_root(rgb_image, rgb_image_sz, dimensions, dimensions_sz, filename, filename_sz, nboxes, nboxes_sz, dets, dets_sz);

#if  (defined(HPVM) && defined(HPVM_CV_ROOT)) && true
	__hetero_task_end(T1);
	__hetero_section_end(Section);
#endif
}

#endif // ifdef USE_OLD_MODEL
#endif // CV_PIPELINE

int main(int argc, char * argv[]) {
	struct sockaddr_in bag_servaddr;
	struct sockaddr_in xmit_servaddr;
	struct sockaddr_in recv_servaddr;
	struct sockaddr_in car_servaddr;
	unsigned char l_buffer[20] = {
		0 };
	// unsigned char buffer[200002] = {0};

	lidar_inputs_t lidar_inputs;

#ifdef RISCV
	snprintf(bag_inet_addr_str, 20, "192.168.1.99");
	snprintf(wifi_inet_addr_str, 20, "192.168.1.99");
	snprintf(car_inet_addr_str, 20, "192.168.1.99");
#else //X86
	snprintf(bag_inet_addr_str, 20, "127.0.0.1");
	snprintf(wifi_inet_addr_str, 20, "127.0.0.1");
	snprintf(car_inet_addr_str, 20, "127.0.0.1");
  //Carla interface
	// snprintf(bag_inet_addr_str, 20, "9.2.210.65");
	// snprintf(wifi_inet_addr_str, 20, "192.168.1.99");
	// snprintf(car_inet_addr_str, 20, "9.2.210.65");
#endif

	
#ifdef ERA1
	printf("This is ERA1\n");
#endif

#ifdef ERA2
	printf("This is ERA2\n");
#endif


	// hpvm: The inits below can probably all go in parallel
	printf("Initializing the OccGrid state...\n");
	init_occgrid_state(); // Initialize the occgrid functions, state, etc.
	printf("Initializing the Transmit pipeline...\n");
	xmit_pipe_init(); // Initialize the IEEE SDR Transmit Pipeline
	printf("Initializing the Receive pipeline...\n");
	recv_pipe_init();
	printf("Initializing the Computer Vision toolset...\n");

#ifdef CV_PIPELINE
#ifdef USE_OLD_MODEL
	if (cv_toolset_init() != success) {
		printf("Computer Vision toolset initialization failed...\n");
		exit(0);
	}
#else
	/*****************************************************************************/
	/* NEW: PyTorch TinyYOLOv2 support (May 2022)                                */
	// TODO: Uncomment the if-statement below! This is commented as my machine doesn't have opencv installed so everything related to cv is currently being ignored on my side.
	if (cv_toolset_init("tiny_yolov2_coco", "/dccstor/epochs/aporvaa/hetero_era/src/cv/yolo/yolov2-tiny.weights") != 0) {
		printf("Computer Vision toolset initialization failed...\n");
		exit(1);
	}
	/*****************************************************************************/
#endif
#endif // CV_PIPELINE

	signal(SIGINT, INThandler);
	signal(SIGPIPE, SIGPIPE_handler);

	// Use getopt to read in run-time options
	// put ':' in the starting of the
	// string so that program can
	// distinguish between '?' and ':'
	int opt;
	while ((opt = getopt(argc, argv, ":hB:W:C:s:")) != -1) {
		switch (opt) {
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
	}
	else {
		printf("Running the entire bag file.\n");
	}

	printf("Connecting to bag-server at IP %s PORT %u\n", bag_inet_addr_str, BAG_PORT);
	// Open and connect to the BAG_SERVER
	if ((bag_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
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
		if (connect(bag_sock, (struct sockaddr *) &bag_servaddr, sizeof(bag_servaddr)) != 0) {
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
	printf("Connecting to xmit-server at IP %s PORT %u\n", wifi_inet_addr_str, XMIT_PORT);
	if ((xmit_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
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
		if (connect(xmit_sock, (struct sockaddr *) &xmit_servaddr, sizeof(xmit_servaddr)) != 0) {
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
	if ((recv_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
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
		if (connect(recv_sock, (struct sockaddr *) &recv_servaddr, sizeof(recv_servaddr)) != 0) {
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
	if ((car_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
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
		if (connect(car_sock, (struct sockaddr *) &car_servaddr, sizeof(car_servaddr)) != 0) {
			printf("connection with the CAR server failed...\n");
			sleep(1);
			continue;
		}
		else {
			printf("connected to the CAR server..\n");
			break;
		}
	}

#if PARALLEL_PTHREADS
	// Now set up the processing threads - 1 for Lidar input, one for WiFi RECV processing (fusion)
	pthread_attr_t pt_attr;
	pthread_attr_init(&pt_attr);
	// pthread_t form_occmap_thread;
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

	/*************************************************************************/
	/*                           M A I N   L O O P                           */
	/*************************************************************************/
	while ((!hit_eof) && (lidar_count < max_time_steps)) {

		DBGOUT(printf("Calling read_all on the BAG socket...\n"); fflush(stdout));
		// int valread = read(bag_sock , l_buffer, 10);
		//#ifdef INT_TIME
		gettimeofday(&start_proc_rdbag, NULL);
		//#endif
		int valread = read_all(bag_sock, (char *) l_buffer, 10);
		//#ifdef INT_TIME
		gettimeofday(&stop_proc_rdbag, NULL);
		proc_rdbag_sec += stop_proc_rdbag.tv_sec - start_proc_rdbag.tv_sec;
		proc_rdbag_usec += stop_proc_rdbag.tv_usec - start_proc_rdbag.tv_usec;
		//#endif
		DBGOUT(printf("Top: read %d bytes\n", valread));
		if (valread < 10) {
			if (valread == 0) {
				// Zero bytes out implies we hit EOF
				hit_eof = true;
			}
			else {
				printf("  TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, 10);
				closeout_and_exit("Top read got too few bytes...", -1);
			}
		}

		/***********************************************************************/
		/* Checking if the received message includes lidar information         */
		/***********************************************************************/
		if (l_buffer[0] == 'L' && l_buffer[9] == 'L') {

			//#ifdef INT_TIME
			gettimeofday(&start_proc_lidar, NULL);
			//#endif
			char * ptr;
			int message_size = strtol((char *) l_buffer + 1, &ptr, 10);
			DBGOUT(printf("Lidar: expecting message size: %d\n", message_size));
			send(bag_sock, ack, 2, 0);

			int total_bytes_read = read_all(bag_sock, lidar_inputs.data, message_size);
			if (total_bytes_read < message_size) {
				if (total_bytes_read == 0) {
					printf("  Lidar read got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("Lidar read got zero bytes...", -1);
				}
				else {
					printf("  Lidar read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", total_bytes_read,
						message_size);
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

			// Arguments to lidar_root
			int n_cmp_bytes = 0;
			size_t n_cmp_bytes_sz = sizeof(int);
			unsigned char cmp_data[MAX_COMPRESSED_DATA_SIZE];
			size_t cmp_data_sz = MAX_COMPRESSED_DATA_SIZE;

			// Start of arguments to cloudToOccgrid (indirectly called by lidar_root)
			double AVxyzw = 1.5;
			bool rolling_window = false;
			double min_obstacle_heght = 0.05;
			double max_obstacle_heght = 2.05;
			double raytrace_range = RAYTR_RANGE;
			unsigned int size_x = GRID_MAP_X_DIM;
			unsigned int size_y = GRID_MAP_Y_DIM;
			unsigned int resolution = GRID_MAP_RESLTN;
			// End of arguments to cloudToOccgrid (indirectly called by lidar_root)

			int timer_sequentialize = 0;

			// Start of arguments to do_xmit_pipeline (called by lidar_root)
			int n_xmit_out = 0;
			size_t n_xmit_out_sz = sizeof(int);
			float xmit_out_real[MAX_XMIT_OUTPUTS];
			size_t xmit_out_real_sz = MAX_XMIT_OUTPUTS * sizeof(float);
			float xmit_out_imag[MAX_XMIT_OUTPUTS];
			size_t xmit_out_imag_sz = MAX_XMIT_OUTPUTS * sizeof(float);
			// End of arguments to do_xmit_pipeline (called by lidar_root)

			// Start of arguments to do_xmit_pipeline (called by lidar_root) and transmit_occgrid
			int psdu_len = 0;
			size_t psdu_len_sz = sizeof(int);
			uint8_t pckt_hdr_out[64];
			size_t pckt_hdr_out_sz = 64;
			int pckt_hdr_len = 0;
			size_t pckt_hdr_len_sz = sizeof(int);
			float msg_stream_real[MAX_SIZE];
			size_t msg_stream_real_sz = MAX_SIZE * sizeof(float);
			float msg_stream_imag[MAX_SIZE];
			size_t msg_stream_imag_sz = MAX_SIZE * sizeof(float);
			float ofdm_car_str_real[ofdm_max_out_size];
			size_t ofdm_car_str_real_sz = ofdm_max_out_size * sizeof(float);
			float ofdm_car_str_imag[ofdm_max_out_size];
			size_t ofdm_car_str_imag_sz = ofdm_max_out_size * sizeof(float);
			int ofc_res = 0;
			size_t ofc_res_sz = sizeof(int);
			float fft_out_real[ofdm_max_out_size];
			size_t fft_out_real_sz = ofdm_max_out_size * sizeof(float);
			float fft_out_imag[ofdm_max_out_size];
			size_t fft_out_imag_sz = ofdm_max_out_size * sizeof(float);
			float cycpref_out_real[41360];
			size_t cycpref_out_real_sz = 41360 * sizeof(float);
			float cycpref_out_imag[41360];
			size_t cycpref_out_imag_sz = 41360 * sizeof(float);
			// End of arguments to do_xmit_pipeline (called by lidar_root) and transmit_occgrid


			printf("%s %d Calling lidar_root", __FILE__, __LINE__);

#if (defined(HPVM) && defined(HPVM_PROCESS_LIDAR)) && true
			void * lidarDAG = __hetero_launch((void *) lidar_root, 32, &lidar_inputs, sizeof(lidar_inputs_t),
				&observationsArr[next_obs], sizeof(Observation),
				&n_cmp_bytes, sizeof(int),
				cmp_data, MAX_COMPRESSED_DATA_SIZE,
				// Global vars passed for internal use in lidar_root
				(int *) &curr_obs, sizeof(int),
				(int *) &next_obs, sizeof(int),
				(int *) &lidar_count, sizeof(unsigned),
				(int *) &lmap_count, sizeof(unsigned),
				// Args to cloudToOccgrid
				&AVxyzw, sizeof(double),
				&rolling_window, sizeof(bool),
				&min_obstacle_heght, sizeof(double),
				&max_obstacle_heght, sizeof(double),
				&raytrace_range, sizeof(double),
				&size_x, sizeof(unsigned int),
				&size_y, sizeof(unsigned int),
				&resolution, sizeof(unsigned int),
				// Args to sequentialized timer start and end calls along with the code being time
				&timer_sequentialize, sizeof(int),
				// Args for encode_occgrid
				&n_xmit_out, n_xmit_out_sz,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz,
				// Start of local variables for do_xmit_pipeline
				&psdu_len, psdu_len_sz,
				pckt_hdr_out, pckt_hdr_out_sz,
				&pckt_hdr_len, pckt_hdr_len_sz,
				msg_stream_real, msg_stream_real_sz,
				msg_stream_imag, msg_stream_imag_sz,
				ofdm_car_str_real, ofdm_car_str_real_sz,
				ofdm_car_str_imag, ofdm_car_str_imag_sz,
				&ofc_res, ofc_res_sz,
				fft_out_real, fft_out_real_sz,
				fft_out_imag, fft_out_imag_sz,
				cycpref_out_real, cycpref_out_real_sz,
				cycpref_out_imag, cycpref_out_imag_sz,
				// Outputs
				3, &n_cmp_bytes, sizeof(int),
				cmp_data, MAX_COMPRESSED_DATA_SIZE,
				&observationsArr[next_obs], sizeof(Observation)
				/*&n_xmit_out, n_xmit_out_sz,
					xmit_out_real, xmit_out_real_sz,
					xmit_out_imag, xmit_out_imag_sz*/
			);
			__hetero_wait(lidarDAG);
#else
			lidar_root(&lidar_inputs, sizeof(lidar_inputs_t),
				&observationsArr[next_obs], sizeof(Observation),
				&n_cmp_bytes, sizeof(int),
				cmp_data, MAX_COMPRESSED_DATA_SIZE,
				// Global vars passed for internal use in lidar_root
				(int *) &curr_obs, sizeof(int),
				(int *) &next_obs, sizeof(int),
				(int *) &lidar_count, sizeof(unsigned),
				(int *) &lmap_count, sizeof(unsigned),
				// Args to cloudToOccgrid
				&AVxyzw, sizeof(double),
				&rolling_window, sizeof(bool),
				&min_obstacle_heght, sizeof(double),
				&max_obstacle_heght, sizeof(double),
				&raytrace_range, sizeof(double),
				&size_x, sizeof(unsigned int),
				&size_y, sizeof(unsigned int),
				&resolution, sizeof(unsigned int),
				// Args to sequentialized timer start and end calls along with the code being time
				&timer_sequentialize, sizeof(int),
				// Args for encode_occgrid
				&n_xmit_out, n_xmit_out_sz,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz,
				// Start of local variables for do_xmit_pipeline
				&psdu_len, psdu_len_sz,
				pckt_hdr_out, pckt_hdr_out_sz,
				&pckt_hdr_len, pckt_hdr_len_sz,
				msg_stream_real, msg_stream_real_sz,
				msg_stream_imag, msg_stream_imag_sz,
				ofdm_car_str_real, ofdm_car_str_real_sz,
				ofdm_car_str_imag, ofdm_car_str_imag_sz,
				&ofc_res, ofc_res_sz,
				fft_out_real, fft_out_real_sz,
				fft_out_imag, fft_out_imag_sz,
				cycpref_out_real, cycpref_out_real_sz,
				cycpref_out_imag, cycpref_out_imag_sz);
#endif

			printf("%s %d Calling transmit occgrid ", __FILE__, __LINE__);


			// Send the occgrid through the socket
			transmit_occgrid(&n_cmp_bytes, n_cmp_bytes_sz,
				cmp_data, cmp_data_sz,
				n_xmit_out,
				xmit_out_real, xmit_out_real_sz,
				xmit_out_imag, xmit_out_imag_sz,
				&psdu_len, psdu_len_sz,
				pckt_hdr_out, pckt_hdr_out_sz,
				&pckt_hdr_len, pckt_hdr_len_sz,
				msg_stream_real, msg_stream_real_sz,
				msg_stream_imag, msg_stream_imag_sz,
				ofdm_car_str_real, ofdm_car_str_real_sz,
				ofdm_car_str_imag, ofdm_car_str_imag_sz,
				&ofc_res, ofc_res_sz,
				fft_out_real, fft_out_real_sz,
				fft_out_imag, fft_out_imag_sz,
				cycpref_out_real, cycpref_out_real_sz,
				cycpref_out_imag, cycpref_out_imag_sz);
#if PARALLEL_PTHREADS
			; // nothing to do here...
#else

			printf("%s %d Calling recieve_and_fuse_maps ", __FILE__, __LINE__);

			receive_and_fuse_maps(NULL, 0);
			DBGOUT(printf("Returning from process_lidar_to_occgrid\n"); fflush(stdout));
#endif
			DBGOUT(printf("Back from process_lidar_to_occgrid for Lidar\n"); fflush(stdout));
			lidar_count++;
			//#ifdef INT_TIME
			gettimeofday(&stop_proc_lidar, NULL);
			proc_data_sec += stop_proc_lidar.tv_sec - start_proc_data.tv_sec;
			proc_data_usec += stop_proc_lidar.tv_usec - start_proc_data.tv_usec;
			proc_lidar_sec += stop_proc_lidar.tv_sec - start_proc_lidar.tv_sec;
			proc_lidar_usec += stop_proc_lidar.tv_usec - start_proc_lidar.tv_usec;
			//#endif
		}
		/***********************************************************************/
		/* Checking if the received message includes odometry information      */
		/***********************************************************************/
		else if (l_buffer[0] == 'O' && l_buffer[9] == 'O') {

#ifdef INT_TIME
			gettimeofday(&start_proc_odo, NULL);
#endif
			char * ptr;
			int message_size = strtol((char *) l_buffer + 1, &ptr, 10);
			DBGOUT(printf("Odometry: expecting message size: %d\n", message_size));
			send(bag_sock, ack, 2, 0);

			int total_bytes_read = 0;

			// valread = read(bag_sock , buffer, 10000);
			valread = read_all(bag_sock, (char *) l_buffer, message_size);
			DBGOUT(printf("read %d bytes\n", valread));
			if (valread < message_size) {
				if (valread == 0) {
					printf("  Odo read got ZERO bytes -- END of TRANSFER?\n");
					closeout_and_exit("Odometry read got zero bytes...", -1);
				}
				else {
					printf("  Odo read got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, message_size);
					closeout_and_exit("Odometry read got too few bytes...", -1);
				}
			}

			odometry[0] = *((float *) (l_buffer));
			odometry[1] = *((float *) (l_buffer + 4));
			odometry[2] = *((float *) (l_buffer + 8));
			printf("Odometry msg %4u: %.2f %.2f %.2f\n", odo_count, odometry[0], odometry[1], odometry[2]);
			odo_count++;
#ifdef INT_TIME
			gettimeofday(&stop_proc_odo, NULL);
			proc_odo_sec += stop_proc_odo.tv_sec - start_proc_odo.tv_sec;
			proc_odo_usec += stop_proc_odo.tv_usec - start_proc_odo.tv_usec;
#endif
		}
		else {

			/*DBGOUT(printf("BUFFER : '");
				for (int ii = 0; ii < 8; ii++) {
				printf("%c", l_buffer[ii]);
				}
				printf("'\n");
				fflush(stdout));*/
		}

		/***********************************************************************/
		/* We next execute the computer vision task:                           */
		/*   - We use YOLO on images coming from a source TBD.                 */
		/*   - For testing purposes, these images can be taken initially from  */
		/*     the ATR dataset, COCO dataset, or some other available one.     */
		/*   - Ideally, we want these images to come from the actual car       */
		/*     simulator (e.g. CARLA).                                         */
		/***********************************************************************/

#ifdef CV_PIPELINE
#ifdef USE_OLD_MODEL

#ifdef INT_TIME
		gettimeofday(&start_proc_cv, NULL);
#endif

		// HPVM task
		label_t out_label;
		unsigned tr_val = 1;  // TODO: What is the parameter 'tr_val' passed to  run_object_classification()?
#if (defined(HPVM) && defined(HPVM_CV_ROOT))
		void * dfg = __hetero_launch((void *) cv_root_wrapper, 2, tr_val, &out_label, sizeof(out_label),
			1, &out_label, sizeof(out_label));
		__hetero_wait(dfg);
#else
		cv_root(tr_val, &out_label, sizeof(label_t));
#endif
		cv_count++;
#ifdef INT_TIME
		gettimeofday(&stop_proc_cv, NULL);
		proc_cv_sec += stop_proc_cv.tv_sec - start_proc_cv.tv_sec;
		proc_cv_usec += stop_proc_cv.tv_usec - start_proc_cv.tv_usec;
		printf("run_object_classification time in usec %ld\n",
			(stop_proc_cv.tv_sec - start_proc_cv.tv_sec) * 1000000 + (stop_proc_cv.tv_usec -
				start_proc_cv.tv_usec));
#endif
		printf("run_object_classification returned %u\n", out_label);
#else

		/*****************************************************************************/
		/* NEW: PyTorch TinyYOLOv2 support (May 2022)                                */
		int width, height, channels;
		uint8_t * rgb_image = stbi_load("/dccstor/epochs/aporvaa/hetero_era/test.jpg", &width, &height, &channels, 3);

		if (rgb_image != NULL) {

			char filename[300];
			int nboxes = 0;
			snprintf(filename, 270, "test_output.jpg");

			dim_t dimensions;
			dimensions.width = width;
			dimensions.height = height;
			dimensions.c = channels;

			detection_t * dets;
			//dets = run_object_classification(rgb_image, dimensions, filename, &nboxes);
#if (defined(HPVM) && defined(HPVM_CV_ROOT))
			size_t rgb_image_sz = width * height * channels;
			void * dfg = __hetero_launch((void *) cv_root_wrapper, 5, rgb_image, rgb_image_sz,
				&dimensions, sizeof(dim_t), filename, (size_t) 500,
				&nboxes, sizeof(int), dets, sizeof(detection_t),
				0);
			__hetero_wait(dfg);
#else
			cv_root_wrapper(rgb_image, width * height * channels, &dimensions, sizeof(dim_t), filename, 500,
				&nboxes, sizeof(int), dets, sizeof(detection_t));
#endif

			if (dets == NULL)
				printf("run_object_classification failed (skipping this frame)\n");

			stbi_image_free(rgb_image);


		}
		else {
			printf("test.jpg image not found\n");
		}
		/*****************************************************************************/
#endif
#endif // CV_PIPELINE

	}

	dump_final_run_statistics();
	close(bag_sock);
	close(xmit_sock);
	close(recv_sock);
}

void dump_final_run_statistics() {
	//printf("\nFinal Run Stats, %u, Odo, %u, Lidar, %u, LMAP, %u, XMIT, %u, RECV, %u, CAR-SEND, %u, CV\n",
	//		odo_count, lidar_count, lmap_count, xmit_count, recv_count, car_send_count, cv_count);
	printf("\nFinal Run Stats, %u, Odo, %u, Lidar, %u, LMAP, %u, XMIT, %u, RECV, %u, CAR-SEND\n",
		odo_count, lidar_count, lmap_count, xmit_count, recv_count, car_send_count);
	printf("Occ-Map Dimensions, %u, by, %u, grid, res, %lf, ray_r, %u\n", GRID_MAP_X_DIM,
		GRID_MAP_Y_DIM, GRID_MAP_RESLTN, RAYTR_RANGE);

	printf("Timing (in usec):");
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
	uint64_t total_exec = (uint64_t) (stop_prog.tv_sec - start_prog.tv_sec) * 1000000 + (uint64_t) (stop_prog.tv_usec - start_prog.tv_usec);
	uint64_t proc_rdbag = (uint64_t) (proc_rdbag_sec) * 1000000 + (uint64_t) (proc_rdbag_usec);
	uint64_t proc_odo = (uint64_t) (proc_odo_sec) * 1000000 + (uint64_t) (proc_odo_usec);
	uint64_t proc_lidar = (uint64_t) (proc_lidar_sec) * 1000000 + (uint64_t) (proc_lidar_usec);
	uint64_t proc_data = (uint64_t) (proc_data_sec) * 1000000 + (uint64_t) (proc_data_usec);
	uint64_t proc_cv = (uint64_t) (proc_cv_sec) * 1000000 + (uint64_t) (proc_cv_usec);

#ifdef INT_TIME
	uint64_t pd_cloud2grid = (uint64_t) (pd_cloud2grid_sec) * 1000000 + (uint64_t) (pd_cloud2grid_usec);
	uint64_t pd_lz4_cmp = (uint64_t) (pd_lz4_cmp_sec) * 1000000 + (uint64_t) (pd_lz4_cmp_usec);
	uint64_t pd_wifi_pipe = (uint64_t) (pd_wifi_pipe_sec) * 1000000 + (uint64_t) (pd_wifi_pipe_usec);
	uint64_t pd_wifi_send = (uint64_t) (pd_wifi_send_sec) * 1000000 + (uint64_t) (pd_wifi_send_usec);
	uint64_t pd_wifi_send_rl = (uint64_t) (pd_wifi_send_rl_sec) * 1000000 + (uint64_t) (pd_wifi_send_rl_usec);
	uint64_t pd_wifi_send_im = (uint64_t) (pd_wifi_send_im_sec) * 1000000 + (uint64_t) (pd_wifi_send_im_usec);
	uint64_t pd_wifi_recv_th = (uint64_t) (pd_wifi_recv_th_sec) * 1000000 + (uint64_t) (pd_wifi_recv_th_usec);
	uint64_t pd_wifi_lmap_wait = (uint64_t) (pd_wifi_lmap_wait_sec) * 1000000 + (uint64_t) (pd_wifi_lmap_wait_usec);
	uint64_t pd_wifi_recv_wait = (uint64_t) (pd_wifi_recv_wait_sec) * 1000000 + (uint64_t) (pd_wifi_recv_wait_usec);
	uint64_t pd_wifi_recv_all = (uint64_t) (pd_wifi_recv_all_sec) * 1000000 + (uint64_t) (pd_wifi_recv_all_usec);
	uint64_t pd_wifi_recv_rl = (uint64_t) (pd_wifi_recv_rl_sec) * 1000000 + (uint64_t) (pd_wifi_recv_rl_usec);
	uint64_t pd_wifi_recv_im = (uint64_t) (pd_wifi_recv_im_sec) * 1000000 + (uint64_t) (pd_wifi_recv_im_usec);
	uint64_t pd_recv_pipe = (uint64_t) (pd_recv_pipe_sec) * 1000000 + (uint64_t) (pd_recv_pipe_usec);
	uint64_t pd_lz4_uncmp = (uint64_t) (pd_lz4_uncmp_sec) * 1000000 + (uint64_t) (pd_lz4_uncmp_usec);
	uint64_t pd_combGrids = (uint64_t) (pd_combGrids_sec) * 1000000 + (uint64_t) (pd_combGrids_usec);
	uint64_t pd_carSend = (uint64_t) (pd_wifi_car_sec) * 1000000 + (uint64_t) (pd_wifi_car_usec);

	// This is the cloud2grid breakdown
	uint64_t ocgr_cl2g_total = (uint64_t) (ocgr_c2g_total_sec) * 1000000 + (uint64_t) (ocgr_c2g_total_usec);
	uint64_t ocgr_cl2g_initCM = (uint64_t) (ocgr_c2g_initCM_sec) * 1000000 + (uint64_t) (ocgr_c2g_initCM_usec);
	uint64_t ocgr_cl2g_updOrig = (uint64_t) (ocgr_c2g_updOrig_sec) * 1000000 + (uint64_t) (ocgr_c2g_updOrig_usec);
	uint64_t ocgr_cl2g_updBnds = (uint64_t) (ocgr_c2g_updBnds_sec) * 1000000 + (uint64_t) (ocgr_c2g_updBnds_usec);

	uint64_t ocgr_upBd_total = (uint64_t) (ocgr_upBd_total_sec) * 1000000 + (uint64_t) (ocgr_upBd_total_usec);
	uint64_t ocgr_upBd_rayFSp = (uint64_t) (ocgr_upBd_rayFSp_sec) * 1000000 + (uint64_t) (ocgr_upBd_rayFSp_usec);
	uint64_t ocgr_upBd_regObst = (uint64_t) (ocgr_upBd_regObst_sec) * 1000000 + (uint64_t) (ocgr_upBd_regObst_usec);

	/** No need to do this here -- provides no more info than exterior measure, really
		uint64_t ocgr_ryFS_total  = (uint64_t)(ocgr_ryFS_total_sec)  * 1000000 + (uint64_t)(ocgr_ryFS_total_usec);
		uint64_t ocgr_ryFS_rtLine = (uint64_t)(ocgr_ryFS_rtLine_sec) * 1000000 + (uint64_t)(ocgr_ryFS_rtLine_usec); **/

		// This is the xmit_pipe.c breakdown
	uint64_t x_pipe = (uint64_t) (x_pipe_sec) * 1000000 + (uint64_t) (x_pipe_usec);
	uint64_t x_genmacfr = (uint64_t) (x_genmacfr_sec) * 1000000 + (uint64_t) (x_genmacfr_usec);
	uint64_t x_domapwk = (uint64_t) (x_domapwk_sec) * 1000000 + (uint64_t) (x_domapwk_usec);
	uint64_t x_phdrgen = (uint64_t) (x_phdrgen_sec) * 1000000 + (uint64_t) (x_phdrgen_usec);
	uint64_t x_ck2sym = (uint64_t) (x_ck2sym_sec) * 1000000 + (uint64_t) (x_ck2sym_usec);
	uint64_t x_ocaralloc = (uint64_t) (x_ocaralloc_sec) * 1000000 + (uint64_t) (x_ocaralloc_usec);
	uint64_t x_fft = (uint64_t) (x_fft_sec) * 1000000 + (uint64_t) (x_fft_usec);
	uint64_t x_ocycpref = (uint64_t) (x_ocycpref_sec) * 1000000 + (uint64_t) (x_ocycpref_usec);

#ifdef XMIT_HW_FFT
	uint64_t x_fHtotal = (uint64_t) (x_fHtotal_sec) * 1000000 + (uint64_t) (x_fHtotal_usec);
	uint64_t x_fHcvtin = (uint64_t) (x_fHcvtin_sec) * 1000000 + (uint64_t) (x_fHcvtin_usec);
	uint64_t x_fHcomp = (uint64_t) (x_fHcomp_sec) * 1000000 + (uint64_t) (x_fHcomp_usec);
	uint64_t x_fHcvtout = (uint64_t) (x_fHcvtout_sec) * 1000000 + (uint64_t) (x_fHcvtout_usec);
#endif

	// This is the Xmit doMapWork breakdown
	uint64_t xdmw_total = (uint64_t) (xdmw_total_sec) * 1000000 + (uint64_t) (xdmw_total_usec);
	uint64_t xdmw_cnvEnc = (uint64_t) (xdmw_cnvEnc_sec) * 1000000 + (uint64_t) (xdmw_cnvEnc_usec);
	uint64_t xdmw_punct = (uint64_t) (xdmw_punct_sec) * 1000000 + (uint64_t) (xdmw_punct_usec);
	uint64_t xdmw_intlv = (uint64_t) (xdmw_intlv_sec) * 1000000 + (uint64_t) (xdmw_intlv_usec);
	uint64_t xdmw_symbols = (uint64_t) (xdmw_symbls_sec) * 1000000 + (uint64_t) (xdmw_symbls_usec);
	uint64_t xdmw_mapout = (uint64_t) (xdmw_mapout_sec) * 1000000 + (uint64_t) (xdmw_mapout_usec);

	// This is the recv_pipe.c breakdown
	uint64_t r_pipe = (uint64_t) (r_pipe_sec) * 1000000 + (uint64_t) (r_pipe_usec);
	uint64_t r_cmpcnj = (uint64_t) (r_cmpcnj_sec) * 1000000 + (uint64_t) (r_cmpcnj_usec);
	uint64_t r_cmpmpy = (uint64_t) (r_cmpmpy_sec) * 1000000 + (uint64_t) (r_cmpmpy_usec);
	uint64_t r_firc = (uint64_t) (r_firc_sec) * 1000000 + (uint64_t) (r_firc_usec);
	uint64_t r_cmpmag = (uint64_t) (r_cmpmag_sec) * 1000000 + (uint64_t) (r_cmpmag_usec);
	uint64_t r_cmpmag2 = (uint64_t) (r_cmpmag2_sec) * 1000000 + (uint64_t) (r_cmpmag2_usec);
	uint64_t r_fir = (uint64_t) (r_fir_sec) * 1000000 + (uint64_t) (r_fir_usec);
	uint64_t r_div = (uint64_t) (r_div_sec) * 1000000 + (uint64_t) (r_div_usec);
	uint64_t r_sshort = (uint64_t) (r_sshort_sec) * 1000000 + (uint64_t) (r_sshort_usec);
	uint64_t r_slong = (uint64_t) (r_slong_sec) * 1000000 + (uint64_t) (r_slong_usec);
	uint64_t r_fft = (uint64_t) (r_fft_sec) * 1000000 + (uint64_t) (r_fft_usec);
	uint64_t r_eqlz = (uint64_t) (r_eqlz_sec) * 1000000 + (uint64_t) (r_eqlz_usec);
	uint64_t r_decsignl = (uint64_t) (r_decsignl_sec) * 1000000 + (uint64_t) (r_decsignl_usec);
	uint64_t r_descrmbl = (uint64_t) (r_descrmbl_sec) * 1000000 + (uint64_t) (r_descrmbl_usec);

	// This is the receiver Hardware FFT breakdown
#ifdef RECV_HW_FFT
	uint64_t r_fHtotal = (uint64_t) (r_fHtotal_sec) * 1000000 + (uint64_t) (r_fHtotal_usec);
	uint64_t r_fHcvtin = (uint64_t) (r_fHcvtin_sec) * 1000000 + (uint64_t) (r_fHcvtin_usec);
	uint64_t r_fHcomp = (uint64_t) (r_fHcomp_sec) * 1000000 + (uint64_t) (r_fHcomp_usec);
	uint64_t r_fHcvtout = (uint64_t) (r_fHcvtout_sec) * 1000000 + (uint64_t) (r_fHcvtout_usec);
#endif

	// This is the sync_short.c "equalize" breakdown
	uint64_t rssh_total = (uint64_t) (sysh_total_sec) * 1000000 + (uint64_t) (sysh_total_usec);
	uint64_t rssh_search = (uint64_t) (sysh_search_sec) * 1000000 + (uint64_t) (sysh_search_usec);
	uint64_t rssh_frame = (uint64_t) (sysh_frame_sec) * 1000000 + (uint64_t) (sysh_frame_usec);

	// This is the synch_long.c "equalize" breakdown
	uint64_t rslg_total = (uint64_t) (sylg_total_sec) * 1000000 + (uint64_t) (sylg_total_usec);
	uint64_t rslg_firG = (uint64_t) (sylg_firG_sec) * 1000000 + (uint64_t) (sylg_firG_usec);
	uint64_t rslg_search = (uint64_t) (sylg_search_sec) * 1000000 + (uint64_t) (sylg_search_usec);
	uint64_t rslg_outgen = (uint64_t) (sylg_outgen_sec) * 1000000 + (uint64_t) (sylg_outgen_usec);

	// This is the gr_equalizer.c "equalize" breakdown
	uint64_t reql_total = (uint64_t) (reql_total_sec) * 1000000 + (uint64_t) (reql_total_usec);
	uint64_t reql_sym_set = (uint64_t) (reql_symset_sec) * 1000000 + (uint64_t) (reql_symset_usec);
	uint64_t reql_ls_eql = (uint64_t) (reql_lseq_call_sec) * 1000000 + (uint64_t) (reql_lseq_call_usec);
	uint64_t reql_out_sym = (uint64_t) (reql_outsym_sec) * 1000000 + (uint64_t) (reql_outsym_usec);
	uint64_t reql_ds_fld = (uint64_t) (reql_decSF_sec) * 1000000 + (uint64_t) (reql_decSF_usec);

	// This is the ofdm.c decode-signal breakdown
	uint64_t rdec_total = (uint64_t) (rdec_total_sec) * 1000000 + (uint64_t) (rdec_total_usec);
	uint64_t rdec_map_bitr = (uint64_t) (rdec_map_bitr_sec) * 1000000 + (uint64_t) (rdec_map_bitr_usec);
	uint64_t rdec_get_bits = (uint64_t) (rdec_get_bits_sec) * 1000000 + (uint64_t) (rdec_get_bits_usec);
	uint64_t rdec_dec_call = (uint64_t) (rdec_dec_call_sec) * 1000000 + (uint64_t) (rdec_dec_call_usec);
#endif
	printf(" Total workload main-loop : %10lu usec\n", total_exec);
	printf("   Total proc Read-Bag      : %10lu usec\n", proc_rdbag);
	printf("   Total proc Odometry      : %10lu usec\n", proc_odo);
	printf("   Total proc Lidar         : %10lu usec\n", proc_lidar);
	printf("     Total proc Data          : %10lu usec\n", proc_data);
	printf("     Total proc CV          : %10lu usec\n", proc_cv);
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

#undef HPVM

