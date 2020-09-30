#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <signal.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <arpa/inet.h> // for inet_addr

#include "globals.h"

#include "occgrid.h"    // Occupancy Grid Map Create/Fuse
#include "lz4.h"        // LZ4 Compression/Decompression
#include "xmit_pipe.h"  // IEEE 802.11p WiFi SDR Transmit Pipeline
#include "recv_pipe.h"  // IEEE 802.11p WiFi SDR Receive Pipeline

// The PORT is now defined in the compilation process, and comforms to the
// definition in the read_bag_x.py files.
//#define PORT 
//#define PORT1 5556
//#define PORT2 5557

int sock = 0;

float odometry[] = {0.0, 0.0, 0.0};

char pr_map_char[256] = {'.','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /*  16 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /*  32 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /*  48 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /*  64 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /*  80 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /*  96 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 112 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 128 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 144 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 160 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 176 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 192 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 208 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 224 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?','?','?',  /* 240 */
			 '?','?','?','?','?','?','?','?','?','?','?','?','?','?',' ','X'}; /* 256 */
void INThandler(int dummy)
{
  printf("In SIGINT INThandler -- Closing the connection and exiting\n");
  close(sock);
  exit(-1);
}

float bytes_to_float(unsigned char * bytes)
{
	unsigned char b[] = {bytes[0], bytes[1], bytes[2], bytes[3]};
	float f;
	memcpy(&f, &b, sizeof(f));

	return f;
}

int counter = 0;

void write_array_to_file(unsigned char * data, long size)
{
	const int dimx = 50, dimy = 50;
  int i, j;

  char file_name[32];

  snprintf(file_name,sizeof(char)*32, "image%d.ppm", counter);

  FILE *fp = fopen(file_name, "w");
  fprintf(fp, "P3 %d %d 255\n", dimx, dimy);

  for (j = 0; j < dimy*dimx; ++j) {
    fprintf(fp, " %d %d %d ", data[j], data[j], data[j]);
   }

  fclose(fp);
  counter++;
}

//#define MAX_GRID_SIZE   50 * 50  // Taken from size_x, size_y, resolution
#define MAX_UNCOMPRESSED_DATA_SIZE  sizeof(Costmap2D) // MAX_GRID_SIZE
#define MAX_COMPRESSED_DATA_SIZE    MAX_UNCOMPRESSED_DATA_SIZE //(In case of no compression)?  

#define MAX_XMIT_OUTPUTS  41800  // Really something like 41782 I think

// Setting these up as globals so I can test the combineGrids
//  against "historic" gridmaps (rather than the same one)
#define  RMAP_HIST_DEPTH  10
unsigned int  uncmp_count = 0;
unsigned char uncmp_data[RMAP_HIST_DEPTH][MAX_UNCOMPRESSED_DATA_SIZE];

void process_data(char* data, int data_size)
{
  DBGOUT(printf("Calling cloudToOccgrid...\n"));
	unsigned char * grid = cloudToOccgrid((float*)data, data_size/sizeof(float),
		odometry[0],odometry[1],odometry[2],1.5,
		false,
		0.05, 2.05,
		100,
	        100, 100, 2.0,  // size_x, size_y, resolution
		NO_INFORMATION);

	write_array_to_file(grid, 100/2.0*100/2.0);

	// Now we compress the grid for transmission...
	Costmap2D* local_map = &(master_observation.master_costmap);
	DBGOUT(printf("Calling LZ4_compress_default...\n");
	       printf("  Input CostMAP: AV x %lf y %lf z %lf\n", local_map->av_x, local_map->av_y, local_map->av_z);
	       printf("               : Cell_Size %lf X-Dim %u Y-Dim %u\n", local_map->cell_size, local_map->x_dim, local_map->y_dim);
	       printf("  ");
	       for (int ii = 0; ii < 50; ii++) {
		 for (int ij = 0; ij < 50; ij++) {
		   int idx = 50*ii + ij;
		   printf("%c", pr_map_char[local_map->costmap_[idx]]);
		 }
		 printf("\n  ");
	       }
	       printf("\n"));

	unsigned char cmp_data[MAX_COMPRESSED_DATA_SIZE];
	int n_cmp_bytes = LZ4_compress_default((char*)local_map, (char*)cmp_data, MAX_UNCOMPRESSED_DATA_SIZE, MAX_COMPRESSED_DATA_SIZE);
	double c_ratio = 100*(1-((double)(n_cmp_bytes)/(double)(MAX_UNCOMPRESSED_DATA_SIZE)));
	DBGOUT(printf("  Back from LZ4_compress_default: %lu bytes -> %u bytes for %5.2f%%\n", MAX_UNCOMPRESSED_DATA_SIZE, n_cmp_bytes, c_ratio));

	// Now we encode and transmit the grid...
	DBGOUT(printf("Calling do_xmit_pipeline for %u compressed grid elements\n", n_cmp_bytes));
	int n_xmit_out;
	float xmit_out_real[MAX_XMIT_OUTPUTS];
	float xmit_out_imag[MAX_XMIT_OUTPUTS];
	do_xmit_pipeline(n_cmp_bytes, cmp_data, &n_xmit_out, xmit_out_real, xmit_out_imag);
	DBGOUT(printf("  Back from do_xmit_pipeline with %u xmit outputs...\n", n_xmit_out));

	// This is now the content that should be sent out via IEEE 802.11p WiFi
	//  The n_xmit_out values of xmit_out_real and xmit_out_imag


	// If we receive a transmission, the process to turn it back into the gridMap is:
	int   n_recvd_in;
	float recvd_in_real[MAX_XMIT_OUTPUTS];
	float recvd_in_imag[MAX_XMIT_OUTPUTS];
	int   recvd_msg_len;
	unsigned char recvd_msg[1500]; // MAX size of original message in bytes
	//do_recv_pipeline(int n_recvd_in, recvd_in_real, recvd_in_imag, recvd_msg_len, recvd_msg)

	// Now we decompress the grid received via transmission...
	DBGOUT(printf("Calling LZ4_decompress_default...\n"));
	//unsigned char uncmp_data[MAX_UNCOMPRESSED_DATA_SIZE];
	//int dec_bytes = LZ4_decompress_safe((char*)recvd_msg, (char*)uncmp_data, n_recvd_in, MAX_UNCOMPRESSED_DATA_SIZE);
	unsigned int  uncmp_idx = uncmp_count % RMAP_HIST_DEPTH;
	unsigned int  rmap_idx  = (uncmp_count >= RMAP_HIST_DEPTH) ? (uncmp_idx + 1)%RMAP_HIST_DEPTH : 0;
	printf("uncmp_idx = %d :  %d    MOD %d\n", uncmp_idx, uncmp_count, RMAP_HIST_DEPTH);
	printf("rmap_idx  = %d : (%d+1) MOD %d = %d\n", rmap_idx, uncmp_idx,  RMAP_HIST_DEPTH, (uncmp_idx + 1)%RMAP_HIST_DEPTH);
	int dec_bytes = LZ4_decompress_safe((char*)cmp_data, (char*)uncmp_data[uncmp_idx++], n_cmp_bytes, MAX_UNCOMPRESSED_DATA_SIZE);
	uncmp_count++;

	Costmap2D* remote_map = (Costmap2D*)&(uncmp_data[rmap_idx]); // Convert "type" to Costmap2D
	DBGOUT(printf("  Back from LZ4_decompress_safe with %u decompressed bytes\n", dec_bytes);
	       printf("  Output CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
	       printf("                : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map->cell_size, remote_map->x_dim, remote_map->y_dim);
	       printf("  ");
	       for (int ii = 0; ii < 50; ii++) {
		 for (int ij = 0; ij < 50; ij++) {
		   int idx = 50*ii + ij;
		   printf("%c", pr_map_char[remote_map->costmap_[idx]]);
		 }
		 printf("\n  ");
	       }
	       printf("\n"));

	DBGOUT(printf("  Remote CostMAP: AV x %lf y %lf z %lf\n", remote_map->av_x, remote_map->av_y, remote_map->av_z);
	       printf("                : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map->cell_size, remote_map->x_dim, remote_map->y_dim);
	       printf("  ");
	       for (int ii = 0; ii < 50; ii++) {
		 for (int ij = 0; ij < 50; ij++) {
		   int idx = 50*ii + ij;
		   printf("%c", pr_map_char[remote_map->costmap_[idx]]);
		 }
		 printf("\n  ");
	       }
	       printf("\n"));

	// Then we should "Fuse" the received GridMap with our local one
	//  We need to "peel out" the remote odometry data from somewhere (in the message?)
	//unsigned char* combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
	//unsigned char *fusedMap = combineGrids(local_map, recvd_msg,
	//		                         odometry[0], odometry[1], // local_x,   local_y
	//		                         remote_x,    remote_y,    // remotel_x, remotel_y
	//                                       100, 100, 2.0,  // size_x, size_y, resolution
	//                                       ?? ); // def_val -- unused?
	DBGOUT(printf("\nCalling combineGrids: count %d u_idx %d r_idx %d\n", uncmp_count, uncmp_idx, rmap_idx));
	combineGrids(local_map->costmap_, remote_map->costmap_,
		     local_map->av_x, local_map->av_y,
		     remote_map->av_x, remote_map->av_y,
		     local_map->x_dim, local_map->y_dim, local_map->cell_size,
		     local_map->default_value);
	DBGOUT(printf("  Fused CostMAP : AV x %lf y %lf z %lf  IDX %d\n", remote_map->av_x, remote_map->av_y, remote_map->av_z, rmap_idx);
	       printf("                : Cell_Size %lf X-Dim %u Y-Dim %u\n", remote_map->cell_size, remote_map->x_dim, remote_map->y_dim);
	       printf("  ");
	       for (int ii = 0; ii < 50; ii++) {
		 for (int ij = 0; ij < 50; ij++) {
		   int idx = 50*ii + ij;
		   printf("%c", pr_map_char[remote_map->costmap_[idx]]);
		 }
		 printf("\n  ");
	       }
	       printf("\n"));

	DBGOUT(printf("Returning from process_buffer\n"));
	fflush(stdout);
}


int main(int argc, char *argv[])
{
	struct sockaddr_in servaddr;
	char *ack = "OK";
	unsigned char buffer[200000] = {0};

	xmit_pipe_init(); // Initialize the IEEE SDR Transmit Pipeline
	
	signal(SIGINT, INThandler);

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
		printf("Socket creation failed...\n");
		exit(0);
	}
	else {
		printf("Socket successfully created..\n");
	}

	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	servaddr.sin_port = htons(PORT);

	while (true) {

		if (connect(sock, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
			printf("connection with the server failed...\n");
			sleep(1);
			continue;
		}
		else {
			printf("connected to the server..\n");
			break;
		}
	}
	bool hit_eof = false;
	while (!hit_eof) {
	  DBGOUT(printf("Calling read on the socket...\n"); fflush(stdout));
		int valread = read(sock , buffer, 10);
		DBGOUT(printf("Top: read %d bytes\n", valread));
		if (valread == 0) {
		  // This means EOF?
		  hit_eof = true;
		}

		if(buffer[0] == 'L' && buffer[7] == 'L') {
			char * ptr;
			int message_size = strtol(buffer+1, &ptr, 10);
			DBGOUT(printf("Lidar: expecting message size: %d\n", message_size));
			send(sock, ack, 2, 0);

			char * message_ptr = buffer;
			int total_bytes_read = 0;
			while(total_bytes_read < message_size) {
				valread = read(sock , message_ptr, 10000);
				message_ptr = message_ptr + valread;
				total_bytes_read += valread;
				printf("read %d bytes for %d total bytes of %d\n", valread, total_bytes_read, message_size);
			}
			if (total_bytes_read > message_size) {
                          printf("NOTE: read more total bytes than expected: %u vs %u\n", total_bytes_read, message_size);
                        }
			DBGOUT(printf("Calling process_buffer for %d total bytes\n", total_bytes_read));
			process_data(buffer, total_bytes_read);
			DBGOUT(printf("Back from process_buffer for Lidar\n"));
			fflush(stdout);
		}
		else if(buffer[0] == 'O' && buffer[3] == 'O') {
			char * ptr;
			int message_size = strtol(buffer+1, &ptr, 10);
			DBGOUT(printf("Odometry: expecting message size: %d\n", message_size));
			send(sock, ack, 2, 0);

			int total_bytes_read = 0;

			valread = read(sock , buffer, 10000);
			DBGOUT(printf("read %d bytes\n", valread));


			odometry[0] = bytes_to_float(buffer);
			odometry[1] = bytes_to_float(buffer+4);
			odometry[2] = bytes_to_float(buffer+8);

			printf("odometry: %f %f %f\n", odometry[0], odometry[1], odometry[2]);

		} else {
		  /*DBGOUT(printf("BUFFER : '");
			 for (int ii = 0; ii < 8; ii++) {
			   printf("%c", buffer[ii]);
			 }
			 printf("'\n");
			 fflush(stdout));*/
		}

	}

	close(sock);
}
