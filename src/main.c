#include "occgrid.h"

#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <signal.h>

#include "occgrid.h"

#define PORT 5556

int sock = 0;

float odometry[] = {0.0, 0.0, 0.0};

void INThandler(int dummy)
{
	printf("Closing the connection\n");
  close(sock);
  exit(0);
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

void process_data(char* data, int data_size)
{

	unsigned char * grid = cloudToOccgrid((float*)data, data_size/sizeof(float),
		odometry[0],odometry[1],odometry[2],1.5,
		false,
		0.05, 2.05,
		100,
		100, 100, 2.0,
		254);

	write_array_to_file(grid, 100/2.0*100/2.0);

}


int main(int argc, char *argv[])
{
	struct sockaddr_in servaddr;
	char *ack = "OK";
	unsigned char buffer[200000] = {0};


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

	while (true) {

		int valread = read(sock , buffer, 10);
		printf("read %d bytes\n", valread);

		if(buffer[0] == 'L' && buffer[7] == 'L') {

			int message_size = strtol(buffer+1, buffer+6, 10);
			printf("expecting message size: %d\n", message_size);
			send(sock, ack, 2, 0);

			char * message_ptr = buffer;
			int total_bytes_read = 0;
			while(total_bytes_read != message_size) {

				valread = read(sock , message_ptr, 10000);
				printf("read %d bytes\n", valread);

				message_ptr = message_ptr + valread;
				total_bytes_read += valread;
			}

			process_data(buffer, total_bytes_read);

		}

		if(buffer[0] == 'O' && buffer[3]) {
			int message_size = strtol(buffer+1, buffer+2, 10);
			printf("expecting message size: %d\n", message_size);
			send(sock, ack, 2, 0);

			int total_bytes_read = 0;

			valread = read(sock , buffer, 10000);
			printf("read %d bytes\n", valread);


			odometry[0] = bytes_to_float(buffer);
			odometry[1] = bytes_to_float(buffer+4);
			odometry[2] = bytes_to_float(buffer+8);

			printf("odometry: %f %f %f\n", odometry[0], odometry[1], odometry[2]);


		}

	}

	close(sock);
}