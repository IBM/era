#include "occgrid.h"

#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#define PORT 5555


void process_data(char* data, int data_size)
{





}


int main(int argc, char *argv[])
{

	int sock = 0;
	struct sockaddr_in servaddr;
	char *ack = "OK";
	unsigned char buffer[200000] = {0};

	// socket create and verification

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
	}

	close(sock);






}