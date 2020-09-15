#!/usr/bin/python

import sys
import rosbag
import socket

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5555        # Port to listen on (non-privileged ports are > 1023)

bag_file = ""

def send_data(bag_file):

	bag = rosbag.Bag(bag_file)

	s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, PORT))
	s.listen(1)
	conn, addr = s.accept()
	print('Connected by', addr)
	for topic, msg, t in bag.read_messages('/carla/hero1/lidar/lidar1/point_cloud'):
		conn.sendall('L' + str(len(msg.data)) + 'L')
		data = conn.recv(10)
		print(data)
		if data == 'OK' :
			conn.sendall(msg.data)
			print('sent ', len(msg.data))

	bag.close();

def main():
	global bag_file
	for arg in sys.argv[1:]:
		bag_file = arg

	send_data(bag_file)

if __name__ == "__main__":
	main()