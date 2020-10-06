#!/usr/bin/python

import sys
import rosbag
import socket
import struct
import argparse

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5556        # Port to listen on (non-privileged ports are > 1023)

bag_file = ""

def send_data(bag_file):
        lidar_msg_count = 0;
        odo_msg_count = 0;
        
	bag = rosbag.Bag(bag_file)

	s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	s.bind((HOST, PORT))
	s.listen(1)
	conn, addr = s.accept()
	print('Connected to ' + str(addr))
	for topic, msg, t in bag.read_messages(topics=['/carla/hero1/lidar/lidar1/point_cloud','/carla/hero1/odometry']):
		if topic == '/carla/hero1/lidar/lidar1/point_cloud' and msg.data :

			conn.sendall('L' + str(len(msg.data)) + 'L')
			data = conn.recv(10)
			#print(data)
			if data == 'OK' :
				conn.sendall(msg.data)
				print('Hero1 Lidar msg %d sent %d bytes' %(lidar_msg_count, len(msg.data)))
                                lidar_msg_count += 1
		if topic == '/carla/hero1/odometry' and msg.pose :
			odometry = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

			ba = bytearray(struct.pack("f", msg.pose.pose.position.x)) + bytearray(struct.pack("f", msg.pose.pose.position.y)) +	bytearray(struct.pack("f", msg.pose.pose.position.z))

			conn.sendall('O' + str(len(ba)) + 'O')
			data = conn.recv(10)
			#print(data)
			if data == 'OK' :
				conn.sendall(ba)
				print('Hero1 Odometry msg %d sent %d bytes' % (odo_msg_count, len(ba)))
                                odo_msg_count += 1


	bag.close();

def main():
	global bag_file
        parser = argparse.ArgumentParser()
        parser.add_argument("bag_file", help="The name of the bag-file to use for AV 1")
        parser.add_argument("-A", "--address", help="define the IP address to connect to")
        args = parser.parse_args()
        
	#for arg in sys.argv[1:]:
	#	bag_file = arg

	send_data(args.bag_file)

if __name__ == "__main__":
	main()
