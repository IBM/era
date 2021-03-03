#!/usr/bin/python

import sys
import rosbag
import socket
import struct
import time
import argparse

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5556        # Port to listen on (non-privileged ports are > 1023)
INTERVAL = 0.1     # 10Hz = .1 seconds

bag_file = ""


def recvall(sock, n):
    # Helper function to recv all 'n' bytes of a message
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


def send_data(bag_file):
    global HOST
    global PORT
    global INTERVAL
    lidar_msg_count = 0
    odo_msg_count = 0
    blank_str = "        "

    bag = rosbag.Bag(bag_file)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #print('Calling s.bind for %s : %d' %(HOST, PORT))
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    print('Connected to ' + str(addr))
    print('USING INTERVAL ' + str(INTERVAL))
    starttime = time.time()
    for topic, msg, t in bag.read_messages(topics=['/carla/hero1/lidar/lidar1/point_cloud', '/carla/hero1/odometry']):
        #print('Got the next topic, msg "%s"' % topic)
        if topic == '/carla/hero1/lidar/lidar1/point_cloud' and msg.data:
            # Hold off to match the desired interval timing
            dtime = (INTERVAL - (time.time() - starttime))  # % INTERVAL
            print(dtime)
            if (dtime > 0):
                time.sleep(dtime)
            starttime = time.time()

            o1_str = str(len(msg.data))
            o2_str = o1_str + blank_str[len(o1_str):]
            o_str = 'L' + o2_str + 'L'
            #print('Sending Lidar message length msg "%s"' % o_str)
            conn.sendall(o_str.encode('utf-8'))
            data = recvall(conn, 2)
            #print('  received reply "%s"' % data)
            if data.decode() == 'OK':
                #print('  Sending Lidar message of %d bytes' % len(msg.data))
                conn.sendall(msg.data)
                print('Hero1 Lidar msg %d sent %d bytes' %
                      (lidar_msg_count, len(msg.data)))
                lidar_msg_count += 1
        elif topic == '/carla/hero1/odometry' and msg.pose:
            odometry = [msg.pose.pose.position.x,
                        msg.pose.pose.position.y, msg.pose.pose.position.z]

            ba = bytearray(struct.pack("f", msg.pose.pose.position.x)) + bytearray(struct.pack(
                "f", msg.pose.pose.position.y)) + bytearray(struct.pack("f", msg.pose.pose.position.z))
            o1_str = str(len(ba))
            o2_str = o1_str + blank_str[len(o1_str):]
            o_str = 'O' + o2_str + 'O'
            #print('Sending Odo message length msg "%s"' % o_str)
            conn.sendall(o_str.encode('utf-8'))
            data = recvall(conn, 2)
            #print('  received reply "%s"' % data)
            if data.decode() == 'OK':
                #print('  sending Odo message of %d bytes' % len(ba))
                conn.sendall(ba)
                print('Hero1 Odometry msg %d sent %d bytes' %
                      (odo_msg_count, len(ba)))
                odo_msg_count += 1
        else:
            print('Bagfile yielded non-topical content.')

    bag.close()


def main():
    global HOST
    global PORT
    global INTERVAL
    global bag_file
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bag_file", help="The name of the bag-file to use for AV 1")
    parser.add_argument("-A", "--address",
                        help="define the IP address to connect to")
    parser.add_argument("-P", "--port", type=int,
                        help="define the Port for the socket to connect to")
    parser.add_argument("-I", "--interval", type=float,
                        help="sets the interval time (rate) of Lidar arrivals")
    args = parser.parse_args()
    if (args.address != None):
        HOST = args.address
        #print('Set HOST to ' + HOST);
    if (args.port != None):
        PORT = args.port
        #print('Set PORT to %d' % PORT);
    if (args.interval != None):
        INTERVAL = args.interval
        #print('Set PORT to %d' % PORT);
    # for arg in sys.argv[1:]:
    #	bag_file = arg
    bag_file = args.bag_file
    print('Using HOST %s and PORT %d for file %s' % (HOST, PORT, bag_file))
    send_data(bag_file)


if __name__ == "__main__":
    main()
