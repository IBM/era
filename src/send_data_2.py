#!/usr/bin/python

import sys
import rosbag
import socket
import struct
import time
import argparse

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5557        # Port to listen on (non-privileged ports are > 1023)
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

def create_connection():
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 5557        # Port to listen on (non-privileged ports are > 1023)
    INTERVAL = 0.1     # 10Hz = .1 seconds


    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #print('Calling s.bind for %s : %d' %(HOST, PORT))
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    print('Connected to ' + str(addr))
    print('USING INTERVAL ' + str(INTERVAL))

    return conn

def send_carla_sim_data(conn, infile):
    lidar_msg_count = 0
    odo_msg_count = 0
    blank_str = "        "

    starttime = time.time()
    # Process odometry data first
    # if topic == '/carla/hero2/odometry' and msg.pose:
    #     odometry = [msg.pose.pose.position.x,
    #                 msg.pose.pose.position.y, msg.pose.pose.position.z]

    #     ba = bytearray(struct.pack("f", msg.pose.pose.position.x)) + bytearray(struct.pack(
    #         "f", msg.pose.pose.position.y)) + bytearray(struct.pack("f", msg.pose.pose.position.z))
    #     o1_str = str(len(ba))
    #     o2_str = o1_str + blank_str[len(o1_str):]
    #     o_str = 'O' + o2_str + 'O'
    #     #print('Sending Odo message length msg "%s"' % o_str)
    #     conn.sendall(o_str.encode('utf-8'))
    #     data = recvall(conn, 2)
    #     #print('  received reply "%s"' % data)
    #     if data.decode() == 'OK':
    #         #print('  sending Odo message of %d bytes' % len(ba))
    #         conn.sendall(ba)
    #         print('Hero2 Odometry msg %d sent %d bytes' %
    #                 (odo_msg_count, len(ba)))
    #         odo_msg_count += 1

    #Process lidar data
    plydata = PlyData.read(infile)
    pc = plydata['vertex'].data
    pc_array = []
    for x,y,z,i in pc:
        pc_array.append(x)
        pc_array.append(y)
        pc_array.append(z)


    #  https://stackoverflow.com/questions/9940859/fastest-way-to-pack-a-list-of-floats-into-bytes-in-python
    msg = struct.pack('%sf' % len(pc_array), *pc_array)
    # print(msg)


    # Hold off to match the desired interval timing
    dtime = (INTERVAL - (time.time() - starttime))  # % INTERVAL
    print(dtime)
    if (dtime > 0):
        time.sleep(dtime)
    starttime = time.time()

    o1_str = str(len(msg))
    o2_str = o1_str + blank_str[len(o1_str):]
    o_str = 'L' + o2_str + 'L'
    #print('Sending Lidar message length msg "%s"' % o_str)
    conn.sendall(o_str.encode('utf-8'))
    data = recvall(conn, 2)
    #print('  received reply "%s"' % data)
    if data.decode() == 'OK':
        #print('  Sending Lidar message of %d bytes' % len(msg))
        conn.sendall(msg)
        print('Hero2 Lidar msg %d sent %d bytes' %
                (lidar_msg_count, len(msg)))
        lidar_msg_count += 1

 


def send_data(conn, msg):
    lidar_msg_count = 0
    odo_msg_count = 0
    blank_str = "        "

    if msg.pose:
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
            print('Hero2 Odometry msg %d sent %d bytes' %
                    (odo_msg_count, len(ba)))
            odo_msg_count += 1
    else:
        print('Bagfile yielded non-topical content.')


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

    conn = create_connection()
    bag = rosbag.Bag(bag_file)
    x = list(bag.read_messages(topics=['/carla/hero2/odometry']))

    for count in range(0, len(x)):
        topic, msg, t = next(bag.read_messages(topics=['/carla/hero2/odometry']))
        send_carla_sim_data(conn, '../carla_scenarios/lidar_out/lidar_data.ply')
        send_data(conn, msg)

    bag.close()




if __name__ == "__main__":
    main()
