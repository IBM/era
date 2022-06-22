#!/usr/bin/python

import sys
import socket
import struct
import time
import argparse
from plyfile import PlyData

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

def create_connection(host, port):

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #print('Calling s.bind for %s : %d' %(HOST, PORT))
    s.bind((host, port))
    s.listen(1)
    conn, addr = s.accept()
    print('Connected to ' + str(addr))

    return conn

def send_lidar_data(conn, interval, pc_array):
    lidar_msg_count = 0
    odo_msg_count = 0
    blank_str = "        "

    starttime = time.time()

    #  https://stackoverflow.com/questions/9940859/fastest-way-to-pack-a-list-of-floats-into-bytes-in-python
    msg = struct.pack('%sf' % len(pc_array), *pc_array)
    # print(msg)


    # Hold off to match the desired interval timing
    dtime = (interval - (time.time() - starttime))  # % interval
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
        print('Hero1 Lidar msg %d sent %d bytes' %
                (lidar_msg_count, len(msg)))
        lidar_msg_count += 1

 


def send_odometry_data(conn, odometry):
    lidar_msg_count = 0
    odo_msg_count = 0
    blank_str = "        "

    if odometry:
        print(odometry)
        ba = bytearray(struct.pack("f", odometry[0])) + bytearray(struct.pack(
            "f", odometry[1])) + bytearray(struct.pack("f", odometry[2]))
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


def main():
    global HOST
    global PORT
    global INTERVAL
    global bag_file
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "lidar_ply_file", help="The name of the lidar-ply-file to use for AV 1")
    parser.add_argument(
        "odometry_csv_file", help="The name of the odometry-csv-file to use for AV 1")
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
    lidar_ply_file = args.lidar_ply_file
    odometry_csv_file = args.odometry_csv_file
    print('Using HOST %s and PORT %d for lidar file %s' % (HOST, PORT, lidar_ply_file))
    print('USING INTERVAL ' + str(INTERVAL))

    conn = create_connection(HOST, PORT)

    for count in range(0, 256):
        #Process lidar data
        plydata = PlyData.read(lidar_ply_file)
        pc = plydata['vertex'].data
        pc_array = []
        for x,y,z,i in pc:
            pc_array.append(x)
            pc_array.append(y)
            pc_array.append(z)

        #Process odometry data
        with open(odometry_csv_file) as file_name:
            location = np.ravel(np.loadtxt(file_name, delimiter=","))
        #Send lidar data    
        send_lidar_data(conn, INTERVAL, pc_array)

        #Send odometry data  
        send_odometry_data(conn, location)

    bag.close()




if __name__ == "__main__":
    main()
