#!/usr/bin/python

import sys
import socket
import struct
import argparse


def recvall(sock, n):
    # Helper function to recv all 'n' bytes of a message
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


def write_output_file(FNAME, O_type, O_count, d_data):
    ofnm = FNAME + '_'
    if (O_count < 10):
        ofnm = ofnm + '000'
    elif (O_count < 100):
        ofnm = ofnm + '00'
    elif (O_count < 1000):
        ofnm = ofnm + '0'
    ofnm = ofnm + str(O_count) + '.txt'
    with open(ofnm, 'w') as FILE:
        FILE.write('Fused CostMAP ' + str(O_count) + '\n')
        if (O_type == 0):
            for c in d_data:
                FILE.write(str(c) + '\n')
        # elif (O_type == 1) :
        # else:


def main():
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    RPORT = 5562         # Port to listen on & receive from
    FNAME = 'carla_era_out'  # base file-name for output files

    msg_count = 0
    O_TYPE = 0

    parser = argparse.ArgumentParser()
    parser.add_argument("-A", "--address",
                        help="define the IP address to connect to")
    parser.add_argument("-R", "--recv_port", type=int,
                        help="define the Port for the socket to receive from")
    parser.add_argument("-F", "--file_name", help="define the save-file name")
    # ; 1 = ascii; 2 = ppm")
    parser.add_argument("-O", "--out_type", type=int,
                        help="define the save-file content type; 0 = none, 1 = raw_data")

    args = parser.parse_args()
    if (args.address != None):
        HOST = args.address
        #print('Set HOST to ' + HOST);
    if (args.recv_port != None):
        RPORT = args.recv_port
        #print('Set RPORT to %d' % RPORT);
    if (args.file_name != None):
        FNAME = args.file_name
    if (args.out_type != None):
        O_TYPE = args.out_type

    if (O_TYPE > 0):
        print('ERROR : output type must be 0')
        exit

    print('Using HOST %s and R-PORT %u' % (HOST, RPORT))
    s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    s1.bind((HOST, RPORT))
    s1.listen(1)
    conn1, addr1 = s1.accept()
    print('Connected to ' + str(addr1))

    while True:
        header = recvall(conn1, 8)  # conn1.recv(8)
        if not header:
            print('... end of run on message %d.' % msg_count)
            break
        #print('Carla msg %d received %d bytes from port %d' % (msg_count, len(header), RPORT))
        #print('   msg: "%s"' % str(header))

        d_len = int(header[1:7])
        #print('    Therefore %d bytes for real/imag parts' % d_len)
        d_data = recvall(conn1, d_len)  # conn1.recv(d_len)
        if not d_data:
            print('... end of run on d_data of message %d.' % msg_count)
            break
        #print('Carla msg %d received %d bytes from port %d' % (msg_count, len(d_data), RPORT))
        #print('   msg: "%s"' % str(d_data))

        print('Carla received all msg-set %d from port %d payload %d bytes' %
              (msg_count, RPORT, len(d_data)))
        # Now save this to a file...
        if O_TYPE:
            write_output_file(FNAME, O_TYPE, msg_count, d_data)
        msg_count += 1


if __name__ == "__main__":
    main()
