#!/usr/bin/python

import sys
import socket
import struct
import argparse

HOST  = '127.0.0.1'  # Standard loopback interface address (localhost)
RPORT = 5560         # Port to listen on & receive from
XPORT = 5559         # Port to listen on & send to

def recvall(sock, n):
        # Helper function to recv all 'n' bytes of a message
        data = bytearray()
        while len(data) < n :
                packet = sock.recv(n - len(data))
                if not packet:
                        return None
                data.extend(packet)
        return data
        

def main():
        msg_count = 0

        parser = argparse.ArgumentParser()
        parser.add_argument("-A", "--address", help="define the IP address to connect to")
        parser.add_argument("-R", "--recv_port", type=int, help="define the Port for the socket to receive from")
        parser.add_argument("-X", "--xmit_port", type=int, help="define the Port for the socket to transmit to")
        args = parser.parse_args()
        if (args.address != None) :
                HOST = args.address
                #print('Set HOST to ' + HOST);
        if (args.recv_port != None) :
                RPORT = args.recv_port
                #print('Set RPORT to %d' % RPORT);
        if (args.xmit_port != None) :
                XPORT = args.xmit_port
                #print('Set XPORT to %d' % XPORT);

        print('Using HOST %s and R-PORT %u and X-PORT %u' %(HOST, RPORT, XPORT));
	s1 =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	s1.bind((HOST, RPORT))
	s1.listen(1)
	conn1, addr1 = s1.accept()
	print('Connected to ' + str(addr1))

	s2 =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	s2.bind((HOST, XPORT))
	s2.listen(1)
	conn2, addr2 = s2.accept()
	print('Connected to ' + str(addr2))

        while True :
		header = recvall(conn1, 8) #conn1.recv(8)
                if not header :
                        print('... end of run on message %d.' % msg_count)
                        break
		#print('Wifi msg %d received %d bytes from port %d' % (msg_count, len(header), RPORT))
                #print('   msg: "%s"' % str(header))

                d_len = int(header[1:7])
                #print('    Therefore %d bytes for real/imag parts' % d_len)
		d_real = recvall(conn1, d_len) # conn1.recv(d_len)
                if not d_real :
                        print('... end of run on d_real of message %d.' % msg_count)
                        break
		#print('Wifi msg %d received %d bytes from port %d' % (msg_count, len(d_real), RPORT))
                #print('   msg: "%s"' % str(d_real))

		d_imag = recvall(conn1, d_len) # conn1.recv(d_len)
                if not d_imag :
                        print('... end of run on d_imag of message %d.' % msg_count)
                        break
		#print('Wifi msg %d received %d bytes from port %d' % (msg_count, len(d_imag), RPORT))
                #print('   msg: "%s"' % str(d_imag))
                print('Wifi received all msg-set %d from port %d payload %d bytes' % (msg_count, RPORT, len(d_real)))
		#print('Wfi sending messages %d to port %d' % (msg_count, XPORT))
                conn2.sendall(header)
                conn2.sendall(d_real)
                conn2.sendall(d_imag)
		print('Wfi sent all messages %d to port %d' %(msg_count, XPORT))
                msg_count += 1

if __name__ == "__main__":
	main()
