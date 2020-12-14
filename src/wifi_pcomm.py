#!/usr/bin/python

import sys
import threading
import socket
import struct
import argparse

HOST  = '127.0.0.1'  # Standard loopback interface address (localhost)
RPORT = 5560         # Port to listen on & receive from
XPORT = 5559         # Port to listen on & send to

exitFlag = 0  # Indicate when program should end
glob_header = ""
glob_dreal = ""
glob_dimag = ""
glob_haveData = 0

exitLock = threading.Lock() # Used to mutex access to exitFlag
#xmsgLock = threading.Lock() # Used to mutex changes to the xmit-msg
hdrLock = threading.Lock()
#drealLock = threading.Lock()
#dimagLock = threading.Lock()


def recvall(sock, n):
        # Helper function to recv all 'n' bytes of a message
        data = bytearray()
        while len(data) < n :
                packet = sock.recv(n - len(data))
                if not packet:
                        return None
                data.extend(packet)
        return data
        

class xmitThread(threading.Thread):
        def __init__(self, threadId, name):
                threading.Thread.__init__(self)
                self.threadId = threadId
                self.name = name
                self.msg_count = 0
	        self.s2 =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	        self.s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                global HOST
                global RPORT
                print('Binding XMIT to %s port %s\n' %(HOST, XPORT))
	        self.s2.bind((HOST, XPORT))
	        self.s2.listen(1)
	        self.conn, self.addr = self.s2.accept()
	        print('Connected to ' + str(self.addr))

        def run(self):
                global exitFlag
                global glob_header
                global glob_dreal
                global glob_dimag
                global glob_haveData
                while not exitFlag:
                        if glob_haveData:
                                #print('Starting to transmit msgs %d\n' % self.msg_count)
                                hdrLock.acquire()
                                self.conn.sendall(glob_header)
                                #print('Sent glob_header "%s"' % glob_header)
                                self.conn.sendall(glob_dreal)
                                #print('Sent glob_dreal of %d' % len(glob_dreal))
                                self.conn.sendall(glob_dimag)
                                #print('Sent glob_dimag of %d' % len(glob_dimag))
                                glob_haveData = 0
                                hdrLock.release()
		                print('Wfi sent all messages %d to port %d' %(self.msg_count, XPORT))
                                self.msg_count += 1
                                
class recvThread(threading.Thread):
        def __init__(self, threadId, name):
                threading.Thread.__init__(self)
                self.threadId = threadId
                self.name = name
                self.msg_count = 0
	        self.s1 =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	        self.s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                global HOST
                global RPORT
                print('Binding RECV to %s port %s\n' %(HOST, RPORT))
	        self.s1.bind((HOST, RPORT))
	        self.s1.listen(1)
	        self.conn, self.addr = self.s1.accept()
	        print('Connected to ' + str(self.addr))

        def run(self):
                global exitFlag
                global glob_header
                global glob_dreal
                global glob_dimag
                global glob_haveData
                while not exitFlag:
                        header = recvall(self.conn, 8)
                        if not header :
                                exitLock.acquire()
                                exitFlag = 1
                                exitLock.release()
                                print('... end of run on message %d.' % self.msg_count)

		        #print('Wifi msg %d received %d bytes from port %d' % (self.msg_count, len(header), RPORT))
                        hdrLock.acquire()
                        glob_header = header
                        #hdrLock.release()
                        d_len = int(header[1:7])
                        #print('    Therefore %d bytes for real/imag parts' % d_len)
		        d_real = recvall(self.conn, d_len) # conn1.recv(d_len)
                        if not d_real :
                                exitLock.acquire()
                                exitFlag = 1
                                exitLock.release()
                                print('... end of run on d_real of message %d.' % self.msg_count)

                        #drealLock.acquire()
                        glob_dreal = d_real
                        #drealLock.release()
		        #print('Wifi msg %d received %d bytes from port %d' % (self.msg_count, len(d_real), RPORT))
		        d_imag = recvall(self.conn, d_len) # conn1.recv(d_len)
                        if not d_imag :
                                print('... end of run on d_imag of message %d.' % self.msg_count)
                                exitLock.acquire()
                                exitFlag = 1
                                exitLock.release()

                        #dimagLock.acquire()
                        glob_dimag = d_imag
                        #dimagLock.release()
                        glob_haveData = 1
                        hdrLock.release()
		        #print('Wifi msg %d received %d bytes from port %d' % (self.msg_count, len(d_imag), RPORT))
                        print('Wifi received msgs iter %d from port %d payload %d bytes' % (self.msg_count, RPORT, len(d_real)))
                        self.msg_count += 1

def main():
        global HOST
        global RPORT
        global XPORT
        parser = argparse.ArgumentParser()
        parser.add_argument("-A", "--address", help="define the IP address to connect to")
        parser.add_argument("-R", "--recv_port", type=int, help="define the Port for the socket to receive from")
        parser.add_argument("-X", "--xmit_port", type=int, help="define the Port for the socket to transmit to")
        args = parser.parse_args()
        if (args.address != None) :
                HOST = args.address
                #print('Set HOST to ' + HOST)
        if (args.recv_port != None) :
                RPORT = args.recv_port
                #print('Set RPORT to %d' % RPORT)
        if (args.xmit_port != None) :
                XPORT = args.xmit_port
                #print('Set XPORT to %d' % XPORT)

        print('Using HOST %s and R-PORT %u and X-PORT %u' %(HOST, RPORT, XPORT))


        rthread = recvThread(1, "R-Thread")
        xthread = xmitThread(2, "X-Thread")

        rthread.start()
        xthread.start()


        rthread.join()
        xthread.join()
        
if __name__ == "__main__":
	main()
