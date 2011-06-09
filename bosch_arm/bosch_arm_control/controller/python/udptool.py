#!/usr/bin/env python
import socket
import sys


if len(sys.argv) == 3: #Client socket (send data)
	slaveIP = sys.argv[1]
	slavesocket = int(sys.argv[2])
	client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

	while 1:
		data = raw_input("send: ")
		if data == "q": exit(0)
		client_socket.sendto(data, (slaveIP,slavesocket))
	client_socket.close()

if len(sys.argv) == 2: #Server socket (receive data)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server_socket.bind(('', int(sys.argv[1])))
    print"UDPServer Waiting for client on port " + sys.argv[1]

    while 1:
	data, address = server_socket.recvfrom(256)
	print str(address[0]) + ":" + str(address[1]) + " said: " + data

else:
    print """Usage:\r
\tTo send:      udptool.py address socket\r
\tTo receive:   udptool.py socket\r"""
    exit(0)
