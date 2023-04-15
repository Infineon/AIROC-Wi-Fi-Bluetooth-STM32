#
# Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#

#!/usr/bin/python

"""
A simple "tcp echo server" for demonstrating TCP usage.
The server listens for TCP packets and echoes any received
packets back to the originating host.

"""

import socket
import optparse
import time
import sys


def echo_server(host, port):
    print "TCP echo server"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        s.bind((host, port))
        s.listen(1)
    except socket.error, msg:
        print "ERROR: ", msg
        s.close()
        s = None

    if s is None:
        sys.exit(1)

    while 1:
        print "Listening on: %s:%d"%(host, port)
        data_len = 0
        index = 0
        try:
            conn, addr = s.accept()
        except KeyboardInterrupt:
            print "Closing Connection"
            s.close()
            s = None
            sys.exit(1)

        print 'Incoming connection accepted: ', addr
       
        try:
            while 1:
                conn.settimeout(30.0)            
                try:
                    data = conn.recv(4096)                    
                except:
                    # conn.send("Hello")
                    continue
                print time.strftime("%b %d %H:%M:%S ", time.localtime()), addr[0], ":", repr(data)
                print ""                        
                               
        except KeyboardInterrupt:
            print "Closing Connection"
            s.close()
            s = None
            sys.exit(1)

        conn.close()

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option("-p", "--port", dest="port", type="int", default=50007, help="Port to listen on [default: %default].")
    parser.add_option("--hostname", dest="hostname", default="", help="Hostname to listen on.")

    (options, args) = parser.parse_args()

    echo_server(options.hostname, options.port)
