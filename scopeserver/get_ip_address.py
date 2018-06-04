#!/usr/bin/env python3

import socket
import fcntl
import struct

# Get IP address
def get_ip_address(ifname = b'wlan0'):
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
  except:
    ip = '127.0.0.1'
  return ip

ip = get_ip_address()
print(ip)
