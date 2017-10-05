#!/usr/bin/python
from scapy.all import *
from netfilterqueue import NetfilterQueue
import sys
import os
import time

print "\n[*] Redirect traffic to the NFQUEUE... \n"
os.system("ip link set dev enp0s3 promisc on")
os.system("iptables -A FORWARD -p tcp -d 192.168.1.164 -j NFQUEUE --queue-num 1")

def deny_camera_image(pkt):
    print(pkt)
    packet = IP(pkt.get_payload())

    #modify payload
    payload = str(packet[TCP].payload)
    
    if packet[TCP].flags==18 and packet[IP].len==1500:
        print payload
        # if payload contains image deny 
        pkt.deny()
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


