#!/usr/bin/python
from scapy.all import *
from netfilterqueue import NetfilterQueue

def modify(packet):
    pkt = IP(packet.get_payload()) #converts the raw packet to a scapy compatible string

    #modify the packet all you want here
    print str(pkt).encode("HEX")
    #pkt.sprintf(r"rosout...........Y...!........./talker_2531_1503399769421....hello robot 1503399818.55	...talker.py....talker............/chatter....")

    packet.set_payload(str(pkt)) #set the packet content to our modified version

    packet.accept() #accept the packet



nfqueue = NetfilterQueue()
#1 is the iptabels rule queue number, modify is the callback function
nfqueue.bind(1, modify) 
try:
    print "[*] waiting for data"
    nfqueue.run()
except KeyboardInterrupt:
    pass
