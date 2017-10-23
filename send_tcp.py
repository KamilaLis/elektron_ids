#!/usr/bin/python
from scapy.all import *
from scapy.utils import rdpcap

# rdpcap comes from scapy and loads in our pcap file
packets = rdpcap("/home/robot/pcaps/fabrykacja_z_atakujacego.pcapng")

# Let's iterate through every packet
for packet in packets:
    print(packet)
    sendp(packet)
    time.sleep(1.5)
