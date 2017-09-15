#!/usr/bin/python
from scapy.all import *
from scapy.utils import rdpcap

ip=IP(src="192.168.1.241", dst="192.168.1.164")

'''SYN=TCP(sport=1050, dport=4000, flags="S", seq=100)
SYNACK=sr1(ip/SYN)

my_ack = SYNACK.seq + 1
ACK=TCP(sport=1050, dport=4000, flags="A", seq=101, ack=my_ack)
send(ip/ACK)

payload="stuff"
PUSH=TCP(sport=1050, dport=4000, flags="PA", seq=11, ack=my_ack)
send(ip/PUSH/payload)'''

#
SYN=sniff(count=1,filter="host 192.168.1.164 and tcp[tcpflags] & tcp-syn != 0")
print SYN[0].summary()

SYNACK = TCP(sport=SYN[0].dport, dport=SYN[0].sport, flags="SA", ack=SYN[0].seq+1)
ACK = sr1(ip/SYNACK)

HTTP=sniff(count=1, filter="host 192.168.1.164 and tcp")
print HTTP[0].summary()
'''pkt=HTTP[0]
my_ack=len(pkt[TCP].payload)+1
print my_ack
myACK=TCP(sport=HTTP[0].dport, dport=HTTP[0].sport, flags="A", seq=1 ,ack=my_ack)
send(ip/myACK)'''

registerTopic=rdpcap("/home/robot/pcaps/sendingControl/registerTopic.pcap")

myACK = registerTopic[4]
myACK[TCP].sport = sport=HTTP[0].dport
myACK[TCP].dport = sport=HTTP[0].sport
sendp(myACK)
print myACK.summary()

myHTTP = registerTopic[5]
myHTTP[TCP].sport = sport=HTTP[0].dport
myHTTP[TCP].dport = sport=HTTP[0].sport
sendp(myHTTP)
print myHTTP.summary()
