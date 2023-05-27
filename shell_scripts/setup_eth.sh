#!/bin/bash
sudo ip addr flush dev enp6s0 
sudo ip addr add 10.6.6.1/24 dev enp6s0 
sudo ip link set enp6s0 up
sudo ip addr show enp6s0
sudo dnsmasq -C /dev/null -kd -F 10.6.6.50,10.6.6.100 -i enp6s0 --bind-dynamic
