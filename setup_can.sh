#!/bin/bash

CAN1_PORT="CAN1_powertrain"
#CAN3_PORT="CAN3"

sudo modprobe vcan
sudo ip link add dev $CAN1_PORT type vcan
sudo ip link set up $CAN1_PORT
