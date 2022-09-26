#!/bin/bash

CAN1_PORT="can0"
CAN2_PORT="can1"

sudo modprobe vcan
sudo ip link add dev $CAN1_PORT type vcan
sudo ip link set up $CAN1_PORT

sudo modprobe vcan
sudo ip link add dev $CAN2_PORT type vcan
sudo ip link set up $CAN2_PORT
