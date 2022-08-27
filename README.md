# Bros

Better Robot Operating System. This repository contains the autonomous system, whose purpose is connecting the car's sensors and actuators with algorithms for perception, planning and controls to drive the car.

## Project structure

- **nodes/** - folder containing all the code for perception & planning
- **vision_node.py** - code for perception and planning
  - **can1_node.py** - CAN1 recv/send communication process
  - **can2_node_.py** - CAN2 recv process
- **can/** - code for communication with the car's CAN networks and with the **dv_sim** simulator
- **missions/** - folder containing all the code for individual autonomous missions
- **master.py** - launch file for the whole system, responsible for selecting a mission and starting all necessary sub-processes

## Installation

## Links & Resources

- [Process communication](https://pymotw.com/2/multiprocessing/communication.html)
- [Shared Memory](https://docs.python.org/3/library/multiprocessing.shared_memory.html)
