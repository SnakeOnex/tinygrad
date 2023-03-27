# Bros

Better Robot Operating System. This repository contains the autonomous system, whose purpose is connecting the car's sensors and actuators with algorithms for perception, planning and controls to drive the car.

## Project structure

- **nodes/** - folder containing all the code for perception & planning
  - **vision_node.py** - code for perception and planning
  - **mission_node.py** - control loop to drive the car
  - **can1_node.py** - CAN1 recv/send communication process
  - **can2_node_.py** - CAN2 recv process
- **can/** - code for communication with the car's CAN networks and with the **dv_sim** simulator
- **missions/** - folder containing all the code for individual autonomous missions
- **master.py** - launch file for the whole system, responsible for selecting a mission and starting all necessary sub-processes

## Setup

**Clone and install submodules:**
``` 
git clone git@eforce1.feld.cvut.cz:eforce-driveless/bros.git &&
cd bros &&
./install.sh
```


**Run the BROS autonomous system:**
```
python master.py
```

**Launch simulation + 3D engine for visualization, in another terminal do:**
```
cd dv_sim && python run_simulation.py --gui --manual --map maps/skidpad_map.json
```

**Run tests:**
```
./dv_sim/setup_can.sh  # sets up virtual can buses
python -m pytest
```

## Links & Resources

- [Process communication](https://pymotw.com/2/multiprocessing/communication.html)
- [Shared Memory](https://docs.python.org/3/library/multiprocessing.shared_memory.html)

# FSD Simulator
Virtuální Milovice

## About
Simulator for testing of FSD autonomous systems written in python and using [ursina](https://www.ursinaengine.org) package for 3D rendering. It offers manual driving aswell as TCP connection to Autonomous system process receiving cone detection data and sending actuator commands. 

## Project Structure
- **models/** - 3D prop models & Texture files
- **objects/** - Python classes encapsulating rendering of individual objects (Formula, Cones, Tires)
- **maps/** - json files containing different map scenarios (cone positions, start/finish position)
- **main.py** - main, simulation rendering, inputs, AutonomousSystem communication
- **state.py** - all of world state, physics, sensors and actuators simulation
- **helpers.py** - general functions used throughout the project (local_to_global, global_to_local, vec_to_3d, etc.)

## Links & Resources
- [Ursina cheatsheet](https://www.ursinaengine.org/cheat_sheet.html)
- [Ursina documentation](https://www.ursinaengine.org/documentation.html)
- [AMZ-FSSIM](https://github.com/AMZ-Driverless/fssim)
- [ViewSTL](https://www.viewstl.com/)
- [Car Physics](https://asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html)
- [Vehicle Dynamics](https://core.ac.uk/download/pdf/128709302.pdf)
- [python-can](https://github.com/hardbyte/python-can)
