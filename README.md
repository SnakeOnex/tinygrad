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
