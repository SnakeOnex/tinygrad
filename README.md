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
git submodule update --init --recursive
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
python -m pytest
```

## Links & Resources

- [Process communication](https://pymotw.com/2/multiprocessing/communication.html)
- [Shared Memory](https://docs.python.org/3/library/multiprocessing.shared_memory.html)
