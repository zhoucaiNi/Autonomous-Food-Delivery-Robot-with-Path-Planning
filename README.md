# CS 81 Autonomous Food Delivery Robot with Path Planning
### Lauren Kidman, Mehmet Eren Aldemir, Zhoucai Ni
This repo tackles the challenge of creating an autonomous food delivery robot specifically designed for the Dartmouth College campus. Navigating a rural university campus with very limited food choices compared to the student body population can be a difficult challenge. We integrated Google Maps API to accurately map the terrain and generate a grid representation of the environment. To optimize route planning and handling multiple deliveries, we employed the A* search algorithm, which provides an efficient and effective pathfinding solution. We also introduced a dynamic solution for the Traveling Salesman Problem, enabling multi-delivery planning and thus enhancing the delivery efficiency. 

# Files
`pathfinder.py` is the main file for rviz visualization for our implementation of multi-delivery planning. (A* and Traveling Salesman Problem)

Quick Demo:

https://github.com/zhoucaiNi/cs81-FinalProject/assets/53072233/c5d97d25-9dfd-46bb-81d5-553886c1fa93

`path_movment` is responsible for taking the path calculated by pathfinder.py and executing it on the robot in simulation. 
`path_planning` contains all the modules and classes for path planning
`map`   contains all the modules for map generation and attempted coordinate transformation

Directory
```bash
├── map
│   ├── get_map.py  
│   └── mapCoordinate.py
├── path_planning
│   ├── A_star.py
│   ├── __init__.py
│   ├── __init__.pyc
│   ├── delivery.py
│   ├── delivery.pyc
│   ├── grid.py
│   ├── grid.pyc
│   ├── node.py
│   ├── node.pyc
│   ├── pathfinder.pyc
│   ├── planner.py
│   ├── planner.pyc
│   ├── tsp.py
│   ├── tsp.pyc
│   └── tspTest.py
└── pathfinder.py
├── path_movement.py
```
## Instructions to run the program
make sure you have docker installed and the vnc-ros envirnment built. 

if you don't have the 2d package installed, you need you first run`apt install ros-melodic-nav2d-tutorials`
after that you need to run `roscore`

open another terminal and then run 
`rosrun map_server map_server setups/map.yml` for map visualization in rviz 

open another terminal and run `rviz`, to see the map and the poses you need to add them to the map using the button in the bottom left.  

To be able to move the robot you need to open another terminal and then run
`rosrun stage_ros stageros setups/map.world`
 
lastly, 
open another terminal in the src directory and run `python src/pathfinder.py` for rviz visualization. 

you should be able to see the implemented random feature on `http://localhost:8080/vnc.html`





