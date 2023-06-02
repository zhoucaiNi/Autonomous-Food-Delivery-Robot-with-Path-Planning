# CS 81 SnackBot
### Lauren Kidman, Mehmet Eren Aldemir, Zhoucai Ni

# Files

## Instructions to run the program
make sure you have docker installed and the vnc-ros envirnment built. 

if you don't have the 2d package installed, you need you first run`apt install ros-melodic-nav2d-tutorials`
after that you need to run `roscore`

open another terminal and then run 
`rosrun map_server map_server setups/map.yml` for map visualization in rviz 

open another terminal and run `rviz`, to see the map and the poses you need to add them to the map using the button in the bottom left.  

To be able to move the robot you need to open another terminal and then run
`rosrun stage_ros stageros setups/map.world`
 
lastly, open another terminal in the src directory and run`python src/pathfinder.py`

you should be able to see the implemented random feature on `http://localhost:8080/vnc.html`





