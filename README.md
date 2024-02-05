# chessrobo
Using UR5e to play chess 


## Simulation Setup of Chessboard on gazebo



### Requirements

For running each sample code:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/

### Setup

After installing the libraries needed to run the project. 

Setup the project:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd ..
catkin_make
cd src
git clone https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/
cd ..
catkin_make
source devel/setup.bash
```

### Usage

Launch the world
```
roslaunch chessrobot_simulation chessboard_world.launch 
```
