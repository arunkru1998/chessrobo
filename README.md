# chessrobo
Using UR5e to play chess 


## Simulation Setup of Chessboard and UR5e on gazebo


<img width="321" alt="image" src="https://github.com/arunkru1998/chessrobo/assets/114765006/51d7edb9-a888-471c-ba89-d17a0e4db8e4">
<img width="292" alt="robot" src="https://github.com/arunkru1998/chessrobo/assets/114765006/cf5b256c-9625-49ee-97fd-353b25b6acc9">


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
git clone https://github.com/arunkru1998/chessrobo
git clone -b noetic https://github.com/ros-industrial/universal_robot.git
cd ..
catkin_make
source devel/setup.bash
```

### Usage

Launch the world with only Chessboard
```
roslaunch chessrobot_simulation chessboard_world.launch 
```
Launch the world with Chessboard and Robot
```
roslaunch my_ur5_description my_ur5_bringup.launch 
```
## Setup Movit for UR5e on Gazebo

### Usage
![ezgif com-video-to-gif-converter](https://github.com/arunkru1998/chessrobo/assets/114765006/4d048b2e-d9fd-42a0-99ac-d42fe5c5c1d7)

Launch the world with UR5e along with RViz for motion control.
```
roslaunch my_ur5_description my_ur5_moveit_bringup.launch

Run the C++ code to control the robot to move to desired location
```
rosrun chessrobot_simulation pick_place_simple

![grasp](https://github.com/arunkru1998/chessrobo/assets/114765006/b62323f0-336b-4418-8ee7-98695029eed5)


