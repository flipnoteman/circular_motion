## ROS-1 Circular Motion
A ROS-1 Noetic package to control a swarm of drones in [Malintha's Multi-UAV-Simulator](https://github.com/Malintha/multi_uav_simulator).
The goal is to have a system that will call any drones to circulate a given position. It should allow for any number of drones to join the circle at anytime.

### To Install:
Navigate to your catkin_ws sources folder and clone this repository:

```sh
cd ~/catkin_ws
git clone https://github.com/flipnoteman/circular_motion src
rm -rf .git # Optional
```

Building the project should work and then can be run by launching the provided simulation:

```sh
setup /devel/setup.bash
roslaunch circular_motion circ.launch
```
And then in a new window launch the publisher (MAY BE CHANGED IN THE FUTURE)
```sh
rosrun circular_motion circular_mover
```
