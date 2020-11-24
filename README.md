# Waypoints Server

This package allows send differents waypoints to the navigation stack of ROS (kinetic). It can be used with the GUI interface in RVIZ or calling their services.

## Installation

Clone and compile this package in your catkin_ws


## Usage

Launch your robot and the navigation stack of your choice.
After start the waypoints server

```
rosrun waypoints waypoints_server

```
### Create a waypoint by console
To create a new waypoint by console, you have to publish in the topic **/waypoint** giving the name and the pose of the waypoint.
```
rostopic pub /waypoint waypoints/waypoint_msg "name: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 
```

### Services
1. Create/delete waypoints
2. Create/delete a group of waypoints
3. Run waypoint/group. 
4. Run group in a loop. You can chose from which waypoint start the movement
5. Save and load waypoints and groups. (create two texts files that can be edited manually)
6. Stop movement

## DEMO

In this video you can watch how this server works. https://youtu.be/rPR7Dv711PI
