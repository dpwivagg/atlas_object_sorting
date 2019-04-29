# Atlas Object Sorting
A ROS program to make the Atlas robot sort objects by color.


## Dependencies
This project uses the following tools/libraries to perform object sorting with Atlas:
 - ROS Kinetic
 - PCL
 - OpenCV for C++
 - TOUGH: [Transportable Opensource UI for Generic Humanoids](https://github.com/WPI-Humanoid-Robotics-Lab/tough)
 
 
## Usage
The following nodes are used to perform perception and manipulation tasks for sorting objects.
As always, use `catkin_make install` in the root of your ROS Kinetic Catkin Workspace to build
the project.

### Setup
The TOUGH bringup must be executed to generate the map and load the footstep planner before Atlas
can walk to a goal pose.

### Perception
After building the source files, the perception node can be started with:
```bash
$ rosrun atlas_object_sorting perception _color:=<red/green/blue>
```
This node recieves visual data from Atlas's Multisense head and determines the location
of the specified color of object in the environment. It publishes a Pose2D message on
the `/colorCentroid` topic. The pose is located directly in front of where the object
is positioned, so that Atlas is within reach and can manipulate the object.

### Navigation
The navigation node is started with:
```bash
$ rosrun atlas_object_sorting navigation
```
This node simply waits for Pose2D messages on the topic `/colorCentroid`. When a message
is recieved, the node sends the goal pose to the TOUGH walking library, which handles
footstep planning and walking.
