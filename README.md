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

### Perception
After building the source files, the perception node can be started with:
```bash
$ rosrun atlas_object_sorting perception
```
This node recieves visual data from Atlas's Multisense head and determines the color and location
of objects in the environment. This location information can be used by the manipulation node to
plan footsteps and pick up objects.
