GBeam Assembly Simulation
=========================

This package uses `assembly_sim` to instantiate a simulation of
magnetically-linked GBeams.

## Contents

### Gazebo SDF / URDF Model Files
* `gazebo_planning_scene.urdf` -- An URDF for just broadcasting all objects as part of a gazebo planning scene
* `gbeam_def.sdf.xacro` -- Definitions for GBeam links and atoms
* `gbeam_soup.sdf.xacro` -- An example of an instantiated soup with a fixed number of links and nodes
* `gbeam_sim.world` -- A simple empty world file

### Launchfiles

* `sim.launch` -- A complete example that will launch the soup in gazebo with a master interface

### Scripts

* `tf_scaler.py` -- A simple script to rebroadcast a scaled TF frame

## Examples

To launch the RVIZ UI with a moveable marker:
```
roslaunch sim.launch rviz:=true marker:=true
```

To launch with the Razer Hydra:
```
roslaunch sim.launch hydra:=true
```
