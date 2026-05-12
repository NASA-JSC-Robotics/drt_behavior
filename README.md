# drt_behavior

This repository contains libraries of [behaviortree.cpp](https://behaviortree.dev) behaviors, along with a basic application for tree execution in ROS 2 applications.

Behaviors are separated into individual packages depending on the interfaces they require to be used.
For example `moveit_behavior` depends on `moveit_msgs`, and a running `move_group` node.
This design allows the end user to pick which behavior packages they want to use, based on the dependencies they have installed.
If you do not require moveit, simply skip building the `moveit_behavior` package by adding a `COLCON_IGNORE` file.

`drt_behavior` is the core of the package and contains the behavior tree executor and logger.
Additionally, `drt_behavior` contains behaviors that only use common interfaces.

Each of the packages (besides `drt_behavior`) contains `plugins.cpp` file, that allows simplified registration of the behaviors through plugins infrastructure rather than through direct registration in `DRTBehaviorBtcppExecutor` object.
When writing new behavior it is highly recommended to register behavior through the plugins infrastructure to avoid potential circular dependencies between separated ROS packages.

## Configuration files

DRT behavior executors uses given [yaml parameter file](./drt_behavior/config/sample_bt_executor.yaml) to determine which plugins to load, where to find the behaviors definitions, and behavior trees.
The parameter files' `plugins` and `behavior_tree` fields are important to not when integrating new behaviors.

The `plugins` list declares which plugins to load, behaviors that are registered within the `plugins.cpp` will be loaded.

Yaml list of `behavior_trees` declares where to find xml files of the different behaviors that you write.

Make sure to double check that both of these are filled out properly if you are not able to find the behaviors you are looking for.


## Implemented Behaviors

Generally behavior names should be self-explanatory, but to understand how to use it effectively be sure to check the header, where the inputs and outputs are defined.

* Color Tools Behavior
  * Color Blob Find
  * Detect Color Blobs
  * Get Synced Image Pointcloud Depth
* DRT Behavior
  * Check Param
* Geometry Behavior
  * Align Transform
  * Apply Transform
  * Async Tf Lookup
  * Create Random Transform
  * Publish Transform
  * Transform To Pose
* Moveit Behavior
  * Execute Trajectory
  * Plan To Joint State
  * Plan To Pose
  * Publish Trajectory
