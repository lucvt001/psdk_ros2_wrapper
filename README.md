# psdk_ros2_wrapper

## Overview
This is a ROS2 wrapper for [DJI Payload-SDK](https://github.com/dji-sdk/Payload-SDK). It integrates very closely with the original code, and preserve all the logging utilities and error handling mechanisms even when using `ros2 launch`. It has been tested to work with ROS2 Humble and DJI M30T drone, and is a work in progress.

## Highlights
This wrapper is heavily **service and action** oriented, meaning that:
- All the functionalities are exposed as services and actions, which can be called from other nodes.
- The actions are non-blocking, meaning that they can be called in parallel, cancelled during execution, and allow you to execute other code while a certain action is being executed.
- It allows for handling of exception events such as control authority change, disruptions, and errors gracefully.
- In the long term, it is designed to be perfectly compatible with [BehaviorTree.CPP](https://www.behaviortree.dev/), which is a powerful tool for creating complex robot behaviors.

## Features
So far, only the following critical functionalities have been implemented:
- Flight control
- Liveview
- Gimbal control

## Tutorials

### - [Tutorial 0: Installation](docs/installation.md)
### - [Tutorial 1: Running the wrapper](docs/running_the_wrapper.md)

## Future work
- Implement custom widgets to interact with the wrapper from RC or MSDK.
- Implement more functionalities such as HMS (Health Management System), RTK positioning and additional payload.
- Integrate with BT for proper behavior control.
