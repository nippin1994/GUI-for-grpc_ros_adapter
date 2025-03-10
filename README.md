# GUI for ROS 

## Introduction

This project presents an **Autonomous Unmanned Surface Vehicle (USV) Navigation System** that integrates the **Robot Operating System (ROS)** and **Unity** for real-time simulation. The primary goal of this project is to design and implement a flexible navigation system that dynamically follows target waypoints selected through a Graphical User Interface (GUI). The system uses a PID controller to ensure smooth navigation and accurate control of the USV's heading and throttle.

**Key features include:**

- Real-time selection of target waypoints via the GUI.
- Dynamic feedback of USV metrics such as location, heading, roll, pitch, and distance to the target via the GUI.
- Integration of a PID controller for path tracking and control of the USV.
- Communication between ROS and Unity through the gRPC adapter, ensuring real-time data flow and control updates.
- The project was developed as part of the **MSc in Automation, Controls, and Robotics** at **Sheffield Hallam University**. The design leverages ROS for control and communication, Unity for realistic simulation of boat dynamics, and a custom GUI for waypoint selection and monitoring of the USV's performance in real-time.

This repository includes the system architecture, control algorithms, and setup instructions, along with the necessary dependencies to simulate the USV's path-following capabilities.

## Getting started
Recommended ROS distribution is Noetic.
 **ROS Setup:**
* Create ROS catkin workspace in your linux environment.
* Clone this repository in your catkin workspace.
After cloning, run following command to pull latest proto generated source files:
`git submodule update --init`

* It is also recommended to create a `virtualenv` and then install the requirements:
`pip install -r requirements.txt`

* Clone [uuv_sensor_msgs](https://github.com/labust/uuv_sensor_msgs) in your workspace

* Build with `catkin build`

* If you encounter build errors, you can try building with python3:
`catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m`

* `source devel/setup.bash`

* Start grpc server:
`roslaunch grpc_ros_adapter launch_server.launch`

 **Unity Setup:**
* Clone  [marus-example](https://github.com/MARUSimulator/marus-example) repository and open this project in Unity. Proto messages are maintained in [marus-proto](https://github.com/MARUSimulator/marus-proto) repository.
* Pull `Assets>Scenes>Example scene` into the project space.
* Now, Start the game in the editor.
* Note: Run the ROS server first and then start the simulation in Unity. Unity's Console window should display that ROS connection is established.

## Dependencies
* Unity 2021.3.x LTS

## Usage and documentation

For usage information and examples visit [marus-example](https://github.com/MARUSimulator/marus-example) project repository and [Wiki-marus-example](https://github.com/MARUSimulator/marus-example/wiki).

For other information and documentation visit [documentation homepage](https://marusimulator.github.io).

## Credits & Acknowledgements

* [MARUS](https://github.com/MARUSimulator)
* [gRPC](https://github.com/grpc/grpc)
* [protobuf](https://github.com/protocolbuffers/protobuf)
* [Gemini Unity simulator](https://github.com/Gemini-team/Gemini)

## License
This project is released under the Apache 2.0 License. Please review the [License](https://github.com/MARUSimulator/grpc_ros_adapter/blob/dev/LICENSE) file for more details.
