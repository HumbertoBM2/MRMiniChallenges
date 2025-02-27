
<div align="center">

# MRMiniChallenges

##### Solutions to the mini challenges designed by Manchester Robotics using ROS2 Humble. 

[![C++](https://img.shields.io/badge/C++-%2300599C.svg?logo=c%2B%2B&logoColor=white&?style=for-the-badge)](#)
[![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=fff&?style=for-the-badge)](#)
[![Python](https://img.shields.io/badge/ROS2-Humble-3776AB?logo=ros&logoColor=fff&?style=for-the-badge)](#)

</div>


## Prerequisites

- ROS2 Humble
- Ubuntu 22.04 Jammy
- Python 
- C++
- Text editor 

## Mini challenge 1 


In this challenge, we implemented a ROS2-based signal processing system where:

- A signal generator node publishes a sine wave.
- A processing node applies amplitude reduction, offset, and a phase shift.
- The results are visualized in real-time using PlotJuggler.

Initially, we attempted to use `rqt_plot`, but due to visualization issues, we switched to `PlotJuggler`, which provided a better real-time experience.

```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
```

This repository includes both Python and C++ implementations of the system. Before running any implementation, make sure to have your environment sourced.


```bash
source /opt/ros/humble/setup.zsh
```

#### Python
```bash
cd ~/MCH1_WS
colcon build --packages-select signal_processing
source install/setup.zsh
ros2 launch signal_processing signal_processing_launch.py
```

Once PlotJuggler is prompted, click the start Button in the left menu, and select the signal and proc_signal topics, then, drag and drop the data into the graph area. 

#### C++
```bash
cd ~/mch1cpp
colcon build --packages-select signal_processing_cpp --symlink-install
source install/setup.zsh
ros2 launch signal_processing_cpp signal_processing_launch.xml
```


Once PlotJuggler is prompted, click the start Button in the left menu, and select the signal and proc_signal topics, then, drag and drop the data into the graph area. 



To visualize the node connections, run:


```bash
ros2 run rqt_graph rqt_graph
```

#### Gallery for mini challenge 1

![graph1](Gallery/graph.jpg)
![plot1](Gallery/plot.jpg)


## **Mini Challenge 2**

This challenge involved designing and tuning a PID controller to regulate the behavior of a simulated DC motor in ROS 2. The objective was to ensure that the motor’s actual speed followed the desired set point as closely as possible, while allowing for real-time adjustments and different reference signal types.

### **System Overview**  

The system consisted of the following ROS 2 nodes:

- **Set Point Generator (`sp_gen`)** → Generates a reference trajectory that the motor should follow. It now supports **multiple signal types** (`sine`, `square`, `step`), which can be dynamically selected using `rqt_reconfigure`.  
- **DC Motor (`motor_sys`)** → Simulates a motor's response to control inputs. The motor parameters (**gain, time constant, sample time**) can now be **modified in real-time** through `rqt_reconfigure`.  
- **PID Controller (`ctrl`)** → Adjusts the input voltage to minimize the error between the **motor speed and set point**. It includes **anti-windup for integral control**, **a low-pass filter for smoother control action**, and **optimized derivative handling** to improve stability when tracking discontinuous signals.

Additionally, visualization and monitoring tools were used:

- **PlotJuggler** → Real-time data visualization of input voltages, motor speed, and set point.  
- **rqt_graph** → Visualizes node and topic interactions, providing insight into data flow.  
- **rqt_reconfigure** → Allows real-time tuning of **PID gains**, **motor parameters**, and **set point signal type** (`sine`, `square`, `step`).  

These enhancements make the system more flexible and robust, enabling dynamic parameter tuning and stable control for different types of inputs. With improved PID stability, automated visualization, and real-time configuration, this implementation provides a versatile platform for experimenting with PID control in ROS 2.



To test the developed solution for the challenge, you can run the following commands after cloning the repo:


```bash 
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

To launch only one group of nodes:
```bash
ros2 launch motor_control motor_launch.py
```

To launch multiple groups of nodes: 
```bash
ros2 launch motor_control challenge_launch.py
```


#### Gallery for mini challenge 2

![graph2](Gallery/graph21group.png)

![plot2](Gallery/plot21group.png)

![graph22](Gallery/graph23groups.png)

![plot22](Gallery/plot23groups.png)

![pid22](Gallery/reconfig2.png)

![pid22](Gallery/graph2sine.png)

![pid23](Gallery/graph2square.png)

![pid24](Gallery/graph2step.png)

![pid25](Gallery/graph2squaremulti.png)
