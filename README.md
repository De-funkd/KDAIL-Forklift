# Smart-Forklift

**KDAIL-Forklift** is an intelligent autonomous forklift robot designed as part of a warehouse automation project, built in collaboration with <a href="https://krishnaallied.com/">KDAIL</a>.

# Brief

# Problem statement

Modern warehouses, especially those used by defense orgs face significant challenges in automating indoor material handling tasks due to :
- the lack of GPS 
- the fragmented nature of robotic software stacks  
- security loopholes with proprietery tech 

Existing solutions are often :
- hardware-dependent 
- tailored for specific environments—making them difficult to scale or replicate
- over-complicated and non-intuitive 
- heavily setup dependent 

Moreover, enabling robots to navigate dynamic, cluttered indoor spaces while accurately locating and manipulating goods remains an open problem, especially for small- to medium-scale industries seeking low-cost, swarm-based intelligent automation.

---
# Proposed solution 

We addres these challenges with an end-to-end, modular, and reproducible robotics system tailored for indoor defence warehouse logistics. At its core, the robot fuses <b>UWB-based odometry, 2D LiDAR,</b> and <b>ArUco marker</b> detection to achieve robust SLAM and localization, even in GPS-denied and visually sparse environments. It is capable of autonomously navigating the environment, identifying loads, and executing pick-and-place operations using a forklift actuation mechanism.
The robot is instructed via an intuitive web-based interface backed by a ROS-powered backend, allowing real-time monitoring, control, and task assignment. All hardware—from the custom chassis to sensing and actuation—is designed to be inexpensive and in-house, enabling widespread replication and adoption in budget-constrained setups.

# Features

- 2D LiDAR-based object detection, mapping
- Autonomous navigation and obstacle avoidance
- UWB integration for accurate odometry(sub 5-cms)
- Aruco markers integration for accurate identification and pick-up
- Forklift actuation for pick-and-place
- Simulation + Real-world compatibility
- Modular ROS-based architecture

---

#  Demonstration and Media

- [UWB RTLS demo](https://youtube.com/shorts/iG-MlcV99eU?feature=share)

---


## Tech Stack

- **ROS2 Humble** on Ubuntu 22.04
- **Raspberry pi 4b** for onboard compute 
- **LiDAR** (RPLIDAR A2M8) for mapping and detection
- **UWB-anchors and seekers**(DWM30001CDK) for odometry and ranging
- **OpenCV** for computer vision utilities
- **Gazebo** + **RViz** for simulation and visualization
- **ROS-node backed web interface**for cross-platform communication and easy control. 

---

##  Repository Structure

```bash
KDAIL-Forklift/
├── aruco/                  # Scripts to take camera feed, check aruco markers
├── bot_controls/           # Scripts to do teleop and control crane mechanism of the bot
├── ros2_ws/                # Ros workspace of the repository, contains all the nodes and topics created
├── web_control/            # Inuitive UI for over the air communication/control of the bot
├── README.md               # You're here!
  
