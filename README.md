# ROS Autonomous Driving and Path Planning SLAM with TurtleBot3

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About-this-Repository">About This Repository</a></li>
    <li><a href="#Using-this-Repository">Using this Repository</a></li>
    <li><a href="#Course-Workflow">Course Workflow</a></li>
    <li><a href="#Features">Features</a></li>
    <li><a href="#Pre-Course-Requirments">Pre-Course Requirments</a></li>
    <li><a href="#Link-to-the-Course">Link to the Course</a></li>
    <li><a href="#Notes">Notes</a></li>
    <li><a href="#Instructors">Instructors</a></li>
    <li><a href="#License">License</a></li>
  </ol>
</details>

## About this Repository
This is repository for the course **ROS Autonomous Driving and Path Planning SLAM with TurtleBot3** availble at Udemy . Complete source code is open sourced . Notes are also attached in root of the repository.

 ![alt text](https://github.com/noshluk2/ROS-Autonomous-Driving-and-Path-Planning-SLAM-with-TurtleBot/blob/master/image_resources/main_cover.png)
- **[[Get course Here]](https://www.udemy.com/course/robotics-with-ros-autonomous-driving-and-path-planning-slam/?couponCode=APRIL_END)**
----
## Using this Repository
* Move into your workspace/src folder
 ```
 cd path/to/ros1_ws/src/
##e.g cd ~/catkin_ws/src/
  ```
* Clone the repository in your workspace
```
git clone https://github.com/noshluk2/ROS-Autonomous-Driving-and-Path-Planning-SLAM-with-TurtleBot
```


* Perform make and build through catkin
 ```
 cd /path/to/workspace_root/
 ##e.g ~/catkin_ws/
 catkin_make
 ```
 
* Source your Workspace in any terminal you open to Run files from this workspace ( which is a basic thing of ROS )
```
source /path/to/catkin-ws/devel/setup.bash
```
- (Optional for Power USERs only) Add source to this workspace into bash file
 ```
echo "source /path/to/catkin-ws/devel/setup.bash" >> ~/.bashrc
 ```
  **NOTE:** This upper command is going to add the source file path into your ~/.bashrc file ( Only perform it once and you know what you are doing).This will save your time when running things from the Workspace

----
## Course Workflow
- Main robot we will be using is Turtle Bot 3 by Robotis . Package from official GitHub repository is going to obtained and then we will start to analyze how robot is launched into simulations like Rviz and Gazebo . 
- After Going through multiple launch files we will create a custom launch file to bring the robot in to simulations . SLAM using Gmapping node will be executed for our custom created world containing MAZE . 
- Then we will perform last project of Intruder Detection and Surveillance in which we are going to utilize Navigation stack as a main process.


---
## Features
* **Custom Mapping Maze using Gazebo** 
  -  ![alt text](https://github.com/noshluk2/ROS-Autonomous-Driving-and-Path-Planning-SLAM-with-TurtleBot/blob/master/image_resources/maze_creation.png)
* **Maze Solving using Navigation Stack** 
  -  ![alt text](https://github.com/noshluk2/ROS-Autonomous-Driving-and-Path-Planning-SLAM-with-TurtleBot/blob/master/image_resources/maze_solving.gif)
* **Intruder Surveillance and Navigation**
  - ![alt text](https://github.com/noshluk2/ROS-Autonomous-Driving-and-Path-Planning-SLAM-with-TurtleBot/blob/master/image_resources/Room_navigation.gif)
* 


----
## Pre-Course Requirments 

**Software Based**
* Ubuntu 20.04 (LTS)
* ROS1 - Noetic
---
## Link to the Course
Below is a discounted coupon for people who want to take the course in which more explaination to this code has been added

**[[Get course Here]](https://www.udemy.com/course/robotics-with-ros-autonomous-driving-and-path-planning-slam/?couponCode=APRIL_END)**

----
## Notes
 We have uploaded all the notes made during the lectures of the course so you can get more out of this repository with the instructors Notes. A seperate folder named as **Notes** contain a single PDF carrying all the notes in the root of this repository
----

## Instructors

Muhammad Luqman (ROS Simulation and Control Systems) - [Profile Link](https://www.linkedin.com/in/muhammad-luqman-9b227a11b/)  

----
## License

Distributed under the GNU-GPL License. See `LICENSE` for more information.
