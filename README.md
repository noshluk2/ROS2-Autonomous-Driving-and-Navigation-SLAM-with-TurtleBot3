# ROS2 Autonomous Driving and Path Planning SLAM with TurtleBot3 using NAV2

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About-this-Repository">About This Repository</a></li>
    <li><a href="#Using-this-Repository">Using this Repository</a></li>
    <li><a href="#Course-Workflow">Course Workflow</a></li>
    <li><a href="#Features">Features</a></li>
    <li><a href="#Pre-Course-Requirments">Pre-Course Requirments</a></li>
    <li><a href="#Link-to-the-Course">Link to the Course</a></li>
    <li><a href="#Instructors">Instructors</a></li>
    <li><a href="#License">License</a></li>
  </ol>
</details>

## About this Repository
This is repository for the course **ROS2 Autonomous Driving and SLAM using NAV2 with TurtleBot3** availble at Udemy . Complete source code is open sourced.

 ![alt text](https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3/blob/master/image_resources/main_cover.png)
- **[[Get course Here]](https://www.udemy.com/course/robotics-with-ros-autonomous-driving-and-path-planning-slam/?couponCode=MAY_LEARN)**
----
## Using this Repository
* Move into your workspace/src folder
 ```
 cd path/to/ros2_ws/src/
##e.g cd ~/ros2_ws/src/
  ```
* Clone the repository in your workspace
```
git clone https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3
```


* Perform make and build through colcon
 ```
 cd /path/to/workspace_root/
 ##e.g ~/ros2_ws/
 colcon build
 ```

* Source your Workspace in any terminal you open to Run files from this workspace ( which is a basic thing of ROS )
```
source /path/to/ros2_ws/install/setup.bash
```
- (Optional for Power USERs only) Add source to this workspace into bash file
 ```
echo "source /path/to/ros2_ws/install/setup.bash" >> ~/.bashrc
 ```
----
## Course Workflow
- Main robot we will be using is Turtle Bot 3 by Robotis . Package from official GitHub repository is going to obtained and then we will start to analyze how robot is launched into simulations like Rviz and Gazebo .
- After Going through multiple launch files we will create a custom launch file to bring the robot in to simulations . SLAM using Gmapping node will be executed for our custom created world containing MAZE .
- Then we will perform last project of Autonomous Hotel Waiter  in which we are going to utilize NAV2 stack with simple GUI to send robot to different tables.


---
## Features
* **Turtlebot3 World Navigation using NAV2**
  -  ![alt text](https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3/blob/master/image_resources/Nav2_Multi_goals.gif)
* **Maze Solving using Commander API and NAV2**
  -  ![alt text](https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3/blob/master/image_resources/maze_solving.gif)
* **Autonomous Waiter with GUI**
  - ![alt text](https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3/blob/master/image_resources/Nav2_Waiter_bot.gif)



----
## Pre-Course Requirments

**Software Based**
* Ubuntu 22.04 (LTS)
* ROS2 - Humble
---
## Link to the Course
Below is a discounted coupon for people who want to take the course in which more explaination to this code has been added

**[[Get course Here]](https://www.udemy.com/course/robotics-with-ros-autonomous-driving-and-path-planning-slam/?couponCode=MAY_LEARN)**

----

## Instructors

Muhammad Luqman (ROS2 Simulation and Control Systems) - [Profile Link](https://www.linkedin.com/in/muhammad-luqman-9b227a11b/)

----
## License

Distributed under the GNU-GPL License. See `LICENSE` for more information.
