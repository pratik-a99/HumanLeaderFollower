# HumanLeaderFollower Group : OMADA

This repository contains a ROS package that can track and follow a human subject using April Tags to identify the leader.

## Dependencies
* ROS
* Python
* Xterm
* CUDA (For faster detection)

## How to Use

Clone the github repository in your ROS workspace

```
cd catkin_workspace/src
git clone --recursive https://github.com/pratik-a99/HumanLeaderFollower
cd ..
```
Note: Edit the Makefile to enable GPU utlization.

Build the workspace
```
catkin build
```

Note: For `nvcc fatal   : Unsupported gpu architecture 'compute_30'` error, delete the unsupported architectures from the Makefile and CMakeLists.txt in the `darknet_ros` package

Source the workspace
```
source devel/setup.bash 
```

To launch the apriltag follower, use the following command:
```
roslaunch my_detector april_follower.launch
```

To launch the Human follower, use the following command:
```
roslaunch my_detector validator_follower.launch
```
## Output Screen Captures
1. Gazebo Simulation:

<img width="1435" alt="image" src="https://user-images.githubusercontent.com/24978535/168404650-67d447c5-72d8-4c0e-b480-f63f7c03e5d0.png">

2. Hardware Implementation:

<img width="1435" alt="image" src="https://user-images.githubusercontent.com/24978535/168404692-2599a3e1-9f14-4180-8704-8c6705753328.png">

## Demo Videos
[Gazebo Simulation of Human Leader Follower](https://youtu.be/0TFuDLcJlU0) \
[Gazebo Simulation of Human Leader Follower in a Crowded Environment](https://www.youtube.com/watch?v=5pQ5pH0jd5I&t=2s) \
[Real world demo of AprilTag Follower](https://www.youtube.com/watch?v=5_TJFhDXLic) \
[Real world demo of Human Leader Follower](https://www.youtube.com/watch?v=-fM43jx2K2w) 

## Contributors and Team mates
1. Pratik Acharya, UID: 117513615
2. Sumedh Koppula, UID: 117386066
3. Smith Orlandis, UID: 118493077
