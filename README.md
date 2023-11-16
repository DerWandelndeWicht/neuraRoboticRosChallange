# Vision Programming Challenge
 
## Introduction
In this README I describe shortly my sollution to the given tasks. I solved the task using python and all scripts are located in "./edge_detection/src".
## Packages

## Tasks and results
### 1. Code Execution
To execute my solution run the following commands in the ros_workspace (e.g. catkin_ws). 
#### Terminal1
```console
 roscore
```
#### Terminal 2
Source the Setup file and start the edge detection server. 
```console
 source devel/setup.bash
 rosrun edge_detection edge_detector_server.py
```
#### Terminal 3.1
Run the edge detection task for finding edges of all images in a given directory.
The standard directory for the stored data is "./src/edge_detection_ros_challenge/edge_detection_ros/neuraRoboticRosChallange/edge_detection/data". To change this go into the <em>edge_detection edge_detector_server.py</em> file and change the variable <em>dir</em> to the desired path.
The resulting edge images get stored into the sub directory "<em><path to data></em>/edge_imgs/".
```console
 rosrun edge_detection edge_detector_directory_client.py
```

```console
 roscore
```


### 2. Vision_ROS: 
Provide ROS .srv and .msg files required to create a ROS service for edge detection. Give example usage of this service with a client to detect edges for image files in a directory.

Additionally, detect edges for the images subscribed from an image topic that is available when the given ROS bag (.bag) file is played. The input image and detected edges should be visualised on RViz.

Convert the detected edge pixels from pixel coordinates (u, v) to 3D data (x, y, z) using the depth images and camera parameters also available in the .bag file. Publish the 3D data to a ROS topic (suggestion: of type sensor_msgs/PointCloud), with a topic name edge_points.

### 3. Robot_ROS: 
Extend the code further by visualizing the 3D edge points for each frame as RViz markers together with the visualization of a robot. You can use the robot URDF model provided to visualize the robot and multiple frames of reference. Please provide a video of the markers and the robot on Rviz for a duration of one loop of the given .bag file as part of the submission.

### 4. Advanced: 
Do all the above tasks.

