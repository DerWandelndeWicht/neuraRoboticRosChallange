# Vision Programming Challenge
 
## Introduction
In this README I describe shortly my sollution to the given tasks. I solved the task using python and all scripts are located in "./edge_detection/src".
## Packages

## Tasks and results
### 1. Code Execution
To execute my solution run the following commands in the ros_workspace (e.g. catkin_ws). 
#### Terminal 1
```console
 roscore
```
#### Terminal 2
Source the Setup file and start the edge detection server. 
```console
 source devel/setup.bash &&
 rosrun edge_detection edge_detector_server.py
```
#### Terminal 3
For tasks using the rosbag start the rosbag. 
```console
 rosbag play --clock -l <path to bagfile>
```
#### Terminal 4.1
Run the edge detection task for finding edges of all images in a given directory.
The standard directory for the stored data is "&lt;catkin_ws&gt;/src/edge_detection_ros_challenge/edge_detection_ros/neuraRoboticRosChallange/edge_detection/data/". To change this go into the <em>edge_detection edge_detector_server.py</em> file and change the variable <strong>dir</strong> to the desired path.
The resulting edge images get stored into the sub directory "<em>&lt;path_to_data&gt;</em>/edge_imgs/".
```console
 rosrun edge_detection edge_detector_directory_client.py
```
#### Terminal 4.2
For edge detection on the images recieved from the rosbag we have to start the edge detection rosbag client. The recieved images are detected on edges and the resulting image is then published to the topic <em>edge_image_publisher</em>. (For the visualiation go to 2.1.)
```console
 rosrun edge_detection edge_detector_rosbag_client.py 
```
#### Terminal 4.3

```console

```

### 2. Visualization
#### 2.1 Visualization of the Rosbag images
To visualize the edge images detected from the rosbag start rviz and subscibe to the topic.
```console
rviz
```
 <stron>add</strong> -&gt; <stron>By topic</strong> -&gt;<stron>/edge_image_publisher</strong> -&gt;<stron>image</strong>.
 
### 2.2 Visualization of the Pointcloud



