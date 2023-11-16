# Vision Programming Challenge
 
## Introduction
In this README I describe shortly my sollution to the given tasks. I solved the task using python and all scripts are located in "./edge_detection/src". <br>
To install the ros project clone the repository to the "&lt;ros_ws&gt;/src/ (in my case catkin_ws) and run 
```console
catkin_make
```
## Packages
Required packages:
```
numpy==1.20.3
cv_bridge
opencv-pyhon
```
## Tasks and results
### 1. Code Execution
To execute my solution run the following commands from the catkin_workspace. <br>
For the visualization make sure to include the the <em>robot</em> direcory containing the <em>mira</em>model.
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
#### Terminal 4.1: Edge Detector in Directory
Run the edge detection task for finding edges of all images in a given directory.
The standard directory for the stored data is "&lt;catkin_ws&gt;/src/edge_detection_ros_challenge/edge_detection_ros/neuraRoboticRosChallange/edge_detection/data/". If you data is stored elsewhere go into the <em>edge_detection edge_detector_server.py</em> file and change the variable <strong>dir</strong> to the desired path. <br>
The resulting edge images get stored into the sub directory "&lt;<em>path_to_data</em>&gt;/edge_imgs/".
```console
 rosrun edge_detection edge_detector_directory_client.py
```
#### Terminal 4.2: Edge Detecotor for Rosbag
For edge detection on the images recieved from the rosbag we have to start the edge detection rosbag client. The recieved images are detected on edges and the resulting image is then published to the topic <em>edge_image_publisher</em>. (For the visualiation go to 2)
```console
 rosrun edge_detection edge_detector_rosbag_client.py 
```
#### Terminal 4.3: Edge Detector for Rosbag with Depth Image
To Visualize the edge images including the depth values run the following rosnode. The rosbag images will be recieved and combined with the depth images. Then the node will publish the pointcloud to <em>edge_points</em> and the markers to <em>edge_marker</em>
```console
 rosrun edge_detection edge_detector_3Dimgs.py
```

### 2. Visualization
#### 2.1 Visualization of the Rosbag images
To visualize the edge images detected from the rosbag start rviz and subscibe to the topic.
```console
 rosparam set /use_sim_time true &&
 source devel/setup.bash &&
 roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false
```
You can add the topics with
 <strong>add</strong> -&gt; <strong>By topic</strong> -&gt; <strong>&lt;<em>topic</em>&gt;</strong>
 



