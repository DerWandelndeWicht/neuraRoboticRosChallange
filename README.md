# Vision Programming Challenge
 
## 1. Introduction
In this README I describe shortly my sollution to the given tasks. I solved the task using python and all scripts are located in "./edge_detection/src". <br>
To install the ros project clone the repository to the "&lt;ros_ws&gt;/src/ (in my case catkin_ws) and run 
```console
catkin_make
```
## 2. Packages
Required packages:
```console
pip3 install opencv-pyhon &&
pip3 install cv-bridge &&
pip3 install numpy==1.20.3
```
## Task and Results
For solving the given task i created a ros package with a service that detect the edges in a given image with a canny edge detector of the opencv library. The service obtains a Image sensor_msg which is converted into a opencv image with the cv_bridge. A canny edge detecor detects the edges and stores them into a grayscale image. The edges on the resulting image are tainted green, converted back into a Image sensor_msg and send back. <br>
The service is in: edge_detection edge_detector_server.py
<br> 
The package contains multiple clients to make use of the sevice: <br>
<ul>
 <li>edge_detector_directory_client.py: Converts all images in an directory and save the into a sub directory.</li>
 <li>edge_detector_rosbag_client.py: Converts rgb images obained by a rosbag and publish the edge images under a new topic.</li>
 <li>edge_detector_3Dimgs.py: Converts rgb images and combines the with the depth images from the rosbag. Publishes the results as a Pointcloud and Markers under a new totpic.</li>
</ul> 
<br>
The solution I used for the problem has good solutions in mose cases but also shows problems with very noisy images like in "./edge_detection/data/Image_2.png". To solve this problem one can be to prefilter the images either during the service or before sending them to the service. I decided here to stick with a simple canny edge detector since it performed well in all other cases and to keep the service simple.
### 2. Code Execution
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

### 3. Visualization
#### 2.1 Visualization of the Rosbag images
To visualize the edge images detected from the rosbag start rviz and subscibe to the topic.
```console
 rosparam set /use_sim_time true &&
 source devel/setup.bash &&
 roslaunch mira_picker display.launch gripper_name:=robotiq2f_140 publish_joint_state:=false publish_robot_state:=false
```
You can add the topics with
 <strong>add</strong> -&gt; <strong>By topic</strong> -&gt; <strong>&lt;<em>topic</em>&gt;</strong>
 



