# Ag-Cypher Lab (UD) <img src="Media/UD_logo.jpg" width="25" height="25">
## Amiga Robot - [Bonsai Robotics (Farm_ng)](https://farm-ng.com)

The following Repository contains the description, video recordings, ROS2 package files, installation steps and documentation of the SLAM Navigation of the Amiga robot. This work done in the Ag-Cypher Lab at University of Delaware by integrating the the MID-360 Livox Lidar to the existing Amiga Robot (Base Model). 

### Problem Statement:

   To enable Autonomous navigation for the Amiga Robot in indoor/GPS-denied environments like green houses and warehouses.

### Description:

   As the problem states we are trying to enable and solve the autonomous navigation of Amiga robot in small farms, greenhouses and warehouses kind of environment where having a multi-camera tracking setup/GPS is not feasible and economical. So inorder to solve this issue we tried to put a cost-effective lidar called [MID-360](https://www.livoxtech.com/mid-360) from the company livox on the base model of the Amiga Robot from Farm_ng company. 

   An additional reason for using the MID-360 Lidar is because of it's range and FoV. As the greenhouse environments mainly have structures like pillar and Vertical poles. We initially recorded pointcloud (3D Map) data inside our test environment [(Nursey)](slam_nav/pcd/Nursery.pcd) and we were able to get more reliable and dense pointcloud results but the space is not big enough for our robot, So we tested the SLAM Navigation in our [Hop_field](slam_nav/pcd/Hop_field.pcd) which has high vertical structure and setup like green house but it is an outdoor environment. 

   So as proof of concept we recorded the 2D occupance grid map of Hop Field and added a stvl layer additionally in Nav_2 stack for much more accurate obstacle detection, localization and successfully performed, tested and validated SLAM autonomous navigation. 

### Brief of my Work:

   - Fused the MID-360 LiDAR with the robot, applied Fast-LIO for 3D mapping and developed a ROS2 package to convert the maps to 2D occupancy grids and integrated with Nav2 for autonomous SLAM navigation in greenhouse trials, improving navigation accuracy and enabling reliable row-following under GPS-denied conditions.

### Package Files:

1) [config](slam_nav/config) - The folder contains the config and parameter files for Lidar, Slam_tool Box and Nav_2 Bringup.
   - [mid_360.yaml]() - The file contains the config and parameters for the livox_ros_driver and fast_lio 
   - [nav2_params_humble.yaml](slam_nav/config/nav2_params_humble.yaml) - This a parameter file for Nav_2 Bring up stack. 
   - [nav2_params_humble_voxel.yaml](slam_nav/config/nav2_params_humble_voxel.yaml) - This a parameter file with the additional stvl layer (spatio-voxel temporal layer) for Nav_2 Bring up stack. 

2) [fast_lio_note](slam_nav/fast_lio_note/) - The directory contains the modified Fast_lio mapping C++ file code file which needs to be swapped with the file in your existing fast_lio package in the src folder and rebuild it to make the effective changes. It changes the reliability policy to best effort where there is less data loss.
   
3) [launch](slam_nav/launch/) - The following directory contains all the launch files of the package
   -  [amiga.launch.py](slam_nav/launch/amiga.launch.py) - It is a launch file that launches the ROS Nodes and Controller nodes that sends velocity commands to Amiga Robot via M4 Feather CAN Express Microcontroller (CAN commands).
   - [pc2l.launch.py](slam_nav/launch/pc2l.launch.py) - This launch file launches the point_cloud to laserScan conversion 
   - [relay_topics.launch.py](slam_nav/launch/relay_topics.launch.py) - The launch file launches static transformation between frames and helper functions to convert the QOS data policy to Best_effort.
   - [slam_nav_helper.launch.py](slam_nav/launch/slam_nav_helper.launch.py) - This launch file is a combination of pc2l.launch.py and relay_topics.launch.py as a single launch file.

4) [livox_note](slam_nav/livox_note/) - The directory contains the modified livox_ros_driver2 C++ code files which needs to be swapped with the file in your existing fast_lio package in the src folder and rebuild it to make the effective changes. It changes the reliability policy to best effort where there is less data loss. 

5) [maps](slam_nav/maps/) - The folder contains the 2D occupancy grid maps recorded using the lidar which acted as our test environments.

6) [meshes](slam_nav/meshes/) - The folder contains the stl files required for the urdf and Robot Model.

7) [pcd](slam_nav/pcd/) - The folder contains the 3D pointcloud maps of our test environments which was mapped using the MID-360 LiDAR. 

8) [rviz](slam_nav/rviz/) - The folder contains the rviz config files for visualization of the data streaming and Maps.

9) [slam_nav](slam_nav/slam_nav/) - The directory contains all the code files of nodes and controllers and so on. 

10) [urdf](slam_nav/urdf/) - The folder contains the urdf files of the Robot. 

11) [Documentation](slam_nav/Documentation) - The file contains the commands to run in the respective terminals to launch the nodes required for autonomous navigation.

### Installation Guide (for ROS2):

**Step 1:** Clone and install the [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) package from the github repository and follow the respective instruction for successfull installation to publish msgs and access the sensor via ROS2.

**Step 2:** Clone and install the [FAST_LIO](https://github.com/hku-mars/FAST_LIO/tree/ROS2) package from the github repository and follow the respective instruction for successfull installation to use the fast_lio mapping algorithm and generate 3D maps.

**Step 3:** Replace the files from [fast_lio_note](slam_nav/fast_lio_note/) and [livox_note](slam_nav/livox_note/) to change the QoS policy to best_effort and rebuild it and source it to make the effective changes.

**Step 4 :** Clone and install the slam_nav package in your ROS2 workspace and then edit the MID_360.yaml config file from the livox_ros_driver2 with your respective IP address and required parameters of your lidar.

**Step 5:**  Then refer the commands from [Documentation.txt](slam_nav/Documentation) and run the following commands as per your scenario nad needs.

### Media: 
   - The following Video [Slam_1.mp4](Media/Slam_1.mp4) is the demonstration of the SLAM navigation of the Amiga Robot. 

      <!-- - The following video demonstrates the manual operation of the Rivulet robot via joystick on the Centre Pivot Irrigation system:
            
      <!-- {% raw %} -->
      <video width="640" height="360" controls>
         <source src="Media/Slam_1.mp4" type="video/mp4">
         Your browser does not support the video tag.
      </video>
      <!-- {% endraw %} --> 

