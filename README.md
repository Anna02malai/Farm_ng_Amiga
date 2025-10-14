# Ag-Cypher Lab
## Amiga Robot - Bonsai Robotics (Farm_Ng)

The following Repository contains the Description, Video Recordings and ROS2 package files of the SLAM Navigation of Amiga robot. This work done in the Ag-Cypher Lab by integrating the the MID-360 Livox Lidar to the Amiga. 

### Description:


### Package Files:

1) [config](slam_nav/config) - The folder contains the config and parameter files for Lidar, Slam_tool Box and Nav_2 Bringup.

2) [fast_lio_note](slam_nav/fast_lio_note/) - The directory contains the modified Fast_lio mapping C++ file code file which needs to be swapped with the file in your existing fast_lio package in the src folder and rebuild it to make the effective changes. It changes the reliability policy to best effort where there is less data loss.
   
3) [launch](slam_nav/launch/) - The following directory contains all the launch files of the package
4) [line_detection.ipynb](Rivulet_2.0/line_detection.ipynb) - The code file contains the filtering, clustering, detection and visualization of horizontal and diagonal trusses of the Center Pivot. It filters the area of interest and uses DBScan to cluster the points and runs RanSac on the inliers and get's the line equations and runs RANSAC again to detect the other diagonal on the outlier

### Report: 
   The [report](Independent_Study_Report.pdf) contains the complete details including steps, procedures and reference files for the project and its implementation along with the final results.

### Media: 
   - The following Video [Field_test.mp4](Media/Field_Test.mp4) is the demonstration of the manual operation of the Rivulet robot via joystick on the Centre Pivot Irrigation system.

      <!-- - The following video demonstrates the manual operation of the Rivulet robot via joystick on the Centre Pivot Irrigation system:
            
      <!-- {% raw %} 
      <video width="640" height="360" controls>
      <source src="Files/Field_Test.mp4" type="video/mp4">
      Your browser does not support the video tag.
      </video>
      {% endraw %} -->

   - The below images show the Rivulet Robot's environmental Field setup on the Center Pivot Irrigation System.
      - ![Rivulet_bot_1](Media/Rivulet_Bot_img2.jpg) 
      - ![Rivulet_bot_3](Media/Rivulet_Bot_img3.png)
      - ![Rivulet_bot_2](Media/Rivulet_Bot_img4.png) 

   - The Following Images are the results of mapping the environment and detecting the trusses of the centre pivot as explained before. 
      - The first image shows the Downsampled point cloud visualization of the center pivot irrigation system. 
         ![Downsampled_Center_Pivot](Media/Downsampled_Center_Pivot.png)
      - The second image displays the final result after detection of trusses of the center pivot in the side view perspective of the center pivot. 
         ![Center_pivot_Side_View](Media/Detected_trusses_CP_Side_view.png)
      - The third image displays the final result after detection of trusses of the center pivot in the front view perspective of the center pivot. 
         ![Center_pivot_Front_View](Media/Detected_trusses_CP_Front_view.png)
