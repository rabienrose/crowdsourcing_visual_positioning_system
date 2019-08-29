# Crowdsourcing Visual Positioning System
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
## Introduction
  - In the future, there will be more and more cameras around us, especially after AR glasses become popular. Using these huge amount of visual information to build a virtual parallel world is a wonderful dream. The goal of this project is to provide vision-based positioning capabilities for this parallel world.
  - This project is also called chamo_vps for short. 
  - The specific goal in the near future is to establish a location system based on the map which is formed by feature points. And all data used to create map are from mobile phone level sensors (monocular camera, imu, gps). And provide a positioning service that give position with gps coordinate system. The accuracy is expected around 20 cm.
<a href="https://www.bilibili.com/video/av65482154" target="_blank"><img src="https://github.com/rabienrose/crowdsourcing_visual_positioning_system/blob/master/doc/map_loc.jpg" 
alt="Chamo" width="240" height="180" border="10" /></a>
<a href="https://www.bilibili.com/video/av65569066" target="_blank"><img src="https://github.com/rabienrose/crowdsourcing_visual_positioning_system/blob/master/doc/app.jpg" 
alt="Chamo" width="280" height="180" border="10" /></a>
## Main Content
  - Video data collection and LocalMap creation on the mobile phone.
  - Image data collection and feature point extraction. These means the system supports both sequence frame and single picture update.
  - After uploading the LocalMap or image feature points to the server, the server merges the data.
  - The merging process includes the culling of map to maintain trackable map size.
  - New data that is not matched by the map will be stored, waiting for matching with future data.
## Status
  - Map visualization in APP.
  - Merging new sensor data is OK, but not stable. After the clean function complete, it will be more stable.
  - Recording sensor data with custom exposure adjustment fucntion (Fix expousre time and adjust ISO).
  - Align layout with localization map.
  - Localization with only global match.
  - Average GPS to get more accurate global position.
  - Copy out the recorded bag with itune.
  - There is memory leak problem, some memory could not be released.
## Backlog
| Sprint | Target |
| ------ | ------ |
| September 1st | Use covisibility to choose update candidators |
| September 8th | Clean all distached or bad data in the end |
| September 15th | Self-calibration on APP |
| September 22th | Tracking with odomtry and global match |
## Future Function
  - Resume function for interrupt in processing a bag.
  - Reset the change caused by the lasted bag update.
  - Make global match realtime on IOS.
  - Show keypoints on frame image during SLAM and localization.
  - BA with IMU function
  - Automatical divide the bag.
  - Exposure duration max limitation.
  - Use SIM3 to transform the whole map before se3 pose optimization.
  - Further reduce memory leak.
  - Show percentage of mapping process.
## Technical Details
  - Compilation framework: catkin
  - Serialization method: custom format based on fstream read and write
  - Debugging visualization tool: rviz
  - Client end: ios
  - Math library: eigen
## Algorithm Flow
  - ![alt text](https://github.com/rabienrose/crowdsourcing_visual_positioning_system/blob/master/doc/algo_process.png "Workflow")
  - The green box represents the input data, which can be a sequence of pictures arranged in time, or it can be a single picture. And an approximate location obtained by global positioning is required for all data, which is used to put the data into the corresponding map block.
  - The purple box represents the processing flow on the clinet end. The final piece of data is split into multiple LocalMaps according to the covisibilities.
  - The blue box represents the processing flow on the server end.
  - The red box represents the data stored on the server after all processing.
## Build for ubuntu
  - Clone this project.
  - Download third-party code from link: https://drive.google.com/file/d/10ht85TEX7BfTgujHfyDWa96j6SfmQ0C3/view?usp=sharing
  - Extract and copy the download folder into src folder.
  - Run: bash build_all.sh to compile.
## Build for IOS
  - Clone this project.
  - Download third-party code from link: https://drive.google.com/file/d/1tGzyzI1jp5T0dIiNuiiF43nxzoMBHrCJ/view?usp=sharing
  - Extract and copy the download folder into ios folder.
  - Open the xcode file: ios/MapLoc/GlobalMap.xcodeproj, and build it use xcode. You may need to reset the path of lib and header in thirdpart folder.
## Usage
  - Build map: run_script/run_globalmap_api.sh
  - Test global localization: run_script/run_global_match_test.sh
  - Visualization of map: src/display_map
  - The scripts or packages named here are used for as a start point to look into the details of how to use.
