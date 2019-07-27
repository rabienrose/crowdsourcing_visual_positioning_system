# Crowdsourcing Visual Positioning System
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
## Introduction
  - In the future, there will be more and more cameras around us, especially after AR glasses become popular. Using there huge amount of visual information to build a virtual parallel world is a wonderful dream. The goal of this project is to provide vision-based positioning capabilities for this parallel world.
  - This project is also called chamo_vps for short. 
  - The specific target in the near future is to establish a location map based on the point descriptor based on the sensor of the mobile phone (monocular camera, imu, gps), and provide a positioning service in the gps coordinate system based on the map.
## Main Content
  - Video data collection and LocalMap creation on the mobile phone.
  - Image data collection and feature point extraction. These means the system supports both sequence frame and single picture map creation and update.
  - After uploading the LocalMap or image feature points to the server, the server merges the data.
  - The merging process includes the culling of map to maintain trackable map size.
  - New data that is not matched by the map will be stored, waiting for matching with future data.
## RoadMap
| Date | Traget |
| ------ | ------ |
| August 11th | Serialization and visualization of GlobalMap |
| August 25th| Generate LocalMap |
| September 1st | Matching with GlobalMap |
| September 8th | Optimize affected frames |
| September 29th | Implenment on IOS |
| October 6th | Self-calibration function |
| October 27th | Apple store online |
## Technical Details
  - Compilation framework: catkin
  - Serialization method: custom format based on fstream read and write
  - Debugging visualization tool: rviz
  - Client end: ios
  - Math library: eigen
## Algorithm Flow
  - ![alt text](https://github.com/rabienrose/crowdsourcing_visual_positioning_system/blob/master/doc/algo_process.png "Workflow")
  - The green box represents the input data, which can be a sequence of pictures arranged in time, or it can be a single picture. And a approximate location obtained by coarse positioning is required for all data, which is used to put the data into the corresponding map block.
  - The purple box represents the processing flow on the clinet end. The final piece of data is split into multiple LocalMaps according to the covisibilities.
  - The blue box represents the processing flow on the server end.
  - The red box represents the data stored on the server after all processing.
