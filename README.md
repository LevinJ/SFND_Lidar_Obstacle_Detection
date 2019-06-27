# Lidar Obstable Detection Project

Course project for Udacity Sensor Fusion Engineer Nanodegree Program

## Overview


This project implements a basic pipeline to detect obstables using lidar data. The pipeline consists of below stage.

1. Filtering
2. plane segmentation
3. obstacle clustering

All the key algorithms used in the pipeline like RANSAC plane segmentation, three dimension KD tree, and Euclidean Clutering are implemented with C++, as opposed to the ready API provided in PCl library in the attempt to gain deeper understanding about these algorithms. 

## Final Result
The pipleine is applied to lidar data stream, and resonably good result are obtained as below.

Raw lidar point cloud


Obstable detection result

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Implementation Details

### Fitlering

Two filtering operation are done:
1) Use VoxelGrid to reduce the resolution of lidar point cloud
2) Use Region of interest to fitler out areas that are far away from the ego car, as well as the roof area


### Plane Segmentation

Used RANSAC to segment ground plane and obstacles

### Obstacle detection

## Reflections


