# lidar_detection
 The main goal of the project is to filter, segment, and cluster  
 real point cloud data to detect obstacles in a driving environment.
 
 
 This project implements pipeline for converting the raw LIDAR sensor measurements into trackable objects. 
 It implements filtering, segmentation, clustering, boundbox routines. Filtering was performed using 
 the PCL functions. 
 Functions were implemented for segmentation and clustering. The pipeline details are as the following.
```


## PipeLine 
Following are the steps for the pipeline
- Read a PCD file.
- Filter the raw LIDAR data 
- Segment the filtered data to identify road and objects
- Clustering: from objects identify the clusters.
- For each cluster append a bounding box. 
- Rendering: Render the objects and road to the viewer. 



## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/enginBozkurt/LidarObstacleDetection.git
$> cd LidarObstacleDetection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
