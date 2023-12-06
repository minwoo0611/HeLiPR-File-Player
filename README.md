# [HeLiPR Dataset](https://sites.google.com/view/heliprdataset) (File Player)

Explore the diverse landscapes and advanced LiDAR technology captured in our HeLiPR dataset!

🔍 **See the HeLiPR LiDAR Variety**  
Dive into the dataset's multifaceted LiDAR variety with this informative GIF.  
<p align="center">
  <img src="/cfg/helipr_dataset.gif" alt="HeLiPR LiDAR gif">
</p>

🌐 **Understanding the HeLiPR Dataset's Necessity**  
Discover the challenges posed by heterogeneous LiDARs in our detailed image.  
<p align="center">
  <img src="/cfg/problem.jpg" alt="HeLiPR Dataset Problem">
</p>

🛰️ **Insight into Sensor Utilization**  
Get a closer look at our system setup and the sensors involved.  
<p align="center">
  <img src="/cfg/setup.jpg" alt="HeLiPR Dataset System", width = "50%">
</p>

🌍 **Diverse Scenarios Explored**  
Explore our collection of various sequences and environments through this visual guide.  
<p align="center">
  <img src="/cfg/trajectory.png" alt="HeLiPR Dataset Trajectory">
</p>

🔗 **Linking with the MulRan Dataset**  
Learn about the long-term place recognition capabilities in conjunction with the MulRan dataset.  
<p align="center">
  <img src="/cfg/Mulran.jpg" alt="HeLiPR Dataset with MulRan", width = "70%">
</p>

**Maintainer**: Minwoo Jung (moonshot@snu.ac.kr)

## Recent Updates (August 2023)

---
**2023/12/06**: Open another repository for Pointcloud processing, which name is [HeLiPR-Pointcloud-Toolbox](https://github.com/minwoo0611/HeLiPR-Pointcloud-Toolbox).

**2023/12/05**: We post some images to help readers understand the HeLiPR dataset.

**2023/09/26**: Link for [paper](https://arxiv.org/abs/2309.14590) now available (arxiv version)

**2023/08/17**: Link for HeLiPR dataset now available.

**2023/08/15**: Repository for HeLiPR file player now available.

We're diligently working to upload the [dataset link](https://sites.google.com/view/heliprdataset). This paper is under-reviewed, but readers can be found our paper in [arxiv](https://arxiv.org/abs/2309.14590). If user wants to utilize the undistortion or accumulation of pointcloud from .bin file, please visite the [HeLiPR-Pointcloud-Toolbox](https://github.com/minwoo0611/HeLiPR-Pointcloud-Toolbox).

---

## 1. Pre-requisites

Before utilizing the file player, it's crucial to have both the `novatel-gps-msgs` and `livox` custom messages. Ensure you install these drivers:

### Novatel GPS Driver Installation:

Replace 'version' with your appropriate ROS version (e.g., melodic, noetic).

```bash
sudo apt-get install ros-'version'-novatel-gps-driver
```

### Livox ROS Driver Installation:

Visit the official Livox SDK repository on GitHub:

[**Livox ROS Driver on GitHub**](https://github.com/Livox-SDK/livox_ros_driver)

Ensure that both drivers are correctly installed for seamless operation of the file player.

## 2. Obtain Dependent Package (Defined msg)

To set up the necessary workspace and clone the file player repository:

```bash
$ mkdir ~/catkin_ws
$ cd ~/catkin_ws
$ mkdir src
$ cd src
$ git clone https://github.com/rpmsnu/helipr_file_player.git
$ cd ~/catkin_ws/src/helipr_file_player
```

## 3. Build the Workspace

Compile and build the workspace:

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## 4. Execute the File Player

To run the file player, make sure to source your workspace:

```bash
$ source devel/setup.bash
$ roslaunch file_player file_player.launch
```

## 5. Load Data Files and Play

Here's a step-by-step guide:

1. Click the 'Load' button.
2. Navigate and select the desired dataset folder.
3. Hit the player button to commence publishing data as ROS messages.
4. Use the 'Stop skip' button to skip intervals when the vehicle remains stationary. This feature enhances the user experience by focusing on significant data.
5. The loop button ensures that the data resumes playback from the beginning once completed.

## 6. Contributors:

- **Jinyong Jeong**: Original creator of the package.
- **Giseop Kim, Seungsang Yun**: Maintainers for different versions of the file player.

For any issues, queries, or contributions, please contact the maintainers.
