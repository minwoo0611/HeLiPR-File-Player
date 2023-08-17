# File Player for HeLiPR Dataset

**Maintainer**: Minwoo Jung (moonshot@snu.ac.kr)

## Recent Updates (August 2023)

---
**2023/08/17**: Link for HeLiPR dataset now available!
**2023/08/15**: Repository for HeLiPR file player now available!

We're diligently working to upload the [dataset link](https://sites.google.com/view/heliprdataset). As we're also preparing for the release of our papers, there might be a slight delay for showing our paper. We appreciate your patience.

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
