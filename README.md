# File player for HeLiPR Dataset

Maintainer: Minwoo Jung (moonshot@snu.ac.kr)

## 1. Obtain dependent package (defined msg)

```
$mkdir ~/catkin_ws
$cd ~/catkin_ws
$mkdir src
$cd src
$git clone https://github.com/rpmsnu/helipr_file_player.git
$cd ~/catkin_ws/src/helipr_file_player
```

## 2. Build workspace

```
$cd ~/catkin_ws
$catkin_make
```

## 3. Run file player

```
$source devel/setup.bash
$roslaunch file_player file_player.launch
```

## 4. Load data files and play

1. Click 'Load' button.
2. Choose data set folder.
3. The player button starts publishing data in the ROS message.
4. The Stop skip button skips data while the vehicle is stationary for convenience.
5. The loop button resumes when playback is finished.

## 5. Contributor
* Jinyong Jeong : The original author
* Giseop Kim, Seungsang Yun : Maintainer of file player (other versions)
