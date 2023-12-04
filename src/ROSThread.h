#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <rosgraph_msgs/Clock.h>

#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_gps_msgs/Inspva.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>


#include <dynamic_reconfigure/server.h>
#include <file_player/dynamic_file_playerConfig.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "file_player/color.h"
#include "rosbag/bag.h"
#include <ros/transport_hints.h>
#include "file_player/datathread.h"
#include <sys/types.h>

#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <livox_ros_driver/CustomMsg.h>

using namespace std;
using namespace cv;

struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity) 
    (uint16_t, ring, ring) (uint16_t, ambient, ambient)
)

struct AevaPointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    float reflectivity;
    float velocity;
    int32_t time_offset_ns;
    uint8_t line_index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (AevaPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, reflectivity, reflectivity) (float, velocity, velocity) 
    (int32_t, time_offset_ns, time_offset_ns) (uint8_t, line_index, line_index)
)

struct LivoxPointXYZI
{
    PCL_ADD_POINT4D;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
    uint32_t offset_time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPointXYZI,
    (float, x, x) (float, y, y) (float, z, z) (uint8_t, reflectivity, reflectivity)
    (uint8_t, tag, tag) (uint8_t, line, line) (uint32_t, offset_time, offset_time)
)

struct Point3D {

  float x;
  float y;
  float z;
  Point3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) {

  }
};

using pc_type = PointXYZIRT;
using pc_type_o = OusterPointXYZIRT;
using pc_type_l = LivoxPointXYZI;
using pc_type_a = AevaPointXYZIRT;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();
    void ros_initialize(ros::NodeHandle &n);
    void run();
    QMutex *mutex_;
    std::mutex bag_mutex_;
    ros::NodeHandle nh_;
    ros::NodeHandle left_camera_nh_;
    ros::NodeHandle right_camera_nh_;

    ros::NodeHandle thermal_left_camera_nh_;
    ros::NodeHandle thermal_right_camera_nh_;

    ros::NodeHandle thermal_14bit_left_camera_nh_;
    ros::NodeHandle thermal_14bit_right_camera_nh_;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> left_cinfo_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> right_cinfo_;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> thermal_left_cinfo_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> thermal_right_cinfo_;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> thermal_14bit_left_cinfo_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> thermal_14bit_right_cinfo_;

    int64_t initial_data_stamp_;
    int64_t last_data_stamp_;

    bool auto_start_flag_;
    int stamp_show_count_;

    bool play_flag_;
    bool pause_flag_;
    bool save_flag_;
    bool loop_flag_;
    bool stop_skip_flag_;
    double play_rate_;
    rosbag::Bag bag_;
    bool process_flag_;
    int bag_idx_;
    string data_folder_path_;

    int imu_data_version_;

    void Ready();
    void ResetProcessStamp(int position);

signals:
    void StampShow(quint64 stamp);
    void StartSignal();

private:

    int search_bound_;

    float minimum;
    bool omni_active_;

    ros::Subscriber start_sub_;
    ros::Subscriber stop_sub_;

    ros::Publisher inspva_pub_;

    ros::Publisher imu_origin_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher magnet_pub_;
    ros::Publisher ouster_pub_;
    ros::Publisher velodyne_pub_;
    ros::Publisher avia_pub_;
    ros::Publisher aeva_pub_;


    ros::Publisher clock_pub_;

    int64_t prev_clock_stamp_;

    multimap<int64_t, string>                    data_stamp_;
    map<int64_t, novatel_gps_msgs::Inspva>    inspva_data_;
    map<int64_t, sensor_msgs::Imu>         imu_data_;
    map<int64_t, sensor_msgs::MagneticField>         mag_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> inspva_thread_;
    DataThread<int64_t> imu_thread_;

    DataThread<int64_t> ouster_thread_;
    DataThread<int64_t> velodyne_thread_;
    DataThread<int64_t> avia_thread_;
    DataThread<int64_t> aeva_thread_;
    DataThread<int64_t> rosbag_thread_;

    map<int64_t, int64_t> stop_period_; //start and stop stamp

    void DataStampThread();
    void InspvaThread();
    void ImuThread();
    void OusterThread();
    void VelodyneThread();
    void AviaThread();
    void AevaThread();
    void RosbagSaveThread();


    void FilePlayerStart(const std_msgs::BoolConstPtr& msg);
    void FilePlayerStop(const std_msgs::BoolConstPtr& msg);

    vector<string> ouster_file_list_;
    vector<string> velodyne_file_list_;
    vector<string> avia_file_list_;
    vector<string> aeva_file_list_;

    ros::Timer timer_;
    void TimerCallback(const ros::TimerEvent&);
    int64_t processed_stamp_;
    int64_t pre_timer_stamp_;
    bool reset_process_stamp_flag_;

    pair<string,sensor_msgs::PointCloud2> ouster_next_;
    pair<string,sensor_msgs::PointCloud2> velodyne_next_;
    pair<string,livox_ros_driver::CustomMsg> avia_next_;
    pair<string,sensor_msgs::PointCloud2> aeva_next_;



    int GetDirList(string dir, vector<string> &files);


public slots:

};

#endif // VIEWER_LCM_H
