#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  save_flag_ = false;
  process_flag_ = false;
  minimum = 0;
  search_bound_ = 10;
  // search_bound_ = 1000000;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  prev_clock_stamp_ = 0;
  bag_idx_= 0;
}

ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;
  inspva_thread_.active_ = false;
  imu_thread_.active_ = false;
  ouster_thread_.active_ = false;
  velodyne_thread_.active_ = false;
  avia_thread_.active_ = false;
  aeva_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();
  velodyne_thread_.cv_.notify_all();
  if(velodyne_thread_.thread_.joinable()) velodyne_thread_.thread_.join();
  avia_thread_.cv_.notify_all();
  if(avia_thread_.thread_.joinable()) avia_thread_.thread_.join();
  aeva_thread_.cv_.notify_all();
  if(aeva_thread_.thread_.joinable()) aeva_thread_.thread_.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_    = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  inspva_pub_ = nh_.advertise<novatel_gps_msgs::Inspva>("/inspva", 1000);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
  magnet_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);
  ouster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ouster/points", 10000);
  velodyne_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne/points", 10000);
  avia_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/avia/points", 10000);
  aeva_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aeva/points", 10000);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  inspva_thread_.active_ = false;
  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();
  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  ouster_thread_.active_ = false;
  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();
  velodyne_thread_.active_ = false;
  velodyne_thread_.cv_.notify_all();
  if(velodyne_thread_.thread_.joinable()) velodyne_thread_.thread_.join();
  avia_thread_.active_ = false;
  avia_thread_.cv_.notify_all();
  if(avia_thread_.thread_.joinable()) avia_thread_.thread_.join();
  aeva_thread_.active_ = false;
  aeva_thread_.cv_.notify_all();
  if(aeva_thread_.thread_.joinable()) aeva_thread_.thread_.join();

  //check path is right or not
  ifstream f((data_folder_path_+"/stamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
//    data_stamp_[stamp] = data_name;
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;


  //Read inspva data
  fp = fopen((data_folder_path_+"/Inertial_data/inspva.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double height, north_velocity, east_velocity, up_velocity, roll, pitch, azimuth;
  char status_buffer[50];  // This buffer is to read "status: #" format
  int status;
  novatel_gps_msgs::Inspva inspva_data;
  inspva_data_.clear();
  while (fscanf(fp, "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%[^,\n]", 
                &stamp, &latitude, &longitude, &height, 
                &north_velocity, &east_velocity, &up_velocity, 
                &roll, &pitch, &azimuth, status_buffer) == 11) 
  {
      // Extracting the actual status value from "status: #"
      sscanf(status_buffer, "status: %d", &status);

      inspva_data.header.stamp.fromNSec(stamp);
      inspva_data.header.frame_id = "inspva";
      inspva_data.latitude = latitude;
      inspva_data.longitude = longitude;
      inspva_data.height = height;
      inspva_data.north_velocity = north_velocity;
      inspva_data.east_velocity = east_velocity;
      inspva_data.up_velocity = up_velocity;
      inspva_data.roll = roll;
      inspva_data.pitch = pitch;
      inspva_data.azimuth = azimuth;
      inspva_data.status = status;
      inspva_data_[stamp] = inspva_data;
  }

  fclose(fp);
  cout << "Inspva data are loaded" << endl;


  //Read IMU data
  fp = fopen((data_folder_path_+"/Inertial_data/xsens_imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
  sensor_msgs::Imu imu_data;
  sensor_msgs::MagneticField mag_data;
  imu_data_.clear();
  mag_data_.clear();

  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
    if(length != 8 && length != 17) break;
    if(length == 8){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;

      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;

    }else if(length == 17){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = g_x;
      imu_data.angular_velocity.y = g_y;
      imu_data.angular_velocity.z = g_z;
      imu_data.linear_acceleration.x = a_x;
      imu_data.linear_acceleration.y = a_y;
      imu_data.linear_acceleration.z = a_z;

      imu_data.orientation_covariance[0] = 3;
      imu_data.orientation_covariance[4] = 3;
      imu_data.orientation_covariance[8] = 3;
      imu_data.angular_velocity_covariance[0] = 3;
      imu_data.angular_velocity_covariance[4] = 3;
      imu_data.angular_velocity_covariance[8] = 3;
      imu_data.linear_acceleration_covariance[0] = 3;
      imu_data.linear_acceleration_covariance[4] = 3;
      imu_data.linear_acceleration_covariance[8] = 3;


      imu_data_[stamp] = imu_data;
      mag_data.header.stamp.fromNSec(stamp);
      mag_data.header.frame_id = "imu";
      mag_data.magnetic_field.x = m_x;
      mag_data.magnetic_field.y = m_y;
      mag_data.magnetic_field.z = m_z;
      mag_data_[stamp] = mag_data;
      imu_data_version_ = 2;

    }
  }
  cout << "IMU data are loaded" << endl;
  fclose(fp);

  ouster_file_list_.clear();
  velodyne_file_list_.clear();
  avia_file_list_.clear();
  aeva_file_list_.clear();

  GetDirList(data_folder_path_ + "/LiDAR/Ouster",ouster_file_list_);
  GetDirList(data_folder_path_ + "/LiDAR/Velodyne",velodyne_file_list_);
  GetDirList(data_folder_path_ + "/LiDAR/Avia",avia_file_list_);
  GetDirList(data_folder_path_ + "/LiDAR/Aeva",aeva_file_list_);


  data_stamp_thread_.active_ = true;
  inspva_thread_.active_ = true;
  imu_thread_.active_ = true;
  ouster_thread_.active_ = true;
  velodyne_thread_.active_ = true;
  avia_thread_.active_ = true;
  aeva_thread_.active_ = true;


  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  inspva_thread_.thread_ = std::thread(&ROSThread::InspvaThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
  ouster_thread_.thread_ = std::thread(&ROSThread::OusterThread,this);
  velodyne_thread_.thread_ = std::thread(&ROSThread::VelodyneThread,this);
  avia_thread_.thread_ = std::thread(&ROSThread::AviaThread,this);
  aeva_thread_.thread_ = std::thread(&ROSThread::AevaThread,this);
}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }

    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("inspva") == 0){
      inspva_thread_.push(stamp);
      inspva_thread_.cv_.notify_all();
    }else if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }else if(iter->second.compare("ouster") == 0){
        ouster_thread_.push(stamp);
        ouster_thread_.cv_.notify_all();
    }else if(iter->second.compare("velodyne") == 0){
        velodyne_thread_.push(stamp);
        velodyne_thread_.cv_.notify_all();
    }else if(iter->second.compare("livox_avia") == 0){
        avia_thread_.push(stamp);
        avia_thread_.cv_.notify_all();
    }else if(iter->second.compare("aeva") == 0){
        aeva_thread_.push(stamp);
        aeva_thread_.cv_.notify_all();
    }
    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::Clock clock;
        clock.clock.fromNSec(stamp);
        clock_pub_.publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }
    if(save_flag_ == true && process_flag_ == false){
      bag_.open(data_folder_path_ + "/" + to_string(bag_idx_) + ".bag", rosbag::bagmode::Write);
      process_flag_ = true;
    }
    else if(save_flag_ == false && process_flag_ == true){
      process_flag_ = false;
      bag_.close();
      bag_idx_++;
    }
  }
  cout << "Data publish complete" << endl;

}

void ROSThread::InspvaThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(inspva_thread_.mutex_);
    inspva_thread_.cv_.wait(ul);
    if(inspva_thread_.active_ == false) return;
    ul.unlock();
    while(!inspva_thread_.data_queue_.empty()){
      auto data = inspva_thread_.pop();
      //process
      if(inspva_data_.find(data) != inspva_data_.end()){
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/inspva", inspva_data_[data].header.stamp, inspva_data_[data]);
        }
        inspva_pub_.publish(inspva_data_[data]);
      }

    }
    if(inspva_thread_.active_ == false) return;
  }
}

void ROSThread::ImuThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
    imu_thread_.cv_.wait(ul);
    if(imu_thread_.active_ == false) return;
    ul.unlock();
    while(!imu_thread_.data_queue_.empty()){
      auto data = imu_thread_.pop();
      //process
      if(imu_data_.find(data) != imu_data_.end()){
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/imu/data_raw", imu_data_[data].header.stamp, imu_data_[data]);
        }
        imu_pub_.publish(imu_data_[data]);
      }

    }
    if(imu_thread_.active_ == false) return;
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}

//ouster
void ROSThread::OusterThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(ouster_thread_.mutex_);
    ouster_thread_.cv_.wait(ul);
    if(ouster_thread_.active_ == false) return;
    ul.unlock();

    while(!ouster_thread_.data_queue_.empty()){
      auto data = ouster_thread_.pop();
      //process
      //publish data
      if(to_string(data) + ".bin" == ouster_next_.first){
        //publish
        ouster_next_.second.header.stamp.fromNSec(data);
        ouster_next_.second.header.frame_id = "ouster";
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/ouster/points", ouster_next_.second.header.stamp, ouster_next_.second);
        }
        ouster_pub_.publish(ouster_next_.second);
      }else{
//        cout << "Re-load right ouster from path" << endl;
        //load current data
        pcl::PointCloud<pc_type_o> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/LiDAR/Ouster" +"/"+ to_string(data) + ".bin";
        if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") != ouster_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                pc_type_o point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
                file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "ouster";
            ouster_pub_.publish(publish_cloud);
        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<pc_type_o> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") - ouster_file_list_.begin();
      if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),ouster_file_list_[current_file_index+1]) != ouster_file_list_.end()){
          string next_file_name = data_folder_path_ + "/LiDAR/Ouster" +"/"+ ouster_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              pc_type_o point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
              file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
              file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);

          ouster_next_ = make_pair(ouster_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(ouster_thread_.active_ == false) return;
  }
} //end ouster

void ROSThread::VelodyneThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(velodyne_thread_.mutex_);
    velodyne_thread_.cv_.wait(ul);
    if(velodyne_thread_.active_ == false) return;
    ul.unlock();

    while(!velodyne_thread_.data_queue_.empty()){
      auto data = velodyne_thread_.pop();
      //process
      //publish data
      if(to_string(data) + ".bin" == velodyne_next_.first){
        //publish

        velodyne_next_.second.header.stamp.fromNSec(data); 
        velodyne_next_.second.header.frame_id = "velodyne";
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/velodyne/points", velodyne_next_.second.header.stamp, velodyne_next_.second);
        }
        velodyne_pub_.publish(velodyne_next_.second);

      }else{
//        cout << "Re-load right velodyne from path" << endl;
        //load current data
        pcl::PointCloud<pc_type> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/LiDAR/Velodyne" +"/"+ to_string(data) + ".bin";

        if(find(next(velodyne_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_file_list_.end(),to_string(data)+".bin") != velodyne_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                pc_type point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
                cloud.points.push_back (point);
            }

            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "velodyne";
            velodyne_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<pc_type> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(velodyne_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_file_list_.end(),to_string(data)+".bin") - velodyne_file_list_.begin();

      if(find(next(velodyne_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_file_list_.end(),velodyne_file_list_[current_file_index+1]) != velodyne_file_list_.end()){
          string next_file_name = data_folder_path_ + "/LiDAR/Velodyne" +"/"+ velodyne_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              pc_type point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
              file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
              cloud.points.push_back (point);
            }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          velodyne_next_ = make_pair(velodyne_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(velodyne_thread_.active_ == false) return;
  }
}

void ROSThread::AevaThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(aeva_thread_.mutex_);
    aeva_thread_.cv_.wait(ul);
    if(aeva_thread_.active_ == false) return;
    ul.unlock();
std::cout.precision(20);
    while(!aeva_thread_.data_queue_.empty()){
      auto data = aeva_thread_.pop();
      //process
      //publish data
      if(to_string(data) + ".bin" == aeva_next_.first){
        //publish
        aeva_next_.second.header.stamp.fromNSec(data) ;
        aeva_next_.second.header.frame_id = "aeva";
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/aeva/points", aeva_next_.second.header.stamp, aeva_next_.second);
        }
        aeva_pub_.publish(aeva_next_.second);

      }else{
        //load current data
        pcl::PointCloud<pc_type_a> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/LiDAR/Aeva" +"/"+ to_string(data) + ".bin";
        if(find(next(aeva_file_list_.begin(),max(0,previous_file_index-search_bound_)),aeva_file_list_.end(),to_string(data)+".bin") != aeva_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                pc_type_a point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
                file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
                if(data > 1691936557946849179)
                    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data); 
            publish_cloud.header.frame_id = "aeva";          
            aeva_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<pc_type_a> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(aeva_file_list_.begin(),max(0,previous_file_index-search_bound_)),aeva_file_list_.end(),to_string(data)+".bin") - aeva_file_list_.begin();
      if(find(next(aeva_file_list_.begin(),max(0,previous_file_index-search_bound_)),aeva_file_list_.end(),aeva_file_list_[current_file_index+1]) != aeva_file_list_.end()){
          string next_file_name = data_folder_path_ + "/LiDAR/Aeva" +"/"+ aeva_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              pc_type_a point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
              file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
              if(data > 1691936557946849179)
                  file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          aeva_next_ = make_pair(aeva_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(aeva_thread_.active_ == false) return;
  }
}

void ROSThread::AviaThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(avia_thread_.mutex_);
    avia_thread_.cv_.wait(ul);
    if(avia_thread_.active_ == false) return;
    ul.unlock();

    while(!avia_thread_.data_queue_.empty()){
      auto data = avia_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == avia_next_.first){
        //publish
        avia_next_.second.header.stamp.fromNSec(data);
        avia_next_.second.header.frame_id = "avia";
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/avia/points", avia_next_.second.header.stamp, avia_next_.second);
        }
        avia_pub_.publish(avia_next_.second);

      }else{
//        cout << "Re-load right avia from path" << endl;
        //load current data
        livox_ros_driver::CustomMsg avia_msg;
        string current_file_name = data_folder_path_ + "/LiDAR/Avia" +"/"+ to_string(data) + ".bin";
        if(find(next(avia_file_list_.begin(),max(0,previous_file_index-search_bound_)),avia_file_list_.end(),to_string(data)+".bin") != avia_file_list_.end()){
            int i = 0;
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                livox_ros_driver::CustomPoint point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint16_t));
                avia_msg.points.push_back(point);
                i++;
            }
            file.close();

            avia_msg.point_num = i;
            avia_msg.header.stamp.fromNSec(data);
            avia_msg.header.frame_id = "livox_avia";
            avia_pub_.publish(avia_msg);

        }
        previous_file_index = 0;
      }

      //load next data
      livox_ros_driver::CustomMsg avia_msg;
      current_file_index = find(next(avia_file_list_.begin(),max(0,previous_file_index-search_bound_)),avia_file_list_.end(),to_string(data)+".bin") - avia_file_list_.begin();
      if(find(next(avia_file_list_.begin(),max(0,previous_file_index-search_bound_)),avia_file_list_.end(),avia_file_list_[current_file_index+1]) != avia_file_list_.end()){
          string next_file_name = data_folder_path_ + "/LiDAR/Avia" +"/"+ avia_file_list_[current_file_index+1];
          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          int i = 0;
          while(!file.eof()){
              livox_ros_driver::CustomPoint point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
              avia_msg.points.push_back(point);
              i++;
          }
          file.close();
          avia_msg.point_num = i;
          avia_msg.header.stamp.fromNSec(data);
          avia_msg.header.frame_id = "livox_avia";
          avia_next_ = make_pair(avia_file_list_[current_file_index+1], avia_msg);
      }

      previous_file_index = current_file_index;
    }
    if(avia_thread_.active_ == false) return;
  }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{

  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }
}
