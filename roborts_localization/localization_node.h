/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H

#include <memory>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/GetMap.h>

#include "log.h"
#include "localization_config.h"
#include "amcl/amcl.h"
#include "localization_math.h"
#include "types.h"
#include <std_msgs/Float64MultiArray.h>
#include <vector>

#define THREAD_NUM 4 // ROS SPIN THREAD NUM

namespace roborts_localization
{
  // struct Labelmsg
  // {
  //   int toward;     //标签的朝向
  //   double x;       //标签的坐标x
  //   double y;       //标签的坐标y
  //   float distance; //相机与标签的距离
  //   float arfa;     //相机的位置向标签看，标签中心与相机的连线与光轴的夹角，arfa 的正负为，左负右正
  //   float sita;     //由相机解算出来的车的角度
  // };

  class LocalizationNode
  {
  public:
    /**
   * @brief Localization Node construct function
   * @param name Node name
   */
    LocalizationNode(std::string name);

    /**
   * @brief Localization initialization
   * @return Returns true if initialize success
   */
    bool Init();

    //void vision_location_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // float pose_vision();

    /**
   * @brief Laser scan messages callback function (as main loop in localization node)
   * @param laser_scan_msg Laser scan data
   */
    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

    /**
   * @brief Manually initialize localization init_pose
   * @param init_pose_msg Init pose (2D Pose Estimate button in RViz)
   */
    void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg);
    /*******************************************************************************/
    //void posviCallback(const roborts_localization::vi::ConstPtr &P);
    // void vision_location_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void sentryCallback(const std_msgs::Float64MultiArray::ConstPtr &sentry_msg);
    void lostCallback(const std_msgs::Float64MultiArray::ConstPtr &lost_msg);
    void htcCallback(const std_msgs::Float64MultiArray::ConstPtr &htc_msg);
    void Read_costmap_Attr(const char *pszNode);
    void laseractCallback(const std_msgs::Float64MultiArray::ConstPtr &laseract_msg);

    // void anglecarheadCallback(const std_msgs::Float64MultiArray::ConstPtr &angle_msg);
    // void thetaCallback(const std_msgs::Float64MultiArray::ConstPtr &theta_msg);
    // void Getlabelpose(HypPose &hyp_pose,
    //                   Labelmsg &labelmsg,
    //                   const float *labelarr);
    void FinalPose(HypPose &hyp_pose_new,
                     HypPose &hyp_pose_old,
                     double *updatearr,
                     const double &updateway);
    /*******************************************************************************/ 
    /**
   * @brief Publish visualize messages
   */
    void PublishVisualize();

    /**
   * @brief Publish transform information between odom_frame and global_frame
   * @return True if no errors
   */
    bool PublishTf();

  private:
    bool GetPoseFromTf(const std::string &target_frame,
                       const std::string &source_frame,
                       const ros::Time &timestamp,
                       Vec3d &pose);

    bool GetStaticMap();

    bool GetLaserPose();

    void TransformLaserscanToBaseFrame(double &angle_min,
                                       double &angle_increment,
                                       const sensor_msgs::LaserScan &laser_scan_msg);

  private:
    //Mutex
    std::mutex mutex_;

    //Algorithm object
    std::unique_ptr<Amcl> amcl_ptr_;

    //ROS Node handle
    ros::NodeHandle nh_;

    //TF
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
    std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

    //ROS Subscriber

    ros::Subscriber initial_pose_sub_;
    ros::Subscriber vosion_location_sub;
    //ros::Subscriber pos_vi_sub;
    ros::Subscriber map_sub_;
    ros::Subscriber uwb_pose_sub_;
    ros::Subscriber ground_truth_sub_;
    ros::Subscriber sentry_pose_sub;
    ros::Subscriber lost_pose_sub;
    ros::Subscriber htc_pose_sub;
    ros::Subscriber laser_error_sub;
    ros::Subscriber angle_car_head_sub;
    ros::Subscriber theta_sub;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
    std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;

    //ROS Publisher
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::Publisher distance_map_pub_;
    ros::Publisher beam_sample_pub_;
    ros::Publisher pub_pos_vision;
    ros::Publisher vi_sign_pub;
    ros::Publisher map_test_pub_;
    ros::Publisher fort_mode_pub_;

    //ROS Service
    ros::ServiceServer random_heading_srv_;
    ros::ServiceClient static_map_srv_;

    //Parameters
    std::string odom_frame_;
    std::string global_frame_;
    std::string base_frame_;
    std::string laser_topic_;
    Vec3d init_pose_;
    Vec3d init_cov_;
    //  bool enable_uwb_;
    ros::Duration transform_tolerance_;
    bool publish_visualize_;

    //Status
    bool initialized_ = false;
    bool map_init_ = false;
    bool laser_init_ = false;
    bool first_map_received_ = false;
    bool latest_tf_valid_ = false;
    bool sent_first_transform_ = false;
    bool publish_first_distance_map_ = false;
    bool vi_sign_ = 0;

    //Data
    HypPose hyp_pose_;              //AMCL位姿
    HypPose hyp_pose_1, hyp_pose_2; //最终位姿  / 辅助功能计算出来的位姿
    HypPose last_pose;              //上一时刻的位姿
    float pose_msg_pose_position_x_last = 7.5, pose_msg_pose_position_y_last = 4.5;
    geometry_msgs::PoseArray particlecloud_msg_;
    geometry_msgs::PoseStamped pose_msg_;
    // std_msgs::Float64MultiArray fort_mode_msg;
    ros::Time last_laser_msg_timestamp_;
    ros::Time last_laser_msg_timestamp_1;

    tf::Transform latest_tf_;
    tf::Stamped<tf::Pose> latest_odom_pose_;

     //此数组用于何处？？？
    float arry[20][3] = {{0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}, {0, 0, 25}};
    

    double updatearr_[4] = {0, 0, 0, 0}; //辅助功能回传的位姿及更新方式
    double updateway_ = 0;
    int robot_color;
    int color = 0;
    int robot_id = 0;
    double sentrypose[5] = {-1, -1, -1, -1, -1};
    double htcpose[4];
    int lostflag = 0;
    bool has_htc_pose = false;
    bool has_my_sentry_pose = false;
    bool has_sentry_pose = false;
    bool has_label_pose = false;
    bool laser_act = true;
    double angle_car_head = 0;
    ros::Time last_sentry_update;
    ros::Time last_sentry_recv;
    ros::Time last_htc_recv;
    ros::Time first_theta_stamp;
    ros::Time last_pub_time;
    bool origin_time = true;
    // Labelmsg labelmsg_ = {0, 0, 0, 0, 0, 0};
    //相机的位置向标签看
    //label_angle的角度为 相机与标签的连线 和 标签右侧的夹角
    float label_recv[4] = {0, 0, 0, -1}; //[x,z,angle,figure]
    double last_yaw = 0.0;
    int fort_mode = 0;
    double theta_ = 0;
    double x_pose = 0;
    double y_pose = 0;
    double x_vel = 0.0;
    double y_vel = 0.0;
    double delta_time = 0.0;
    bool get_first_theta = false;
    double first_theta = 0;
    double fort_x = 0.0;
    double fort_y = 0.0;

  };

} // roborts_localization

#endif // ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
