#include "localization_node.h"//定义了roborts_localization的命名空间【Labelmsg标签    roborts_localization类】      
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
//#include "re_vi.h"//此文件用于将标签的位置信息及编号放入容器中PATROL_POINT中
#include <vector>
#include "tinyxml.h"
#include <ros/package.h>

using namespace std;

namespace roborts_localization
{
  LocalizationNode::LocalizationNode(std::string name)
  {
    cout<<"【LocalizationNode】初始化"<<endl;
    CHECK(Init()) << "Module " << name << " initialized failed!";
    initialized_ = true;
    cout<<"【LocalizationNode】结束"<<endl;
  }

//如果地图信息和激光雷达都获得到，就ture
  bool LocalizationNode::Init()   
  {
    LocalizationConfig localization_config;
    localization_config.GetParam(&nh_);
    odom_frame_ = std::move(localization_config.odom_frame_id);//move为性能而生，直接转移，不用再创建，拷贝等等
    global_frame_ = std::move(localization_config.global_frame_id);
    base_frame_ = std::move(localization_config.base_frame_id);
    laser_topic_ = std::move(localization_config.laser_topic_name);
    init_pose_ = {localization_config.initial_pose_x,//撒粒子的方差，使粒子均匀的围绕初始化点  
                  localization_config.initial_pose_y, 
                  localization_config.initial_pose_a};
                  cout<<"【Init】参数位姿"<<localization_config.initial_pose_x<<" , "<<localization_config.initial_pose_y<<endl;
                  
    init_cov_ = {localization_config.initial_cov_xx,
                 localization_config.initial_cov_yy,
                 localization_config.initial_cov_aa};
    transform_tolerance_ = ros::Duration(localization_config.transform_tolerance);
    publish_visualize_ = localization_config.publish_visualize;
    std::cout << "【Init】完成读参" << std::endl;  
    
   
    //tf坐标变换（广播器    监听器）
    tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
    tf_listener_ptr_ = std::make_unique<tf::TransformListener>();
   //发布的话题 （地图  粒子云 Mp定位点   fort_mode）
    distance_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 1, true);
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 2, true);
    fort_mode_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fort_mode", 3, true);
    //订阅的话题（同步激光雷达和里程计数据）
    laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_topic_, 100);
    laser_scan_filter_ = std::make_unique<tf::MessageFilter<sensor_msgs::LaserScan>>(*laser_scan_sub_,
                                                                                     *tf_listener_ptr_,
                                                                                     odom_frame_,
                                                                                     100);//当scan消息能转换到odom坐标系中就调用回调函数
    laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::LaserScanCallback, this, _1));
    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::InitialPoseCallback, this);
  
    //标签定位
    //vosion_location_sub = nh_.subscribe("vision_tolocation",2,&LocalizationNode::vision_location_Callback, this);
    //哨岗定位
    sentry_pose_sub = nh_.subscribe("sentry_info", 3, &LocalizationNode::sentryCallback, this);
    //失效判断
    lost_pose_sub = nh_.subscribe("/laser_msg", 3, &LocalizationNode::lostCallback, this);//接收到一个数组，如果定位在地图外{1,?,?}，否则{0,?,?}
    //htc定位
    htc_pose_sub = nh_.subscribe("htc_info", 3, &LocalizationNode::htcCallback, this);
    //云台和底盘的机械角
    angle_car_head_sub = nh_.subscribe("info_tondecision", 3, &LocalizationNode::anglecarheadCallback, this);
    //雷达报错
    laser_error_sub = nh_.subscribe("laser_error", 3, &LocalizationNode::laseractCallback, this);
    //陀螺仪角度
    theta_sub = nh_.subscribe("theta_msg", 3, &LocalizationNode::thetaCallback, this);
  
    amcl_ptr_ = std::make_unique<Amcl>();
    amcl_ptr_->GetParamFromRos(&nh_);//从amcl_config获取参数----定位基础参数
    cout<<"【Init】初始位姿"<<init_pose_[0]<<" , "<<init_pose_[1]<<endl;
    amcl_ptr_->Init(init_pose_, init_cov_);//

    map_init_ = GetStaticMap();//获取静态地图成功----true
    laser_init_ = GetLaserPose();//获取到laser位姿----ture

    Read_costmap_Attr("decision");//得到robot_color和robot_id

    std::cout << "【Init】结束" << std::endl;  
    return map_init_ && laser_init_; 
  }
 

//获取静态地图  获取成功则调用 AMCL::HandleMapMessage
  bool LocalizationNode::GetStaticMap()
  {
    static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    ros::service::waitForService("static_map", -1);
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;
    if (static_map_srv_.call(req, res))
    {
      LOG_INFO << "Received Static Map";
      amcl_ptr_->HandleMapMessage(res.map, init_pose_, init_cov_);
      //将位姿保存在容器pose[] 和 cov[][]中
      //这个函数是主要的模型处理函数，它根据地图信息创造了地图空闲区域、粒子滤波器、里程计运动模型与激光感知的似然域模型
      first_map_received_ = true;
      return true;
    }
    else
    {
      LOG_ERROR << "Get static map failed";
      return false;
    }
  }
//得到雷达位姿      laser_pose(0，0，0)  laser---base
  bool LocalizationNode::GetLaserPose()
  {
    auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);

    Vec3d laser_pose;
    laser_pose.setZero();
    //ROS_ERRO_STREAM("GetLaserPose");
    GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
    laser_pose[2] = 0; // No need for rotation, or will be error
    DLOG_INFO << "Received laser's pose wrt robot: " << laser_pose[0] << ", " << laser_pose[1] << ", " << laser_pose[2];
    //cout<<"【雷达与base变换】获取成功"<< laser_pose[0] << ", " << laser_pose[1] << ", " << laser_pose[2]<<endl;

    amcl_ptr_->SetLaserSensorPose(laser_pose);
    return true;
  }
 //读取一个自己写的config.xml   //得到robot_color和robot_id
  void LocalizationNode::Read_costmap_Attr(const char *pszNode)
  {
    const char *pszXmlName;
    std::string str;
    std::string path = ros::package::getPath("decision") + "/config/config.xml";
    TiXmlDocument doc(path);
    if (!doc.LoadFile())
    {
      return;
    }
    TiXmlElement *p_root = doc.RootElement();
    for (TiXmlNode *p_node = p_root->FirstChildElement(); p_node; p_node = p_node->NextSiblingElement())
    {
      pszXmlName = p_node->Value();
      if (strcmp(pszXmlName, pszNode) == 0)
      {
        TiXmlElement *p_element = p_node->ToElement();
        for (TiXmlAttribute *p_attribute = p_element->FirstAttribute(); p_attribute; p_attribute = p_attribute->Next())
        {
          pszXmlName = p_attribute->Name();
          str = p_attribute->Value();
          if (str == "red1")
          {
            robot_color = 1; //红 1
            robot_id = 1;
          }
          else if (str == "red2")
          {
            robot_color = 1;
            robot_id = 2;
          }
          else if (str == "blue1")
          {
            robot_color = 2;
            robot_id = 1;
          }
          else if (str == "blue2")
          {
            robot_color = 2;
            robot_id = 2;
          }
          else
          {
            robot_color = -1;
            robot_id = -1;
          }
          break;
        }
      }
      break;
    }
    //颜色：红 1 ，蓝 2 。
    //装甲板数字：1,2 。
    std::cout << "********localization*********" << str << "," << robot_color << "," << robot_id << std::endl;
  }
 
 
  //从实际情况中获取位姿（确保初始位姿在原点）
  void LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg)
  {
    cout<<"initialpos回调函数"<<endl;
    if (init_pose_msg->header.frame_id == "")
    {
      LOG_WARNING << "Received initial pose with empty frame_id.";
    } // Only accept initial pose estimates in the global frame
    else if (tf_listener_ptr_->resolve(init_pose_msg->header.frame_id) !=
             tf_listener_ptr_->resolve(global_frame_))
    {
      LOG_ERROR << "Ignoring initial pose in frame \" "
                << init_pose_msg->header.frame_id
                << "\"; initial poses must be in the global frame, \""
                << global_frame_;
      return;
    }

    tf::StampedTransform tx_odom;
    try
    {
// 检查在时间time， source_frame能否变换到target_frame
// 它将休眠并重试每个polling_duration，直到超时的持续时间已经过去。

      ros::Time now = ros::Time::now();
      tf_listener_ptr_->waitForTransform(base_frame_,
                                         init_pose_msg->header.stamp,
                                         base_frame_,
                                         now,
                                         odom_frame_,
                                         ros::Duration(0.5));
      tf_listener_ptr_->lookupTransform(base_frame_,
                                        init_pose_msg->header.stamp,
                                        base_frame_,
                                        now,
                                        odom_frame_, tx_odom);
    }
    catch (tf::TransformException &e)
    {
      tx_odom.setIdentity();
    }
    tf::Pose pose_new;
    tf::Pose pose_old;
    tf::poseMsgToTF(init_pose_msg->pose.pose, pose_old);
    pose_new = pose_old * tx_odom;

    // Transform into the global frame
    DLOG_INFO << "Setting pose " << ros::Time::now().toSec() << ", "
              << pose_new.getOrigin().x() << ", " << pose_new.getOrigin().y();

    Vec3d init_pose_mean;
    Mat3d init_pose_cov;
    init_pose_mean.setZero();
    init_pose_cov.setZero();
    double yaw, pitch, roll;
    init_pose_mean(0) = pose_new.getOrigin().x();
    init_pose_mean(1) = pose_new.getOrigin().y();
    pose_new.getBasis().getEulerYPR(yaw, pitch, roll);
    init_pose_mean(2) = yaw;
    init_pose_cov = math::MsgCovarianceToMat3d(init_pose_msg->pose.covariance);

    amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);
  }

//调用TransformLaserscanToBaseFrame（0，0，激光雷达消息）GetPoseFromTf从TF树上获取某时刻的位姿 AMCL::update更新信息
  void LocalizationNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr)
  {
    double angle_min = 0, angle_increment = 0;
    sensor_msgs::LaserScan laser_scan_msg = *laser_scan_msg_ptr;
    TransformLaserscanToBaseFrame(angle_min, angle_increment, laser_scan_msg);
    //如果雷达正常工作
    if (laser_act)
    {
      last_laser_msg_timestamp_ = laser_scan_msg_ptr->header.stamp;
      first_theta_stamp = ros::Time::now();
      get_first_theta = true;

      //从TF上获取odom位姿  // 从TF树上获取某时刻的位姿
      Vec3d pose_in_odom;
      if (!GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom))
      {
        LOG_ERROR << "Couldn't determine robot's pose";
        return;
      }

      amcl_ptr_->Update(pose_in_odom,
                        laser_scan_msg,
                        angle_min,
                        angle_increment,
                        particlecloud_msg_,
                        hyp_pose_,
                        updatearr_);

      LOG_ERROR_IF(!PublishTf()) << "Publish Tf Error!";

      if (publish_visualize_)
      {
        PublishVisualize();
      }
    }
  }

//哨岗定位回调函数   （哨岗辅助定位）GetPoseFromTf
  void LocalizationNode::sentryCallback(const std_msgs::Float64MultiArray::ConstPtr &sentry_msg)
  {
    has_sentry_pose = true;
    if (origin_time)
    {
      last_sentry_update = ros::Time::now();
      last_sentry_recv = ros::Time::now();
      origin_time = false;
    }

    int j = 0;
    //去除重复帧及乱码
    if (sentrypose[0] != sentry_msg->data[0] &&
        sentry_msg->data[0] >= 1 &&
        sentry_msg->data[0] < 10000000)
    {
      for (int i = 0; i < 48; i += 8)
      {
        //取用条件为，颜色匹配,装甲板数字匹配，坐标在地图内
        if (sentry_msg->data[i + 1] == robot_color &&
            sentry_msg->data[i + 2] == robot_id &&
            sentry_msg->data[i + 3] < 8.0 &&
            sentry_msg->data[i + 3] > 0.5 &&
            sentry_msg->data[i + 4] < 4.2 &&
            sentry_msg->data[i + 4] > 0.5)
        {
          sentrypose[0] = sentry_msg->data[i];     //序号
          sentrypose[1] = sentry_msg->data[i + 1]; //颜色
          sentrypose[2] = sentry_msg->data[i + 2]; //数字
          sentrypose[3] = sentry_msg->data[i + 3]; //x
          sentrypose[4] = sentry_msg->data[i + 4]; //y
          j++;
          last_sentry_update = ros::Time::now();
        }
      }

      //当出现多次赋值或未赋值的情况时，不用此组数据
      if (j == 1)
      {
        has_my_sentry_pose = true;
        // ROS_INFO("NUM:   %d", int(sentrypose[0]));
        // ROS_INFO("COLOR: %d", int(sentrypose[1]));
        // ROS_INFO("ID:    %d", int(sentrypose[2]));
        // ROS_INFO("X:     %.3f", sentrypose[3]);
        // ROS_INFO("Y:     %.3f", sentrypose[4]);
        ROS_INFO("Sentry: %d, %d, %d, %.2f, %.2f",
                 int(sentrypose[0]),
                 int(sentrypose[1]),
                 int(sentrypose[2]),
                 sentrypose[3],
                 sentrypose[4]);
      }
      else
      {
        has_my_sentry_pose = false;
      }
      last_sentry_recv = ros::Time::now();
    }

    if ((ros::Time::now() - first_theta_stamp).toSec() > 1.0 && (!has_htc_pose) && (!laser_act))
    {
      bool getposefromtf = false;
      bool publishtf = false;
      //雷达失效，1秒未连上htc时，哨岗和陀螺仪代替amcl计算位姿

      last_laser_msg_timestamp_ = ros::Time::now() - ros::Duration(0.03);
      Vec3d pose_in_odom;

      if (GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom))
      {
        ROS_INFO("GET POSE FROM TF OK");
        getposefromtf = true;
      }
      else
      {
        ROS_ERROR_STREAM("Couldn't determine robot's pose");
      }
      if (PublishTf())
      {
        ROS_INFO("PUBLISH TF OK");
        publishtf = true;
      }
      else
      {
        ROS_ERROR_STREAM("Couldn't PUBLISH TF");
      }
      PublishVisualize();
      if (getposefromtf && publishtf)
      {
        ROS_WARN("******   Battery mode has been activated. Your task is to survive !  ******");
      }
    }

    double no_my_data = (ros::Time::now() - last_sentry_update).toSec();
    double sentry_disc = (ros::Time::now() - last_sentry_recv).toSec();
    //超时判断
    if (no_my_data > 5 && sentry_disc < 10)
    {
      // ROS_INFO("my sentry pose has not been updated for %.2f seconds !", no_my_data);
    }
    if (sentry_disc > 10)
    {
      // ROS_WARN("sentry msg has disconnected for %.2f seconds !", sentry_disc);
    }
  }

//失效判断，雷达定位失效： lostflag=1
  void LocalizationNode::lostCallback(const std_msgs::Float64MultiArray::ConstPtr &lost_msg)
  {
    lostflag = lost_msg->data[0];
    // ROS_INFO("lostflag = %d", lostflag);
  }

//得到陀螺仪角度
  void LocalizationNode::thetaCallback(const std_msgs::Float64MultiArray::ConstPtr &theta_msg)
  {
    if (get_first_theta)
    {
      first_theta = theta_msg->data[2];
      get_first_theta = false;
    }
    theta_ = theta_msg->data[2];
    y_vel = theta_msg->data[0];
    x_vel = theta_msg->data[1];
    // ROS_INFO("x_vel = %.3f", theta_msg->data[1]);
    // ROS_INFO("y_vel = %.3f", theta_msg->data[0]);
    // ROS_INFO("theta_msg = %.3f", theta_msg->data[2]);
  }

//htc定位
  static long htc_cd = 0;
  void LocalizationNode::htcCallback(const std_msgs::Float64MultiArray::ConstPtr &htc_msg)
  {
    first_theta_stamp = ros::Time::now();
    get_first_theta = true;
    //当在地图内且不与上一帧重复时，取用
    if (htc_msg->data[0] > 0.1 && htc_msg->data[0] < 8.2 &&
        htc_msg->data[1] > 0.1 && htc_msg->data[1] < 4.5)
    {
      for (int i = 0; i < 3; i++)
      {
        htcpose[i] = htc_msg->data[i];
      }
      htcpose[3] = (htc_msg->data[3]) / 57.3;
      has_htc_pose = true;
    }
    else
    {
      has_htc_pose = false;
    }
    htc_cd++;
    bool getposefromtf = false;
    bool publishtf = false;
    //雷达失效时，htc代替amcl计算位姿
    if (!laser_act)
    {
      last_laser_msg_timestamp_ = ros::Time::now() - ros::Duration(0.03);
      Vec3d pose_in_odom;
      if (GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom))
      {
        ROS_INFO("GET POSE FROM TF OK");
        getposefromtf = true;
      }
      else
      {
        ROS_ERROR_STREAM("Couldn't determine robot's pose");
      }
      if (PublishTf())
      {
        ROS_INFO("PUBLISH TF OK");
        publishtf = true;
      }
      else
      {
        ROS_ERROR_STREAM("Couldn't PUBLISH TF");
      }
      PublishVisualize();
      if (getposefromtf && publishtf)
      {
        ROS_INFO("***************HTC LOCALIZATION WORK************");
      }
    }

    // ROS_INFO("htc_cd = %d", htc_cd);
    // ROS_INFO("htcpose[0] = %f", htcpose[0]); //x
    // ROS_INFO("htcpose[1] = %f", htcpose[1]); //y
    // ROS_INFO("htcpose[2] = %f", htcpose[2]);
    // ROS_INFO("htcpose[3] = %f", htcpose[3]); //yaw
    // ROS_INFO("has_htc_pose = %d", has_htc_pose);
  }
  
  //云台与车体的机械角angle_car_head
  void LocalizationNode::anglecarheadCallback(const std_msgs::Float64MultiArray::ConstPtr &angle_msg)
  {
    //云台与车体的机械角，方向左正右负
    angle_car_head = angle_msg->data[0];
    // ROS_INFO("angle_car_head = %f", angle_car_head);
  }
 
 //雷达报错 laser_act = false;
  void LocalizationNode::laseractCallback(const std_msgs::Float64MultiArray::ConstPtr &laseract_msg)
  {
    if (laseract_msg->data[0] == 0)
    {
      laser_act = false;
      ROS_ERROR_STREAM("**************LASER ERROR , SWITCH TO HTC**************" << laser_act);
    }
    else if (laseract_msg->data[0] == 1)
    {
      laser_act = true;
    }
  }

//标签定位（被注释了）
  // vector<double> label;
  // void LocalizationNode::vision_location_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  // {
  //   label_recv[0] = (msg->data[0]) / 1000;
  //   label_recv[1] = (msg->data[1]) / 1000;
  //   label_recv[2] = (msg->data[2]) * 3.1415 / 180;
  //   label_recv[3] = msg->data[3];
  //   labelmsg_.arfa = atan(label_recv[0] / label_recv[1]);
  //   labelmsg_.distance = sqrt(label_recv[0] * label_recv[0] + label_recv[1] * label_recv[1]);
  //   //用相机发来的数据解算标签的朝向与大概的坐标
  //   Getlabelpose(hyp_pose_, labelmsg_, label_recv);
  //   //用计算出来的标签与标准标签位置做对比，若与某个标准位置方向相同，位置相差较小，则将此标准位置赋给此标签，并写入容器中
  //   for (int i = 0; i < 28; i++)
  //   {
  //     if (labelmsg_.toward == MyConfig.PATROL_POINT[i].z &&
  //         fabs(labelmsg_.x - MyConfig.PATROL_POINT[i].x) < 0.3 &&
  //         fabs(labelmsg_.y - MyConfig.PATROL_POINT[i].y) < 0.3)
  //     {
  //       has_label_pose = true;
  //       if (label.size() == 0)
  //       {
  //         label.push_back(MyConfig.PATROL_POINT[i].x);
  //         label.push_back(MyConfig.PATROL_POINT[i].y);
  //         label.push_back(labelmsg_.toward);
  //         label.push_back(label_recv[3]);
  //         /***********************************将采集到的标签写入kk.txt***********************************/
  //         ofstream kk;                                                                  //基本变量
  //         kk.open("/home/tdt/gitee/src/RoboRTS/roborts_localization/kk.txt", ios::app); //流写入位置
  //         kk << MyConfig.PATROL_POINT[i].x << "\t" << MyConfig.PATROL_POINT[i].y << "\t" << labelmsg_.toward << "\t" << label_recv[3] << endl;
  //         kk.close();
  //         /***********************************将采集到的标签写入kk.txt***********************************/
  //       }
  //       else
  //       {
  //         int num_1 = 0;
  //         for (int j = 3; j < label.size();)
  //         {
  //           if (label_recv[3] == label[j])
  //           {
  //             num_1++;
  //           }
  //           j += 4;
  //         }
  //         if (num_1 == 0)
  //         {
  //           label.push_back(MyConfig.PATROL_POINT[i].x);
  //           label.push_back(MyConfig.PATROL_POINT[i].y);
  //           label.push_back(labelmsg_.toward);
  //           label.push_back(label_recv[3]);
  //           /***********************************将采集到的标签写入kk.txt***********************************/
  //           ofstream kk;                                                                  //基本变量
  //           kk.open("/home/tdt/gitee/src/RoboRTS/roborts_localization/kk.txt", ios::app); //流写入位置
  //           kk << MyConfig.PATROL_POINT[i].x << "\t" << MyConfig.PATROL_POINT[i].y << "\t" << labelmsg_.toward << "\t" << label_recv[3] << endl;
  //           kk.close();
  //           /***********************************将采集到的标签写入kk.txt***********************************/
  //         }
  //       }
  //     }
  //     else
  //     {
  //       has_label_pose = false;
  //     }
  //   }
  // }

//发布到可视化
  void LocalizationNode::PublishVisualize()
  {
    if (pose_pub_.getNumSubscribers() > 0)//"发布amcl_pose"话题信息
    {
      pose_msg_.header.stamp = ros::Time::now();
      pose_msg_.header.frame_id = global_frame_;
      pose_msg_.pose.position.x = hyp_pose_1.pose_mean[0];
      pose_msg_.pose.position.y = hyp_pose_1.pose_mean[1];
      pose_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(hyp_pose_1.pose_mean[2]);
      pose_pub_.publish(pose_msg_);

      pose_msg_pose_position_x_last = pose_msg_.pose.position.x;
      pose_msg_pose_position_y_last = pose_msg_.pose.position.y;
    }

    if (particlecloud_pub_.getNumSubscribers() > 0)//发布粒子云信息
    {
      particlecloud_msg_.header.stamp = ros::Time::now();
      particlecloud_msg_.header.frame_id = global_frame_;
      particlecloud_pub_.publish(particlecloud_msg_); 
    }

    if (!publish_first_distance_map_)
    {
      distance_map_pub_.publish(amcl_ptr_->GetDistanceMapMsg());
      publish_first_distance_map_ = true;
    }
  }
 
  //发布TF
  static long localization_cd = 0;
  bool LocalizationNode::PublishTf()
  {
    ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);

    if (!laser_act)
    {

      if (has_htc_pose)
      { // //htc帧率过高，限制其速度；
        // if (localization_cd % 2 == 0)
        // {
        //   return false;
        // }
        //当雷达失效时,直接用htc代替AMCL
        hyp_pose_2.pose_mean[0] = htcpose[0];
        hyp_pose_2.pose_mean[1] = htcpose[1];
        if (htcpose[3] - angle_car_head > 3.1415)
        {
          hyp_pose_2.pose_mean[2] = htcpose[3] - angle_car_head - 6.283;
        }
        else if (htcpose[3] - angle_car_head < -3.1415)
        {
          hyp_pose_2.pose_mean[2] = htcpose[3] - angle_car_head + 6.283;
        }
        else
        {
          hyp_pose_2.pose_mean[2] = htcpose[3] - angle_car_head;
        }
        updateway_ = 0;
        FinalPose(hyp_pose_1, hyp_pose_2, updatearr_, updateway_);
        label_recv[3] = -1;
        has_htc_pose = false;
        has_my_sentry_pose = false;
        has_label_pose = false;
        fort_mode = 0;
        std_msgs::Float64MultiArray fort_mode_msg;
        fort_mode_msg.data.push_back(fort_mode);
        fort_mode_pub_.publish(fort_mode_msg);
        fort_mode_msg.data.clear();
        //ROS_INFO("POSE_HTC: %f,%f,%f", htcpose[0], htcpose[1], htcpose[2]);
        ROS_INFO("POSE: %.3f,%.3f,%.3f", hyp_pose_1.pose_mean[0], hyp_pose_1.pose_mean[1], hyp_pose_1.pose_mean[2]);
        last_yaw = hyp_pose_1.pose_mean[2];
      }
      else if ((ros::Time::now() - first_theta_stamp).toSec() > 0.5)
      {

        if (has_my_sentry_pose)
        {
          hyp_pose_1.pose_mean[0] = sentrypose[3];
          hyp_pose_1.pose_mean[1] = sentrypose[4];
          hyp_pose_1.pose_mean[2] = last_yaw + theta_ - first_theta;
        }
        else
        {
          delta_time = (ros::Time::now() - last_pub_time).toSec();
          fort_x = hyp_pose_1.pose_mean[0] + ((x_vel * delta_time) * cos(hyp_pose_1.pose_mean[2]) - (y_vel * delta_time) * cos(hyp_pose_1.pose_mean[2] - 1.57));
          fort_y = hyp_pose_1.pose_mean[1] + ((x_vel * delta_time) * sin(hyp_pose_1.pose_mean[2]) - (y_vel * delta_time) * sin(hyp_pose_1.pose_mean[2] - 1.57));
          if (fort_x < 8.0 && fort_x > 0.5)
          {
            hyp_pose_1.pose_mean[0] += ((x_vel * delta_time) * cos(hyp_pose_1.pose_mean[2]) - (y_vel * delta_time) * cos(hyp_pose_1.pose_mean[2] - 1.57));
          }
          if (fort_y < 4.35 && fort_y > 0.5)
          {
            hyp_pose_1.pose_mean[1] += (((x_vel * delta_time) * sin(hyp_pose_1.pose_mean[2]) - (y_vel * delta_time) * sin(hyp_pose_1.pose_mean[2] - 1.57)));
          }

          hyp_pose_1.pose_mean[2] = last_yaw + theta_ - first_theta;
        }
        ROS_INFO("FORT,%f,%f,%f", last_yaw, theta_, first_theta);
        std_msgs::Float64MultiArray fort_mode_msg;
        fort_mode = 1;
        fort_mode_msg.data.push_back(fort_mode);
        fort_mode_pub_.publish(fort_mode_msg);
        fort_mode_msg.data.clear();
        ROS_WARN("htc fail,switch to sentry,fort mode");
        ROS_INFO("POSE: %.3f,%.3f,%.3f", hyp_pose_1.pose_mean[0], hyp_pose_1.pose_mean[1], hyp_pose_1.pose_mean[2]);
      }
      last_pub_time = ros::Time::now();
      //tf转换
      // Subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        //tmp_tf是base_link在global map下的坐标，即base-->map
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_1.pose_mean[2]), //
                             tf::Vector3(hyp_pose_1.pose_mean[0],                  //
                                         hyp_pose_1.pose_mean[1],                  //
                                         0.0));
        //tmp_tf.inverse()是输入，tmp_tf_stamped.pose是输出
        //tmp_tf_stamped是global map原点在base_link下的坐标，即map-->base
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), //
                                             last_laser_msg_timestamp_,
                                             base_frame_); //

        //将global map原点在base_link下的坐标变换成global map原点在odom下的坐标
        //即map-->odom，相当于在odom原点看map原点的位置
        //这里的odom_to_map并非真的odom-->map，而是反过来map-->odom
        this->tf_listener_ptr_->transformPose(odom_frame_,    //
                                              tmp_tf_stamped, //
                                              odom_to_map);   //
      }
      catch (tf::TransformException &e)
      {
        LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
        return false;
      }
      //转换odom_to_map.pose为latest_tf_
      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;
      //tmp_tf_stamped这个变换是odom原点在map坐标系的坐标，即odom-->map
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), //
                                          transform_expiration,
                                          global_frame_,
                                          odom_frame_);
      this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
      sent_first_transform_ = true;
      return true;
    }
    if (amcl_ptr_->CheckTfUpdate())
    {
      float error_sign_x = 0, error_sign_y = 0, error_sign_angle = 0;
      localization_cd++;
      //每十次循环(连续运动时约两秒)或定位失效时，获取htc/哨岗/标签位姿进行对比找回定位
      if (localization_cd % 20 == 8 || lostflag > 100)
      {
        //如果有htc
        if (has_htc_pose &&
            !(fabs(htcpose[0] - hyp_pose_.pose_mean[0]) < 0.5 &&
              fabs(htcpose[1] - hyp_pose_.pose_mean[1]) < 0.5))
        {
          hyp_pose_2.pose_mean[0] = htcpose[0];
          hyp_pose_2.pose_mean[1] = htcpose[1];
          if (htcpose[3] - angle_car_head > 3.1415)
          {
            hyp_pose_2.pose_mean[2] = htcpose[3] - angle_car_head - 6.283;
          }
          else if (htcpose[3] - angle_car_head < -3.1415)
          {
            hyp_pose_2.pose_mean[2] = htcpose[3] - angle_car_head + 6.283;
          }
          else
          {
            hyp_pose_2.pose_mean[2] = htcpose[3] - angle_car_head;
          }
          updateway_ = 3;
          FinalPose(hyp_pose_1, hyp_pose_2, updatearr_, updateway_);
        }
        // 如果有哨岗
        else if (has_my_sentry_pose &&
                 !(fabs(sentrypose[3] - hyp_pose_.pose_mean[0]) < 0.8 &&
                   fabs(sentrypose[4] - hyp_pose_.pose_mean[1]) < 0.8))
        {
          hyp_pose_2.pose_mean[0] = sentrypose[3];
          hyp_pose_2.pose_mean[1] = sentrypose[4];
          hyp_pose_2.pose_mean[2] = hyp_pose_.pose_mean[2];
          updateway_ = 2;
          FinalPose(hyp_pose_1, hyp_pose_2, updatearr_, updateway_);
        }
        // 如果有标签定位
        /* else if (has_label_pose)
        {
          //如果标签库不为空
          if (label.size() != 0)
          {
            //遍历标签库
            for (int i = 3; i < label.size(); i += 4)
            {
              // 当匹配到标签时
              if (label_recv[3] == label[i])
              {

                //标签及re_vi文件使用说明：
                //标签的序号暂定为32个,9个障碍物分为三行，每行三个，从距离原点（0,0）最近的障碍物开始，向x正方向计数，每行都是向X正方向计数
                //因四个角落的障碍物都有一面与墙接触，面向X+ / X- / Y+ / Y-方向的面各有七个，故共有32个面贴有标签。
                //在re_vi文件中，四列数据分别为，标签的x坐标，y坐标，方向，标签上面的字符。
                //标签坐标的顺序是按上述方式排列的，使用时不要打乱顺序，仅可修改最后一列的字符对应的数字

                //面向Y正方向的标签
                if (label[i - 1] == 2)
                {
                  labelmsg_.sita = 3.14 - label_recv[2] + labelmsg_.arfa;
                  hyp_pose_2.pose_mean[0] = label[i - 3] - labelmsg_.distance * cos(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                  hyp_pose_2.pose_mean[1] = label[i - 2] + labelmsg_.distance * sin(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                  hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                }
                //面向Y负方向的标签
                else if (label[i - 1] == 4)
                {
                  labelmsg_.sita = -label_recv[2] + labelmsg_.arfa;
                  hyp_pose_2.pose_mean[0] = label[i - 3] + labelmsg_.distance * cos(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                  hyp_pose_2.pose_mean[1] = label[i - 2] - labelmsg_.distance * sin(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                  hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                }
                //面向X正方向的标签
                else if (label[i - 1] == 3)
                {
                  labelmsg_.sita = 1.571 - label_recv[2] + labelmsg_.arfa;
                  hyp_pose_2.pose_mean[0] = label[i - 3] + labelmsg_.distance * sin(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                  hyp_pose_2.pose_mean[1] = label[i - 2] + labelmsg_.distance * cos(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                  hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                }
                //面向X负方向的标签
                //此时角度label_recv[2]有从-pi到pi突变的情况，因此分开讨论
                else if (label[i - 1] == 1)
                {
                  //当车位于标签左侧（从相机看向标签）,且相机光轴与标签右侧夹角大于90度时
                  if (label_recv[2] > 1.571 && label_recv[2] - labelmsg_.arfa > 1.5707963)
                  {
                    labelmsg_.sita = 4.71 - label_recv[2] + labelmsg_.arfa;
                    hyp_pose_2.pose_mean[0] = label[i - 3] - labelmsg_.distance * sin(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                    hyp_pose_2.pose_mean[1] = label[i - 2] - labelmsg_.distance * cos(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                    hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                  }
                  //当车位于标签左侧（从相机看向标签）,且相机光轴与标签右侧夹角小于90度时
                  else if (label_recv[2] > 1.571 && label_recv[2] - labelmsg_.arfa < 1.5707963)
                  {
                    labelmsg_.sita = -1.571 - label_recv[2] + labelmsg_.arfa;
                    hyp_pose_2.pose_mean[0] = label[i - 3] - labelmsg_.distance * sin(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                    hyp_pose_2.pose_mean[1] = label[i - 2] - labelmsg_.distance * cos(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                    hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                  }
                  //当车位于标签右侧（从相机看向标签）,且相机光轴与标签右侧夹角大于90度时
                  else if (label_recv[2] < 1.571 && label_recv[2] - labelmsg_.arfa > 1.5707963)
                  {
                    labelmsg_.sita = 4.71 - label_recv[2] + labelmsg_.arfa;
                    hyp_pose_2.pose_mean[0] = label[i - 3] - labelmsg_.distance * sin(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                    hyp_pose_2.pose_mean[1] = label[i - 2] - labelmsg_.distance * cos(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                    hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                  }
                  //当车位于标签右侧（从相机看向标签）,且相机光轴与标签右侧夹角小于90度时
                  else
                  {
                    labelmsg_.sita = -1.571 - label_recv[2] + labelmsg_.arfa;
                    hyp_pose_2.pose_mean[0] = label[i - 3] - labelmsg_.distance * sin(label_recv[2]) + 0.3 * cos(labelmsg_.sita);
                    hyp_pose_2.pose_mean[1] = label[i - 2] - labelmsg_.distance * cos(label_recv[2]) + 0.3 * sin(labelmsg_.sita);
                    hyp_pose_2.pose_mean[2] = labelmsg_.sita;
                  }
                }
                //////////////////////////////////////////////////////////////////////////////
                ///////      场地中心45度斜放的障碍物的标签实际效果不佳，暂不使用     ////////////////
                //////////////////////////////////////////////////////////////////////////////
                //其它未知情况
                else
                {
                  std::cout << "无此标签" << std::endl;
                  break;
                }
                error_sign_x = hyp_pose_2.pose_mean[0] - hyp_pose_.pose_mean[0];
                error_sign_y = hyp_pose_2.pose_mean[1] - hyp_pose_.pose_mean[1];
                error_sign_angle = hyp_pose_2.pose_mean[2] - hyp_pose_.pose_mean[2];

                //与标签对比，若定位迷失
                if (fabs(error_sign_x) > 0.2 || fabs(error_sign_y) > 0.2 || fabs(error_sign_angle) > 0.18)
                {
                  std::cout << "!! !!!机器人迷失!!!!!   =" << std::endl;
                  updateway_ = 1;
                  FinalPose(hyp_pose_1, hyp_pose_2, updatearr_, updateway_);
                  break;
                }
                //若未迷失
                else
                {
                  std::cout << "------------------未迷失---------------------" << std::endl;
                  updateway_ = 0;
                  FinalPose(hyp_pose_1, hyp_pose_, updatearr_, updateway_);
                }
              }
              //若未匹配到标签
              else
              {
                updateway_ = 0;
                FinalPose(hyp_pose_1, hyp_pose_, updatearr_, updateway_);
              }
            }
          }
          //如果标签库为空
          else
          {
            updateway_ = 0;
            FinalPose(hyp_pose_1, hyp_pose_, updatearr_, updateway_);
          }
        } */
       
        //如果无htc/哨岗/标签时
        else if (lostflag > 20)
        {
          updateway_ = 4;
          FinalPose(hyp_pose_1, hyp_pose_, updatearr_, updateway_);
        }
        else
        {
          updateway_ = 0;
          FinalPose(hyp_pose_1, hyp_pose_, updatearr_, updateway_);
        }
      }
      //未与htc/哨岗/标签对比时
      else
      {
        updateway_ = 0;
        FinalPose(hyp_pose_1, hyp_pose_, updatearr_, updateway_);
      }
      //执行完毕后，参数初始化
      label_recv[3] = -1;
      has_htc_pose = false;
      has_my_sentry_pose = false;
      has_label_pose = false;
      last_yaw = hyp_pose_1.pose_mean[2];
      //炮台模式，0为不开启，1为开启
      fort_mode = 0;
      std_msgs::Float64MultiArray fort_mode_msg;
      fort_mode_msg.data.push_back(fort_mode);
      fort_mode_pub_.publish(fort_mode_msg);
      fort_mode_msg.data.clear();
      ROS_INFO("POSE: %.3f,%.3f,%.3f", hyp_pose_1.pose_mean[0], hyp_pose_1.pose_mean[1], hyp_pose_1.pose_mean[2]);
      last_pub_time = ros::Time::now();
      last_yaw = hyp_pose_1.pose_mean[2];
      //tf转换
      // Subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        //tmp_tf是base_link在global map下的坐标，即base-->map
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_1.pose_mean[2]), //
                             tf::Vector3(hyp_pose_1.pose_mean[0],                  //
                                         hyp_pose_1.pose_mean[1],                  //
                                         0.0));                                    //

        //tmp_tf.inverse()是输入，tmp_tf_stamped.pose是输出
        //tmp_tf_stamped是global map原点在base_link下的坐标，即map-->base
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), //
                                             last_laser_msg_timestamp_,
                                             base_frame_); //

        //将global map原点在base_link下的坐标变换成global map原点在odom下的坐标
        //即map-->odom，相当于在odom原点看map原点的位置
        //这里的odom_to_map并非真的odom-->map，而是反过来map-->odom
        this->tf_listener_ptr_->transformPose(odom_frame_,    //
                                              tmp_tf_stamped, //
                                              odom_to_map);   //
      }
      catch (tf::TransformException &e)
      {
        LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
        return false;
      }
      //转换odom_to_map.pose为latest_tf_
      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      //tmp_tf_stamped这个变换是odom原点在map坐标系的坐标，即odom-->map
      tf::StampedTransform tmp_tf_stamped1(latest_tf_.inverse(), //
                                           transform_expiration,
                                           global_frame_,
                                           odom_frame_);
      // this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped1);
      sent_first_transform_ = true;
      return true;
    }
    else if (latest_tf_valid_)
    {
      // Nothing changed, so we'll just republish the last transform
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          global_frame_,
                                          odom_frame_);
      this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
      return true;
    }
    else
    {
      return false;
    }
  }
 
  // 从TF树上获取某时刻的位姿
  bool LocalizationNode::GetPoseFromTf(const std::string &target_frame,
                                       const std::string &source_frame,
                                       const ros::Time &timestamp,
                                       Vec3d &pose)
  {

    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                              tf::Vector3(0, 0, 0)),
                                timestamp,
                                source_frame);
    tf::Stamped<tf::Pose> pose_stamp;
    try
    {
      this->tf_listener_ptr_->transformPose(target_frame,
                                            ident,
                                            pose_stamp);
    }
    catch (tf::TransformException &e)
    {
      LOG_ERROR << "Couldn't transform from "
                << source_frame
                << " to "
                << target_frame;
      return false;
    }

    pose.setZero();
    pose[0] = pose_stamp.getOrigin().x();
    pose[1] = pose_stamp.getOrigin().y();
    double yaw, pitch, roll;
    pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
    pose[2] = yaw;
    return true;
  }

  void LocalizationNode::TransformLaserscanToBaseFrame(double &angle_min,
                                                       double &angle_increment,
                                                       const sensor_msgs::LaserScan &laser_scan_msg)
  {

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    // Construct min and max angles of laser, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan_msg.angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan_msg.header.stamp,
                                      laser_scan_msg.header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan_msg.angle_min + laser_scan_msg.angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan_msg.header.stamp,
                                      laser_scan_msg.header.frame_id);

    try
    {
      tf_listener_ptr_->transformQuaternion(base_frame_,
                                            min_q,
                                            min_q);
      tf_listener_ptr_->transformQuaternion(base_frame_,
                                            inc_q,
                                            inc_q);
    }
    catch (tf::TransformException &e)
    {
      LOG_WARNING << "Unable to transform min/max laser angles into base frame: " << e.what();
      return;
    }

    angle_min = tf::getYaw(min_q);
    angle_increment = (tf::getYaw(inc_q) - angle_min);

    // Wrapping angle to [-pi .. pi]
    angle_increment = (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);
  }

  void LocalizationNode::Getlabelpose(HypPose &hyp_pose,
                                      Labelmsg &labelmsg,
                                      const float *labelarr)
  {
    if (hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) > 225 / 57.3f &&
            hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) < 315 / 57.3f ||
        hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) > -135 / 57.3f &&
            hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) < -45 / 57.3f)
    {
      labelmsg.toward = 1;
      labelmsg.x = hyp_pose.pose_mean[0] - 0.3 * cos(hyp_pose.pose_mean[2]) + labelmsg.distance * sin(*(labelarr + 2));
      labelmsg.y = hyp_pose.pose_mean[1] - 0.3 * sin(hyp_pose.pose_mean[2]) + labelmsg.distance * cos(*(labelarr + 2));
    }
    else if (hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) > 135 / 57.3f &&
             hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) < 225 / 57.3f)
    {
      labelmsg.toward = 2;
      labelmsg.x = hyp_pose.pose_mean[0] - 0.3 * cos(hyp_pose.pose_mean[2]) + labelmsg.distance * cos(*(labelarr + 2));
      labelmsg.y = hyp_pose.pose_mean[1] - 0.3 * sin(hyp_pose.pose_mean[2]) - labelmsg.distance * sin(*(labelarr + 2));
    }
    else if (hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) > 45 / 57.3f &&
             hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) < 135 / 57.3f)
    {
      labelmsg.toward = 3;
      labelmsg.x = hyp_pose.pose_mean[0] - 0.3 * cos(hyp_pose.pose_mean[2]) - labelmsg.distance * sin(*(labelarr + 2));
      labelmsg.y = hyp_pose.pose_mean[1] - 0.3 * sin(hyp_pose.pose_mean[2]) - labelmsg.distance * cos(*(labelarr + 2));
    }
    else if (hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) > -45 / 57.3f &&
             hyp_pose.pose_mean[2] - labelmsg.arfa + *(labelarr + 2) < 45 / 57.3f)
    {
      labelmsg.toward = 4;
      labelmsg.x = hyp_pose.pose_mean[0] - 0.3 * cos(hyp_pose.pose_mean[2]) - labelmsg.distance * cos(*(labelarr + 2));
      labelmsg.y = hyp_pose.pose_mean[1] - 0.3 * sin(hyp_pose.pose_mean[2]) + labelmsg.distance * sin(*(labelarr + 2));
    }
  }

  void LocalizationNode::FinalPose(HypPose &hyp_pose_new,
                                   HypPose &hyp_pose_old,
                                   double *updatearr,
                                   const double &updateway)
  {
    hyp_pose_new.pose_mean[0] = hyp_pose_old.pose_mean[0];
    hyp_pose_new.pose_mean[1] = hyp_pose_old.pose_mean[1];
    hyp_pose_new.pose_mean[2] = hyp_pose_old.pose_mean[2];
    *updatearr = hyp_pose_new.pose_mean[0];
    *(updatearr + 1) = hyp_pose_new.pose_mean[1];
    *(updatearr + 2) = hyp_pose_new.pose_mean[2];
    *(updatearr + 3) = updateway;
  }

} // roborts_localization

int main(int argc, char **argv)
{ 
  roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "localization_node");
  roborts_localization::LocalizationNode localization_node("localization_node");
  std::cout<<"【main】多线程开始"<<std::endl;
  ros::AsyncSpinner async_spinner(THREAD_NUM);//在一个节点中开辟多个线程 
  async_spinner.start();
  ros::waitForShutdown();  
  std::cout<<"【main】结束"<<std::endl;
  return 0;
}
