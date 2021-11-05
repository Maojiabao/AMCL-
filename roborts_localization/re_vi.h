//此文件用于将标签的位置信息及编号放入容器中PATROL_POINT中

#ifndef MY_FILE_H
#define MY_FILE_H

#include <fstream>
#include <ros/package.h>
#include<tf/tf.h>
int Bullet_NUM_First;
namespace roborts_localization
{
  ////
  class RobotPoint
  {
  	public :
    double x;
    double y;
    double z;
    double yaw;
    
    RobotPoint() : x(0),y(0),z(0),yaw(0)
    {}
    
    ~RobotPoint() = default;
  };
  ////
  class RobotInfo
  {
  public :
    std::string color;
    int number;
    
    RobotInfo() : color("RED"),number(1)
    {}
    
    ~RobotInfo() = default;
  };
 ////
  class RobotConfig
  {
  public:
    RobotInfo           ROBOT_INFO;
    RobotPoint          ADDBUFF_POINT;
    RobotPoint          ADDBULLET_POINT;
    RobotPoint          MYESCAPE_POINT1;
    RobotPoint          MYESCAPE_POINT2;
    
    std::vector <RobotPoint> PATROL_POINT;//std::vector定义的变量可存放多个值,一个vector为一维，两个则为２维
    
    std::vector <RobotPoint> BONUS_PENALTY_POINT;
    
    RobotConfig()
    {
      std::string RobotConfigPath = ros::package::getPath("roborts_localization") + "/re_vi/re_vi.txt";//ros::package ::getPath 需要find_package(catkin REQUIRED COMPONENTS roslib)
      ReadConfigFromTxt(RobotConfigPath);//用于读取巡逻点（视觉标签点）文件
    }
    void ReadPoint(RobotPoint& PointType, std::ifstream& infile)
    //ifstream是输入流，将输入流给地址infile
    {
      std::string s,ts;
      std::getline(infile, s);//从传给地址infile输入，赋值给变量s
      
      std::istringstream istring(s);////istringstream类用于执行C++风格的串流的输入操作//从字符串s中读取字符//一行字符串，默认以空格为分隔符把该行分隔开来
      istring >> ts;
      PointType.x = atof(ts.c_str());
      istring >> ts;
      PointType.y = atof(ts.c_str());		//atof把字符串转换成浮点数，直至遇到第一个空格
      istring >> ts;
      PointType.z = atof(ts.c_str());
      istring >> ts;
      PointType.yaw = atof(ts.c_str());
      //一个读取四个数据,用于读取单个数据
    }
    
    void ReadPoint(std::vector<RobotPoint>& PointType, std::ifstream& infile)
    {       
      while(1)
      {
        std::string s,ts;
        std::getline(infile, s);
        if(s.empty())
        return;
        std::istringstream istring(s);
        RobotPoint TempPoint;
        istring >> ts;
        TempPoint.x = atof(ts.c_str());
        istring >> ts;
        TempPoint.y = atof(ts.c_str());
        istring >> ts;
        TempPoint.z = atof(ts.c_str());
        // istring >> ts;
        // TempPoint.yaw = atof(ts.c_str()); 
        //atof把字符串转换成浮点数，直至遇到第一个空格
        //c_str()函数返回一个指向正规C字符串的指针常量, 内容与本string串相同
		    PointType.push_back(TempPoint);
      }  
    }
    
    //读取配置文件中的参数//
    void ReadConfigFromTxt(std::string& file)
    {
      std::ifstream infile;//ifstream输入流的对象
      infile.open(file.data()); //打开文件my_config

      if(!infile.is_open())
      {
        ROS_ERROR("Can't open file : my_config.txt");
        return;
      }
      //返回一个C风格的字符串,只要不是空指针，就说明加载了文件
      std::string s,ts; 
      std::getline(infile, s);         //getline从输入流中读取字符, 并把它们转换成字符串
                                       //s为从输入流（txt文件）中读取字符，并将他们转化为字符串
      std::istringstream istring(s);     
	                               //它可以创建一个对象，然后这个对象就可以绑定一行字符串，然后以空格为分隔符把该行分隔开来
      istring >> ts;          //输入一个到ts
      ROBOT_INFO.color = ts;
      istring >> ts;
      ROBOT_INFO.number = atoi(ts.c_str());	//将读取到的字符串转化为整型数据
      
      if(ROBOT_INFO.color == "RED")
      {   
        while (std::getline(infile, s))
        {	    
        
        
          if(s == "RED1_PATROL_POINT" && ROBOT_INFO.number == 1)
          {
            ReadPoint(PATROL_POINT,infile);//此处调用的为void ReadPoint(std::vector<RobotPoint>& PointType, std::ifstream& infile)
            std::cout <<"RED1 PATROL_POINT is OK, and size is : "<<PATROL_POINT.size()<<std::endl;
          }
        
        
        }
      }
      else
      {
	      ROS_ERROR("Color is wrong!!!");
      }
      
      
      std::cout <<"***************************"<<std::endl;
      int i;
      for(i=0;i<32;i++)
      {
      std::cout << "x= " << PATROL_POINT[i].x <<"y= " << PATROL_POINT[i].y<< std::endl;
      }
      std::cout <<"***************************"<<std::endl;
      
      infile.close();                
    }
   // 读txt文件//
    ~RobotConfig() = default;    
  };
 // 定义类RobotConfig///
}
roborts_localization::RobotConfig MyConfig;

#endif