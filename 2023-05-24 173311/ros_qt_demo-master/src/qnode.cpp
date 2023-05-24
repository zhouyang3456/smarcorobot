/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt_demo/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
    {
    qRegisterMetaType<poselistType>("poselistType");//注册自定义信号
    qRegisterMetaType<myType>("myType");//注册自定义信号
    qRegisterMetaType<statuetype>("statuetype");//注册自定义信号

//    读取topic的设置
    QSettings topic_setting("cyrobot_monitor","settings");
    odom_topic= topic_setting.value("topic/topic_odom","odom").toString();
//    batteryState_topic=topic_setting.value("topic/topic_power","battery").toString();
    batteryState_topic=topic_setting.value("topic/topic_power","PowerVoltage").toString();
    laser_topic=topic_setting.value("topic/topic_laser","scan").toString();
    pose_topic=topic_setting.value("topic/topic_amcl","mypose").toString();
    show_mode=topic_setting.value("main/show_mode","control").toString();
    m_frameRate=topic_setting.value("main/framerate",40).toInt();
    m_threadNum=topic_setting.value("main/thread_num",6).toInt();
    QSettings settings("cyrobot_monitor","Displays");
    map_topic=settings.value("Map/topic",QString("/map")).toString();
    laser_topic=settings.value("Laser/topic",QString("/scan")).toString();
    laser_frame = settings.value("frame/laserFrame","/base_scan").toString().toStdString();
    map_frame = settings.value("frame/mapFrame","/map").toString().toStdString();
    base_frame = settings.value("frame/baseFrame","/base_link").toString().toStdString();
     qRegisterMetaType<sensor_msgs::BatteryState>("sensor_msgs::BatteryState");
    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"cyrobot_monitor_"+show_mode.toStdString());
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  SubAndPubTopic();
  start();
  return true;
}

//初始化的函数*********************************
bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"ros_main");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  SubAndPubTopic();
  start();
  return true;
}

//创建订阅者与发布者
void QNode::SubAndPubTopic(){
  ros::NodeHandle n;
   // Add your ros communications here.
   //创建速度话题的订阅者
   cmdVel_sub =n.subscribe<geometry_msgs::Twist>("cmd_vel",200,&QNode::speedCallback,this);
   battery_sub=n.subscribe("PowerVoltage",1000,&QNode::batteryCallback,this);
   //地图订阅
   map_sub = n.subscribe("map",1000,&QNode::mapCallback,this);
   //机器人位置话题
   // pos_sub=n.subscribe("amcl_pose",1000,&QNode::poseCallback,this);
  pos_sub=n.subscribe("mypose",1000,&QNode::poseCallback,this);
   //导航目标点发送话题
   goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);

   addlinedatarequest = n.advertise<std_msgs::Int32>("DataRequest",1000);//发送电子围栏数据请求
   datarequest = n.advertise<std_msgs::Int32>("Smaro/DataRequest_SeverToCar",1000);//发送数据请求
   electronicfencemaster = n.subscribe<geometry_msgs::Polygon>("ElectronicFenceMaster",1000,&QNode::addlinepointcallback,this);//接收返回的电子围栏数据
   electronicfenceclient = n.advertise<geometry_msgs::Polygon>("ElectronicFenceClient",1000);//发送电子围栏数据

   returnpos_pub = n.advertise<geometry_msgs::Pose>("Smaro/BackHomePoint_SeverToCar",1000);//发送返回点数据
   returnpos_sub = n.subscribe<geometry_msgs::Pose>("Smaro/BackHomePoint_CarToSever",1000,&QNode::returnposcallback,this);//接收返回点数据

   multipos_pub = n.advertise<geometry_msgs::PoseArray>("Smaro/GoalPoints_SeverToCar",1000);//
   multipos_sub = n.subscribe<geometry_msgs::PoseArray>("Smaro/GoalPoints_CarToSever",1000,&QNode::multiposcallback,this);//接收巡航点数据

   statue_car = n.subscribe("Smaro/StatusNag_CarToSever",1000,&QNode::statue_car_callback,this);
   statue_error_car = n.subscribe("Smaro/StatusPolice_CarToSever",1000,&QNode::statue_error_car_callback,this);
   floor_car = n.subscribe("Smaro/StatusFloor_CarToSever",1000,&QNode::floor_car_callback,this);

   nag_statue = n.subscribe("Smarco/NagFailed_movebase",1000,&QNode::nag_statue_callback,this);//导航失败接收信号

   floornum_pub = n.advertise<std_msgs::Int32>("Smaro/StatusFloor_ServerToCar",1000);//
   
   express_pub = n.advertise<std_msgs::UInt8>("Smaro/OpenDoor_ServerToCar",1000);//开快递柜门
   check_express_pub = n.advertise<std_msgs::UInt8>("Smaro/DoorStatus_ServerToCar",1000);//查询快递柜门
   express_statue = n.subscribe("Smaro/DoorStatus_CarToServer",1000,&QNode::express_statue_callback,this);//返回快递柜状态

   //速度控制话题
   cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
   //当前位置设置话题
   current_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1000);
   //取消当前动作
   cancle_pub_ = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1000);
   //获取到达位置信号
   reached_signal = n.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result",1000,&QNode::statuecallback,this);
   //激光雷达点云话题订阅
   m_laserSub=n.subscribe("/scan",1000,&QNode::laserScanCallback,this);
   //m_rosTimer=n.createTimer(ros::Duration(1.0),boost::bind(&QNode::transformPoint,boost::ref(m_tfListener)));
   //全局规划Path
   m_plannerPathSub=n.subscribe("/move_base/GlobalPlanner/plan",1000,&QNode::plannerPathCallback,this);

   m_imu_sub = n.subscribe("/handsfree/imu",1000,&QNode::imu_Callback,this);

   fanDisplay_pub = n.advertise<std_msgs::Int32>("/Smacro/FanDisplay_ServerToCar",10);

   which_multi_point_achieve_sub = n.subscribe("Smacro/ReachPointNum_CarToSever",10,&QNode::ReachPointNum_Callback,this);

   // record gloabal path
   record_global_path_pub = n.advertise<nav_msgs::Path>("Smaro/record/Global_Path",10);//

   pub_test_ = n.advertise<std_msgs::Int32>("/Smacro/test_pub",10);



   image_transport::ImageTransport it(n);
//   m_imageMapPub = it.advertise("image/map",10);
   m_robotPoselistener =new tf::TransformListener;
   m_Laserlistener = new tf::TransformListener;
   try {
     m_robotPoselistener->waitForTransform(map_frame,base_frame,ros::Time(0),ros::Duration(0.4));
     m_Laserlistener->waitForTransform(map_frame,laser_frame,ros::Time(0),ros::Duration(0.4));
   } catch (tf::TransformException& ex) {
//      ROS_ERROR("Received an exception trying to transform a point from \"map\" to \"base_link\": %s", ex.what());
   }
}

void QNode::express_statue_callback(const std_msgs::UInt8MultiArray &msg)//快递柜状态返回函数
{
  uint8_t x,y;
  x = msg.data.at(0);
  y = msg.data.at(1);
  statuetype statue_express;
  statue_express.statue_type<<x<<y;
  emit sendexpressstatue(statue_express);
}

void QNode::imu_Callback(sensor_msgs::ImuConstPtr msg)
{
    emit send_imu_fault_signal();
}

void QNode::ReachPointNum_Callback(const std_msgs::Int32 &point_num_)
{
    qint32 num_ = point_num_.data;
    qDebug() << "ReachPointNum_Callback";
    emit reachPointNum_signal(num_);
}

void QNode::nag_statue_callback(const std_msgs::Int32 &msg)
{
  int nagstatue = msg.data;
  emit sendnagstatue(nagstatue);
}

void QNode::statue_car_callback(const std_msgs::Int32 &msg)
{
  int carstatue = msg.data;
  emit sendcarstatue(carstatue);
}

void QNode::statue_error_car_callback(const std_msgs::Int32 &msg)
{
  int carerrorstatue = msg.data;
  emit sendcarerrorstatue(carerrorstatue);
}

void QNode::floor_car_callback(const std_msgs::Int32 &msg)
{
  int floorcar = msg.data;
  emit sendfloorcar(floorcar);
}

//取消当前动作
void QNode::cancel_action()
{
  actionlib_msgs::GoalID first_id;
  cancle_pub_.publish(first_id);

}

void QNode::statuecallback(move_base_msgs::MoveBaseActionResult result)
{
  statue_result = result.status.status;
  emit updatecarmovestatue(statue_result);
  emit car_achieve_goal_point_signal(statue_result);
}

void QNode::transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
//    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

QMap<QString,QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString,QString> res;
    for(auto topic:topic_list)
    {
        res.insert(QString::fromStdString(topic.name),QString::fromStdString(topic.datatype));
    }
    return res;
}
//planner的路径话题回调
void QNode::plannerPathCallback(nav_msgs::Path::ConstPtr path){
     plannerPoints.clear();
     for(int i=0;i<path->poses.size();i++){
       QPointF roboPos=transMapPoint2Scene(QPointF(path->poses[i].pose.position.x,path->poses[i].pose.position.y));
       plannerPoints.append(roboPos);
     }
     emit plannerPath(plannerPoints);
}

//激光雷达点云话题回调
void QNode::laserScanCallback(sensor_msgs::LaserScanConstPtr laser_msg){
  geometry_msgs::PointStamped laser_point;
  geometry_msgs::PointStamped map_point;
  laser_point.header.frame_id = laser_msg->header.frame_id;
  std::vector<float> ranges = laser_msg->ranges;
  laserPoints.clear();
  //转换到二维XY平面坐标系下;
  for(int i=0; i< ranges.size(); i++)
  {
    //scan_laser坐标系下
    double angle = laser_msg->angle_min + i * laser_msg->angle_increment;
    double X = ranges[i] * cos(angle);
    double Y = ranges[i] * sin(angle);
    laser_point.point.x=X;
    laser_point.point.y=Y;
    laser_point.point.z = 0.0;
    //change to map frame
    try{
      m_Laserlistener->transformPoint(map_frame, laser_point, map_point);
    }
    catch(tf::TransformException& ex){
//      ROS_ERROR("Received an exception trying to transform  %s", ex.what());
    }
    //转化为图元坐标系
    QPointF roboPos = transMapPoint2Scene(QPointF(map_point.point.x,map_point.point.y));
    laserPoints.append(roboPos);
  }
  emit updateLaserScan(laserPoints);
  emit send_laser_fault_signal();

}

//机器人位置话题的回调函数
void QNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos)
{
      //坐标转化为图元坐标系
      QPointF roboPos=transMapPoint2Scene(QPointF(pos.pose.pose.position.x,pos.pose.pose.position.y));
      //yaw
      tf::Quaternion quat;
      tf::quaternionMsgToTF(pos.pose.pose.orientation, quat);
      double roll, pitch, yaw;//定义存储r\p\y的容器
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
      emit updateRoboPose(roboPos,yaw);
      emit current_point_signal_(pos.pose.pose.position.x,
                                 pos.pose.pose.position.y,
                                 pos.pose.pose.orientation.z,
                                 pos.pose.pose.orientation.w);
      // new add
      QPointF roboPos_raw = QPointF(pos.pose.pose.position.x,pos.pose.pose.position.y);
      // qDebug() << "posecallback: " << roboPos_raw;
      emit updateRoboPose_cloud(roboPos_raw, yaw);

}
//void QNode::batteryCallback(const sensor_msgs::BatteryState &message)
//{
//   emit batteryState(message.voltage);
//}
void QNode::batteryCallback(const std_msgs::Float32 &message)
{
   emit batteryState(message.data);
}
void QNode::myCallback(const std_msgs::Float64 &message_holder)
{
    qDebug()<<message_holder.data<<endl;
}

//发布导航目标点信息
void QNode::set_goal(QString frame,double x,double y,double z,double w)
{
    geometry_msgs::PoseStamped goal;
    //设置frame
    goal.header.frame_id=frame.toStdString();
    //设置时刻
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.position.z=0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z=z;
    goal.pose.orientation.w=w;
    goal_pub.publish(goal);
    ros::spinOnce();
}

//发布数据需求编号
void QNode::set_datarequest(int data)
{
  std_msgs::Int32 _data;
  _data.data = data;
  datarequest.publish(_data);
  ros::spinOnce();
}

void QNode::open_express(int data)//发送快递柜开柜
{
  std_msgs::UInt8 _data;
  _data.data = data;
  express_pub.publish(_data);
  ros::spinOnce();
}

void QNode::check_express(int data)//
{
  std_msgs::UInt8 _data;
  _data.data = data;
  check_express_pub.publish(_data);
  ros::spinOnce();
}


//发布数据需求编号
void QNode::addlineset_datarequest(int data)
{
  std_msgs::Int32 _data;
  _data.data = data;
  addlinedatarequest.publish(_data);
  ros::spinOnce();
}

void QNode::sendline_modify(QVector<QPolygonF> point_list)
{
  geometry_msgs::Polygon point_polygen;
  geometry_msgs::Point32 point_32;
  for (int j=0;j<point_list.count();j++)
  {
    point_32.x = point_list.at(j).at(0).x();
    point_32.y = point_list.at(j).at(0).y();
    point_32.z = 0.0;
    point_polygen.points.push_back(point_32);
    point_32.x = point_list.at(j).at(1).x();
    point_32.y = point_list.at(j).at(1).y();
    point_32.z = 0.0;
    point_polygen.points.push_back(point_32);
  }
  electronicfenceclient.publish(point_polygen);
}


void QNode::set_start(QString frame, double x, double y, double z, double w)
{
  geometry_msgs::PoseWithCovarianceStamped start;
  //设置frame
  start.header.frame_id=frame.toStdString();
  //设置时刻
  start.header.stamp=ros::Time::now();
  start.pose.pose.position.x=x;
  start.pose.pose.position.y=y;
  start.pose.pose.position.z=0;
  start.pose.pose.orientation.x = 0;
  start.pose.pose.orientation.y = 0;
  start.pose.pose.orientation.z=z;
  start.pose.pose.orientation.w=w;
  current_pub.publish(start);
  ros::spinOnce();
}

//发送返回点数据
void QNode::sendreturn_pos(double x,double y,double z,double w)
{
  geometry_msgs::Pose return_pos;
  return_pos.position.x = x;
  return_pos.position.y = y;
  return_pos.position.z = 0.0;
  return_pos.orientation.x = 0.0;
  return_pos.orientation.y = 0.0;
  return_pos.orientation.z = z;
  return_pos.orientation.w = w;
  returnpos_pub.publish(return_pos);
}

//处理客户端发过来的点集
void QNode::handlemultiposlist(QVector<QPolygonF> poslist)
{
  geometry_msgs::Pose pose;
  geometry_msgs::PoseArray pose_list;
  for (int i=0;i<poslist.count();i++)
  {
    QPointF startmultipos = poslist.at(i).at(0);
    QPointF lastmultipos = poslist.at(i).at(1);
    geometry_msgs::Quaternion return_pos_quat;
    double tan_radian = (lastmultipos.y()-startmultipos.y())/(lastmultipos.x()-startmultipos.x());
    double radian = atan(tan_radian);
    if(lastmultipos.x()>=startmultipos.x())
    {
      return_pos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastmultipos.x()<startmultipos.x()) &&  (lastmultipos.y()>=startmultipos.y()))
    {
      return_pos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastmultipos.x()<startmultipos.x()) &&  (lastmultipos.y()<startmultipos.y()))
    {
      return_pos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    pose.position.x = startmultipos.x();
    pose.position.y = startmultipos.y();
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = return_pos_quat.z;
    pose.orientation.w = return_pos_quat.w;
    pose_list.poses.push_back(pose);
  }
  multipos_pub.publish(pose_list);
}

void QNode::handle_multipos_list(geometry_msgs::PoseArray pose_list)
{
    multipos_pub.publish(pose_list);
}

void QNode::sendfloornum(int floor)
{
  floor_str_set = QString::number(floor);
  std_msgs::Int32 _data;
  _data.data = floor;
  qDebug() << "floornum_pub.publish: " << floor;
  floornum_pub.publish(_data);
  ros::spinOnce();
}

void QNode::handle_fanDisplay(uint32_t status)
{
    std_msgs::Int32 status_pub;
    status_pub.data = status;
    fanDisplay_pub.publish(status_pub);
}

//地图信息订阅回调函数
void QNode::mapCallback(nav_msgs::OccupancyGrid::ConstPtr map){
      mapWidth=map->info.width;
      mapHeight=map->info.height;
      m_mapOriginX=map->info.origin.position.x;
      m_mapOriginY=map->info.origin.position.y;
      m_mapResolution=map->info.resolution;
      QString map_str;
      map_str = "/home/smarcorobot/smarco_robot/src/turn_on_smarco_robot/map/smarco_F"+floor_str_set+".png";
      QImage m_map(map_str);
      qDebug() << map_str;
      emit updateMap(m_map);
      //计算map坐标系地图中心点坐标
      //scene(0,0) ^
      //           **********|************
      //           **********|************
      //           ----------o-map(0,0)---
      //           **********|************
      //           **********|************
      //origin(x,y)^
      //地图中心点map坐标系坐标
      m_mapCenterPoint.setX(m_mapOriginX+m_mapResolution*mapWidth*0.5);
      m_mapCenterPoint.setY(m_mapOriginY+m_mapResolution*mapHeight*0.5);
      //地图中心点图元坐标系坐标
      m_sceneCenterPoint.setX(mapWidth/2.0);
      m_sceneCenterPoint.setY(mapHeight/2.0);
      win_pointlist.clear();
      win_pointlist<<m_mapCenterPoint<<m_sceneCenterPoint;
      emit sendcenterpoint(win_pointlist);
}

void QNode::updateRobotPose(){
  try {
      tf::StampedTransform transform;
      m_robotPoselistener->lookupTransform(map_frame,base_frame,ros::Time(0), transform);

//      QVector<double> current_point_;
//      current_point_.push_back(transform.getOrigin().getX());
//      current_point_.push_back(transform.getOrigin().getY());
//      current_point_.push_back(transform.getRotation().getZ());
//      current_point_.push_back(transform.getRotation().getW());
//      emit current_point_signal_(transform.getOrigin().getX(),
//                                 transform.getOrigin().getY(),
//                                 transform.getRotation().getZ(),
//                                 transform.getRotation().getW());
      //current_point_.clear();

      tf::Quaternion q=transform.getRotation();
      double x=transform.getOrigin().getX();
      double y=transform.getOrigin().getY();
      tf::Matrix3x3 mat(q);
      double roll,pitch,yaw;
      mat.getRPY(roll,pitch,yaw);
      // qDebug() << "raw point: " << QPoint(x, y);
      emit updateRoboPose_cloud(QPointF(x,y),yaw);
      //坐标转化为图元坐标系
      QPointF roboPos=transMapPoint2Scene(QPointF(x,y));
      emit updateRoboPose(roboPos,yaw);
  } catch (tf::TransformException& ex) {
//     ROS_ERROR("Received an exception trying to updateRobotPose transform a point from \"map\" to \"base_link\": %s", ex.what());
  }
}


void QNode::addlinepointcallback(geometry_msgs::Polygon vector_point32)
{
   QPointF getpoint;
   QVector<QPointF> line_getpoint;
   QVector<QPointF> line_getpoint_aid;
   QVector<QVector<QPointF>> line_getline;
   int point_count=0;
   for (int i = 0;i<vector_point32.points.size();i++)
   {
     for (int j=0;j<2;j++)
     {
       if(point_count == 0)
       {
          getpoint.setX(vector_point32.points.at(i).x);
          point_count++;
       }
       else {
         getpoint.setY(vector_point32.points.at(i).y);
         point_count =0;
         line_getpoint.append(getpoint);
       }
     }
   }

   for(int k = 0;k<(line_getpoint.count()/2);k++)
   {
      line_getpoint_aid.clear();
      line_getpoint_aid<<line_getpoint.at(2*k)<<line_getpoint.at(2*k+1);
      line_getline<<line_getpoint_aid;
   }
   line_mytype.m_DstIdList = line_getline;
   emit sendaddlinepoint(line_mytype);
}

//处理返回点回调函数
void QNode::returnposcallback(geometry_msgs::Pose pos)
{
  //坐标转化为图元坐标系
  QPointF roboPos=transMapPoint2Scene(QPointF(pos.position.x,pos.position.y));
  //yaw
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pos.orientation, quat);
  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
//  double yaw;
//  yaw = tf::getYaw(pos.orientation);//另外一种转变欧拉角的方式
  emit sendreturnpos(roboPos,yaw);
  returnpostostartpos = pos;
    qDebug() << "roboPos: " <<roboPos;
  qDebug() << "yaw: " <<yaw;
  qDebug() << "start point: " << QPointF(pos.position.x,pos.position.y);
}

void QNode::slot_getsetreturnpossignal()
{
  double return_x = returnpostostartpos.position.x;
  double return_y = returnpostostartpos.position.y;
  double return_z = returnpostostartpos.orientation.z;
  double return_w = returnpostostartpos.orientation.w;
  set_start("map", return_x, return_y, return_z, return_w);
}

//巡航点回调函数
void QNode::multiposcallback(geometry_msgs::PoseArray poslist)
{
  poselistType posetype;
  int count = poslist.poses.size();
  for (int i=0;i<count;i++)
  {
    QVector<double> pose;
    pose<<poslist.poses.at(i).position.x<<poslist.poses.at(i).position.y<<poslist.poses.at(i).orientation.z<<poslist.poses.at(i).orientation.w;
    posetype.posearray.append(pose);
  }
  emit sendmultipos(posetype);
}


//速度回调函数
void QNode::speedCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    emit speed_x(msg->linear.x);
    emit speed_raw(msg->angular.z);
}
void QNode::run() {
        qDebug()<<"QNode:"<<QThread::currentThreadId();
        ros::Rate loop_rate(m_frameRate);
        ros::AsyncSpinner spinner(m_threadNum);
        spinner.start();
        //当当前节点没有关闭时
        while ( ros::ok() ) {
            updateRobotPose();
            loop_rate.sleep();
        }
        //如果当前节点关闭
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}
//发布机器人速度控制
 void QNode::move_base(char k,float speed_linear,float speed_trun)
 {
     std::map<char, std::vector<float>> moveBindings
     {
       {'i', {1, 0, 0, 0}},
       {'o', {1, 0, 0, -1}},
       {'j', {0, 0, 0, 1}},
       {'l', {0, 0, 0, -1}},
       {'u', {1, 0, 0, 1}},
       {',', {-1, 0, 0, 0}},
       {'.', {-1, 0, 0, 1}},
       {'m', {-1, 0, 0, -1}},
       {'O', {1, -1, 0, 0}},
       {'I', {1, 0, 0, 0}},
       {'J', {0, 1, 0, 0}},
       {'L', {0, -1, 0, 0}},
       {'U', {1, 1, 0, 0}},
       {'<', {-1, 0, 0, 0}},
       {'>', {-1, -1, 0, 0}},
       {'M', {-1, 1, 0, 0}},
       {'t', {0, 0, 1, 0}},
       {'b', {0, 0, -1, 0}},
       {'k', {0, 0, 0, 0}},
       {'K', {0, 0, 0, 0}}
     };
     char key=k;
     //计算是往哪个方向
     float x = moveBindings[key][0];
     float th = moveBindings[key][1];
     float z = moveBindings[key][2];
     float y = moveBindings[key][3];
     //计算线速度和角速度
     float speed = speed_linear;
     float turn = speed_trun;
	 qDebug()<<"speed::"<<speed;
	 qDebug()<<"turn::"<<turn;
     // Update the Twist message
     geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;
	qDebug()<<x<<y<<z<<th;
    // Publish it and resolve any remaining callbacks
    cmd_pub.publish(twist);
    ros::spinOnce();

 }
 //订阅图片话题，并在label上显示
 void QNode::Sub_Image(QString topic,int frame_id)
 {
   ros::NodeHandle n;
   image_transport::ImageTransport it_(n);
  switch (frame_id) {
      case 0:
//         image_sub0=it_.subscribe("/camera/rgb/image_raw",100,&QNode::imageCallback0,this,image_transport::TransportHints("compressed"));
      break;
      case 1:
//          image_sub1=it_.subscribe("/camera/rgb/image_raw",100,&QNode::imageCallback1,this,image_transport::TransportHints("compressed"));
       break;
      case 2:
//          image_sub2=it_.subscribe("/camera/rgb/image_raw",100,&QNode::imageCallback2,this,image_transport::TransportHints("compressed"));
       break;
      case 3:
//          image_sub3=it_.subscribe("/camera/rgb/image_raw",100,&QNode::imageCallback3,this,image_transport::TransportHints("compressed"));
       break;
  }
  ros::spinOnce();
 }

 //图元坐标系转换为map坐标系
QPointF QNode::transScenePoint2Map(QPointF pos){

  QPointF roboPos;
  roboPos.setX((pos.x()-m_sceneCenterPoint.x())*m_mapResolution+m_mapCenterPoint.x());
  roboPos.setY(-1*(pos.y()-m_sceneCenterPoint.y())*m_mapResolution+m_mapCenterPoint.y());
  return roboPos;
}

//map坐标系转换为图元坐标系
QPointF QNode::transMapPoint2Scene(QPointF pos){
  QPointF roboPos;
  double point_x,point_y;
  point_x = (pos.x()-m_mapCenterPoint.x())/m_mapResolution+m_sceneCenterPoint.x();
  point_y = -1*(pos.y()-m_mapCenterPoint.y())/m_mapResolution+m_sceneCenterPoint.y();
  if((!std::isinf(point_x))&&(!std::isnan(point_x))&&(!std::isinf(point_y))&&(!std::isnan(point_y)))
  {
    roboPos.setX(point_x);
    roboPos.setY(point_y);
  //  roboPos.setX((pos.x()-m_mapCenterPoint.x())/m_mapResolution+m_sceneCenterPoint.x());
  //  roboPos.setY(-1*(pos.y()-m_mapCenterPoint.y())/m_mapResolution+m_sceneCenterPoint.y());
    return roboPos;
  }
}
double QNode::getRealTheta(QPointF start,QPointF end){
  double y=end.y()-start.y();
  double x=end.x()-start.x();
  double theta=radiansToDegrees(atan(y/x));
  // 1 4
  if(end.x()>start.x()){

     // 1
    if(end.y()>start.y()){
      theta = -theta;
    }
    // 4
    else {
       theta = 270 - theta;
    }
  }else {
  // 2 3
     theta = 180- theta;
//    if(end.y()>start.y()){
//      //2
//      theta = 180- theta;
//    }
//    else {

//    }
  }
  return theta;
}
void QNode::pub_imageMap(QImage map){
//     cv::Mat image = QImage2Mat(map);
//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     m_imageMapPub.publish(msg);
}
 //沿x轴翻转地图
 cv::Mat QNode::RotaMap(cv::Mat const& map){
    cv::Mat result;
    result.create(map.size(),map.type());
    int height = map.rows;
    int width = map.cols;
    for(int i=0; i< height; i++){
      for (int j=0;j< width;j++) {
        result.at<uchar>(height-i-1,j) = map.at<uchar>(i,j);
      }
    }
    return result;
 }

void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::send_elec_line(geometry_msgs::Polygon point_polygen)
{
    electronicfenceclient.publish(point_polygen);
    qDebug() << "publish elec line success.";
}

void QNode::handle_pub_global_path(nav_msgs::Path record_global_path)
{
    std_msgs::Int32 x_;
    x_.data = 10;
    record_global_path_pub.publish(record_global_path);
    pub_test_.publish(x_);
    ros::spinOnce();
}
}  // namespace class1_ros_qt_demo
