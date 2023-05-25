/**
 * @file /include/class1_ros_qt_demo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef class1_ros_qt_demo_QNODE_HPP_
#define class1_ros_qt_demo_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <QtConcurrent/QtConcurrent>
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
#include <QDebug>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/
//注册自定义信号
typedef struct
{
    QVector<QVector<QPointF>> m_DstIdList;
}myType;

typedef struct
{
    QVector<QVector<double>> posearray;
}poselistType;

typedef struct
{
    QVector<int> statue_type;
}statuetype;


class QNode : public QThread {
    Q_OBJECT
  //角度转弧度
  #define degreesToradian(x) (M_PI*x/180.0)

  //弧度转角度
  #define radiansToDegrees(x) (180.0*x/M_PI)
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void move_base(char k,float speed_linear,float speed_trun);

  void set_goal(QString frame,double x,double y,double z,double w);
  void set_start(QString frame,double x,double y,double z,double w);
  void set_datarequest(int data);//发送数据请求类型
  void addlineset_datarequest(int data);//发送数据请求类型
  void open_express(int data);//发送快递柜开柜
  void check_express(int data);//发送快递柜开柜
  
  
  void sendline_modify(QVector<QPolygonF> point_list);//发布点位
  void Sub_Image(QString topic,int frame_id);
  void pub_imageMap(QImage map);
  double getRealTheta(QPointF start,QPointF end);
  QPointF transScenePoint2Map(QPointF pos);
  QPointF transMapPoint2Scene(QPointF pos);
  QMap<QString,QString> get_topic_list();
  //处理客户端发过来的点集
  void handlemultiposlist(QVector<QPolygonF> poslist);
  void handle_multipos_list(geometry_msgs::PoseArray pose_list);

  int mapWidth{0};
  int mapHeight{0};
  void run();

  void cancel_action();//取消动作

  void sendreturn_pos(double x, double y, double z, double w);

  void sendfloornum(int floor);

  void handle_fanDisplay(uint32_t status);

  /*********************
  ** Logging
  **********************/
  enum LogLevel {
             Debug,
           Info,
           Warn,
           Error,
           Fatal
   };

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg);

  void send_elec_line(geometry_msgs::Polygon point_polygen);

  // transform map to base_link
  // tf::StampedTransform transform;

public slots:
  void slot_getsetreturnpossignal();

Q_SIGNALS:
  void loggingUpdated();
    void rosShutdown();
    void speed_x(double x);
    void speed_raw(double y);
    void batteryState(float p);
    void Master_shutdown();
    void Show_image(int,QImage);
    void updateRoboPose(QPointF pos,float yaw);
    void updateRoboPose_cloud(QPointF pos,float yaw);
    void updateMap(QImage map);
    void plannerPath(QPolygonF path);
    void updateLaserScan(QPolygonF points);
    void updatecarmovestatue(int statue);
    //添加电子围栏相关
    void sendaddlinepoint(myType line_getline);
    //win通讯相关
    void sendcenterpoint(QPolygonF pointlist);
    void sendreturnpos(QPointF pos,double raw);
    //发送添加导航点数据
    void sendmultipos(poselistType poslist);

    void sendcarstatue(int statue);
    void sendcarerrorstatue(int statue);
    void sendfloorcar(int floorcar);

    void sendnagstatue(int nagstatue);
    void sendexpressstatue(statuetype expressstatue);//快递柜状态发送

    void send_laser_fault_signal();
    void send_imu_fault_signal();

    void car_achieve_goal_point_signal(int statue);

    void reachPointNum_signal(qint32);

    // void current_point_signal_(QVector<double>);
    void current_point_signal_(double, double, double, double);
	
private:
  int init_argc;
  char** init_argv;
  ros::Publisher chatter_publisher;
  ros::Subscriber m_imu_sub;

    ros::Subscriber cmdVel_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber pos_sub;
    ros::Subscriber m_laserSub;
    ros::Subscriber battery_sub;
    ros::Subscriber m_plannerPathSub;
    ros::Subscriber m_compressedImgSub0;
    ros::Subscriber m_compressedImgSub1;
    ros::Publisher goal_pub;
    tf::TransformListener *m_robotPoselistener;
    tf::TransformListener *m_Laserlistener;
    std::string base_frame,laser_frame,map_frame;
    ros::Publisher current_pub;//当前位置
    ros::Publisher cancle_pub_;//取消当前动作

    //添加电子围栏相关
    ros::Publisher datarequest;//数据请求位
    ros::Publisher addlinedatarequest;//数据请求位
    ros::Subscriber electronicfencemaster;//订阅点位数据
    ros::Publisher electronicfenceclient; //发布点位数据

    ros::Subscriber statue_car;//订阅车辆当前状态
    ros::Subscriber statue_error_car;//订阅车辆错误状态
    ros::Subscriber floor_car;//订阅车辆当前所在楼层

    ros::Subscriber nag_statue;//订阅车辆当前状态
	
	ros::Publisher express_pub;//快递柜开锁
	ros::Publisher check_express_pub;//快递柜开锁
	ros::Subscriber express_statue;//订阅车辆当前状态

     ros::Publisher fanDisplay_pub;

     ros::Subscriber which_multi_point_achieve_sub;



    //切换楼层
    ros::Publisher floornum_pub;

    //返回点设置话题
    ros::Publisher returnpos_pub;
    ros::Subscriber returnpos_sub;

    //巡航点设置话题
    ros::Publisher multipos_pub;
    ros::Subscriber multipos_sub;

    ros::Publisher cmd_pub;
    image_transport::Publisher m_imageMapPub;
    QStringListModel logging_model;
    QString show_mode="control";
//    //图像订阅
//    image_transport::Subscriber image_sub0;
    //图像订阅
    image_transport::Subscriber image_sub0;
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;
    //图像format
    QString video0_format;
    QString video1_format;
    QString video2_format;
    QString video3_format;
    //地图订阅
    ros::Subscriber map_sub;
    ros::Subscriber reached_signal;
    //图像format
//    QString video0_format;
    QString odom_topic;
    QString batteryState_topic;
    QString pose_topic;
    QString laser_topic;
    QPolygon mapPonits;
    QPolygonF plannerPoints;
    QPolygonF laserPoints;
    int m_threadNum=4;
    int m_frameRate=40;
    QString map_topic;

    //地图 0 0点坐标对应世界坐标系的坐标
    float m_mapOriginX;
    float m_mapOriginY;
    //地图坐标系中心点坐标
    QPointF m_mapCenterPoint;
    //图元坐标系中心点坐标
    QPointF m_sceneCenterPoint;
    //地图一个像素对应真实世界的距离
    float m_mapResolution;
    //地图是否被初始化
    bool m_bMapIsInit=false;
    //tf::TransformListener m_tfListener(ros::Duration(10));
    //ros::Timer m_rosTimer;
    QImage Mat2QImage(cv::Mat const& src);
    cv::Mat QImage2Mat(QImage &image);
    cv::Mat RotaMap(cv::Mat const& map);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos);
    void speedCallback(const geometry_msgs::Twist::ConstPtr& msg);
//    void batteryCallback(const sensor_msgs::BatteryState &message);
    void batteryCallback(const std_msgs::Float32 &message);
    void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
    void myCallback(const std_msgs::Float64& message_holder);
    void mapCallback(nav_msgs::OccupancyGrid::ConstPtr map);
    void laserScanCallback(sensor_msgs::LaserScanConstPtr scan);
    void transformPoint(const tf::TransformListener& listener);
    void plannerPathCallback(nav_msgs::Path::ConstPtr path);
    void SubAndPubTopic();
    void statuecallback(move_base_msgs::MoveBaseActionResult result);
    int statue_result;
    void updateRobotPose();
    void statue_car_callback(const std_msgs::Int32 &msg);
    void statue_error_car_callback(const std_msgs::Int32 &msg);
    void floor_car_callback(const std_msgs::Int32 &msg);

    void nag_statue_callback(const std_msgs::Int32 &msg);
	
	void express_statue_callback(const std_msgs::UInt8MultiArray &msg);

    void imu_Callback(sensor_msgs::ImuConstPtr msg);

    void ReachPointNum_Callback(const std_msgs::Int32 &point_num);
	
//添加电子围栏相关
    void addlinepointcallback(geometry_msgs::Polygon vector_point);
    myType line_mytype;//定义自定义消息类型
    //win通讯相关
    QPolygonF win_pointlist;
    //返回点回调函数
    void returnposcallback(geometry_msgs::Pose pos);
    //巡航点回调函数
    void multiposcallback(geometry_msgs::PoseArray poslist);

    geometry_msgs::Pose returnpostostartpos;
    QString floor_str_set = "3";

};

}  // namespace class1_ros_qt_demo

#endif /* class1_ros_qt_demo_QNODE_HPP_ */
