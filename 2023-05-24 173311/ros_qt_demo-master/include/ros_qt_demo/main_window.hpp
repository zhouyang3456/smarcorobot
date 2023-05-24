/**
 * @file /include/class1_ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for class1_ros_qt_demo.
 *
 * @date November 2010
 **/
#ifndef class1_ros_qt_demo_MAIN_WINDOW_H
#define class1_ros_qt_demo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "server.h"
#include "client_thread.h"
#include <QtMultimedia/QMediaPlayer>
#include <QtMultimedia/QMediaPlaylist>
#include <QUdpSocket>
#include <QThread>
#include <QFileDialog>
#include "workthread.hpp"
#include <QMutexLocker>
#include <QMutex>
#include "thread_http.hpp"
#include "QUrl"
#include "thread_trackpath.h"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace class1_ros_qt_demo {
// #define __SMARCOROBOT__
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  void qnode_connections();//信号槽连接集合

  void car_cloud_connect();

  //发送描述符+命令+数据
  void sendscok_com_data(int sockDesc, uint8_t int_commend, QByteArray mdata);

  //app相关
  void app_sendscok_com_data(int sockDesc, uint8_t int_commend,uint32_t robo_ID, QByteArray mdata);

  void sendcommendwithdata(uint8_t int_commend, uint32_t robo_ID, QByteArray mdata);//发送命令+数据

  //重新编写通讯协议
  void handledata(int sockDesc, uint8_t commend, QByteArray r_data);
  void cloud_handledata(uint8_t commend,uint32_t roboid, QByteArray r_data);

  void app_handledata(int sockDesc, uint8_t commend, QByteArray r_data);

  //处理发送过来的设置初始位置的点位
  void hanglecurrent_pos(QByteArray r_data);
  //处理发送过来的设置目标位置的点位
  void hanglegoal_pos(QByteArray r_data);
  //处理发送过来的客户端操作状态
  void hangleclientstatue(QByteArray r_data);
  //处理发送过来的设置返航点位置的点位
  void hanglereturn_pos(QByteArray r_data);
  //处理发送过来的设置巡航点位置的点位
  void hanglemulti_pos(QByteArray r_data);
  //处理发送过来的电子围栏数据
  void hangleaddlinedata(QByteArray r_data);


  //处理餐车发送过来的状态
  void handle_food_carstatue(int sockDesc,QByteArray r_data);

  //处理发送过来的是否自动返航的标志
  void handle_autobackhomestatue(QByteArray r_data);

  //处理发送过来的地图
  void handle_changemapdata(QByteArray r_data);

  //处理发送过来的楼层
  void handle_floornum(QByteArray r_data);

  //处理发送过来的车辆操控状态
  void handle_carrunstatue(QByteArray r_data);

  //李建鹏
  // app
  QPolygonF rdatatopolygen(QByteArray r_data);
  void handle_goalpos(QByteArray r_data);
  
  void handle_expressstatue(int sockDesc,QByteArray r_data);
  void handle_openexpress(QByteArray r_data);

  void handle_openexpressvoice(QByteArray r_data);

  void handle_udpdata(QByteArray r_data);

  void handle_appchangemap(QByteArray r_data);
  
  void handle_appinfoinit(int sockDesc,QByteArray r_data);

  // handle cloud data
  // QPolygonF rdatatopolygen(QByteArray r_data);
  //处理发送过来的设置初始位置的点位
  void handle_current_pos(QByteArray r_data);
  //处理发送过来的设置目标位置的点位
  void handle_goal_pos(QByteArray r_data);
  //处理发送过来的客户端操作状态
  void handle_clientstatue(QByteArray r_data);
  //处理发送过来的设置返航点位置的点位
  void handle_return_pos(QByteArray r_data);
  //处理发送过来的设置巡航点位置的点位
  void handle_multi_pos(QByteArray r_data);
  //处理发送过来的电子围栏数据
  void handle_addlinedata(QByteArray r_data);
  // fan display
  void handle_switch_fan_display(QByteArray r_data);
  // send current floor
  void handle_send_current_floor(QByteArray r_data);
  // void handle_get_total_floor_num()
  void handle_play_audio(QByteArray r_data);
  void handle_receive_new_audio(QByteArray r_data);

  void handle_receive_new_video(QByteArray r_data);

  void handle_play_video(QByteArray r_data);

  void open_play_video_thread();
  void close_play_video_thread();

  QByteArray change_int_to_byteArray(int num);

  void handle_multi_navi_play_video_switch(QByteArray r_data);

  void handle_volice_quiet(QByteArray r_data);

  void handle_record_path(QByteArray r_data);

  void handle_points_of_path();

  void play_voice(QString voice_file_name);

  void handle_track_path(QByteArray r_data);


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
signals:
    void sendData(int id, const QByteArray &data);
    void sendqimageData_signal(int id,QByteArray data);
    void sendsetreturnpossignal();

    //app相关
    void app_sendData(int id, const QByteArray &data);

    void signal_play();
    void signal_stop();

    void current_pose_(double, double, double, double);

protected slots:

public slots:
    void recvData(QString ip, int port, int sockDesc, QByteArray datagram);
    void slot_recesockDesc(QList<int> socklist);
    void slot_batteryState(float p);
    void slot_getmap(QImage map);//获取地图信息
    void slot_receivewinpointlist(QPolygonF pointlist);//获取地图和场景的中心点
    void slot_updateRoboPose(QPointF point, float raw);//获取小车当前位置
    void slot_updateRoboPose_cloud(QPointF point, float raw);//获取小车当前位置
	
    void slot_updatalaserscan(QPolygonF laserscan);//获取雷达数据
    void paintPlannerPath(QPolygonF planner);//获取雷达数据
    void slot_getcarmovestatue(int statue);//获取小车运行状态
    void slot_get_image(int frame_id,QImage image);//获取摄像头数据
    void slot_updatereturnpos(QPointF point, double raw);//获取返回点数据
    void slot_updatemultipos(poselistType poslist);//获取巡航点数据
    void slot_updateaddlinepos(myType poslist);//获取电子围栏数据


    void slot_connected();
    void slot_disconnected();
    void slot_receivedata(QByteArray datagram);
    void slot_geterror(QAbstractSocket::SocketError error);

    void slot_flash();//定时发送心跳包
    void slot_loginflash();

    void slot_flashconnectTohost();
    void slot_carstatue(int statue);
    void slot_carerrorstatue(int errorstatue);
    void slot_floorcar(int floor);

    void slot_autobacktimer();
	
	void slot_setreturntimer();
	void slot_setstarttimer();

    void slot_getnagstatue(int nagstatue);

    //app相关
    void app_recvData(QString ip, int port, int sockDesc, QByteArray datagram);
    void app_slot_recesockDesc(QList<int> socklist);

    void getexpressstatue(statuetype expressstatue);
    
    //UDP
    void dataReceived();

    void slot_send_battery();
    void slot_send_floor_num();
    void slot_send_current_car_point();
    void slot_send_robot_running_status();

    void handle_timeout_send_multi_navi_srart();
    void handle_timeout_send_navi_point();
    void handle_timeout_send_car_return();
    void handle_timeout_reconnect();
    void handle_timeout_send_fault();

    void handle_send_laser_fault_signal();
    void handle_send_imu_fault_signal();

    void slot_get_car_move_status(int status);

    void handle_reachPointNum_signal(qint32 point_num);
    void handle_currentPoint_to_returnPoint();
    // void handle_current_point_signal_(QVector<double> current_point_);
    void handle_current_point_signal_(double, double, double, double);

    void handle_set_goal_(QString frame,double x,double y,double z,double w);

    void handle_pub_global_path();

private slots:
    void showConnection(int sockDesc);
    void showDisconnection(int sockDesc);

    void on_check_statue_btu_clicked();

    void on_connect_cloud_clicked();

    //app相关
    void app_showConnection(int sockDesc);
    void app_showDisconnection(int sockDesc);

    void on_pushButton_play_music_clicked();

    void on_pushButton_stop_music_clicked();

    void on_pushButton_stop_video_clicked();

    void on_pushButton_quit_video_clicked();

    void on_pushButton_set_video_url_clicked();


    void slot_timer_test_mp4_play();


    // void on_pushButton_clicked();

    void on_pushButton_test_switch_mp4_play_stop_clicked();

    void on_pushButton_test_switch_mp4_play_clicked();

    //void on_pushButton_2_clicked();

    void startRequest(const QUrl &requestedUrl);
    void replyFinished();
    void reply_readyRead();
    void reply_downloadProgress(qint64 ,qint64);
    //void on_pushButton_clicked();

    void on_pushButton_pub_path_clicked();

public:
	Ui::MainWindowDesign ui;
	QNode qnode;
  Server *m_server;
  Server *app_server;
  Server *voice_server;
  uint16_t modbus_crc16(QByteArray pucFrame);
  QProcess *proc;
  QString savemap;
  QList<int> m_socklist;
  QList<int> app_socklist;
  //电池电压
  QString battery_str = "0";
  //底层连接状态标志位,防止多次接收连接信号
  bool connect_statue = false;
  //接收线速度，角速度，全向轮状态
  float raw_speed = 20;
  float line_speed = 20;
  int use_all_statue = 0;
  //地图
  QByteArray map_ba;
  QPolygonF center_point;
  QByteArray center_ba;
  //当前位置
  double RoboPose_x = 0.0;
  double RoboPose_y = 0.0;
  double RoboPose_raw = 0.0;
  QByteArray RoboPose_data;

  double RoboPose_x_cloud = 0.0;
  double RoboPose_y_cloud = 0.0;
  double RoboPose_raw_cloud = 0.0;
  QByteArray RoboPose_data_cloud;
  //重新编写通讯协议
  uint8_t m_commend = 0xFF;//收到协议中的命令,0XFF为初始状态，命令为空
  uint32_t m_count = 0;//一条数据的长度，数据位3-7位
  bool datahandle_statue = false;//处理数据状态机，false表示没有未处理完的数据，true表示上一帧数据还有没有处理完全的数据
  QByteArray datahandle;//解析出来的数据，用完需clear
  QByteArray data_sur;
  //雷达数据
  QByteArray laserscan_ba;
  //路径数据
  QByteArray planner_ba;
  //车和客户端操作状态
  int car_statue = 3;//2:动作被打断 3：小车动作执行完毕处于待命状态 10:小车处于运行状态
  int client_statue = 0;//0:用户处于空闲状态 1：客户端发出需要执行单点导航 2：客户端发出需要执行返航 3:客户端发出需要巡航
  //摄像头数据
//  QByteArray image0_ba = nullptr;
  QImage image0;
  //返回点数据。1.客户端设置2.车端读取
  double return_posx = 0.0;
  double return_posy = 0.0;
  double return_rawz = 0.0;
  double return_raww = 0.0;
  QVector<double> Robo_doubledata;//小车数据返回上来以后进行封装
  double returnpos_x = 0.0;
  double returnpos_y = 0.0;
  double returnpos_raw = 0.0;
  QByteArray returnpos_ba;
  //电子围栏
  QByteArray addlinelist_ba;
  //作客户端发送数据
  Client_thread *car_client;
  QThread *mythread = nullptr;
  QTimer *mtimer = nullptr;//定时发送心跳包
  QTimer *login_timer = nullptr;//发送登陆信息
  QByteArray empty_ba;
  QString Ip_str;
  int port_str;
  QTimer *socket_timer = nullptr;//发生错误后定时连接服务器
  int connect_count = 0;//连接服务器次数



  uint8_t cloud_m_commend = 0xFF;//收到协议中的命令,0XFF为初始状态，命令为空
  uint32_t cloud_m_count = 0;//一条数据的长度，数据位3-7位
  bool cloud_datahandle_statue = false;//处理数据状态机，false表示没有未处理完的数据，true表示上一帧数据还有没有处理完全的数据
  QByteArray cloud_datahandle;//解析出来的数据，用完需clear
  QByteArray cloud_data_sur;
  uint32_t robot_ID = 230412061;
  uint32_t smarcorobot_ID = 230412061;

  // uint32_t robot_ID = 230419064;
  // uint32_t smarcorobot_ID = 230419064;

  uint8_t currency_m_commend = 0xFF;//收到协议中的命令,0XFF为初始状态，命令为空
  uint32_t currency_m_count = 0;//一条数据的长度，数据位3-7位
  bool currency_datahandle_statue = false;//处理数据状态机，false表示没有未处理完的数据，true表示上一帧数据还有没有处理完全的数据
  QByteArray currency_datahandle;//解析出来的数据，用完需clear
  QByteArray currency_data_sur;



  //送餐车状态
  int food_carstatue = 3;//收到状态改为2后立刻将小车状态变为待命状态
  int food_clientstatue = 0;//（0，3）小车处于待命状态；（1，0）小车处于送餐状态；（1，3）小车处于送餐到达待取餐；（2，0）小车处于返航状态；（2，3）->（0，3）小车转为待命状态；
                            //(3,0)小车处于巡航状态
  //是否自动返航状态
  int autoback_statue=0;
  bool goal_statue = false;
  QTimer *autoback_timer;
  int timercount = 0;//定时器计数器

  //处理发送过来的地图
  QImage n_image;
  
  bool setstartstatue = false;
  
  QPointF Robo_pose;
  
  bool robo_pose_statue = true;

  //上位机发送过来的楼层
  QString floor_num = "3";
  QString total_floor = "10";

  //送餐车状态
  int jinyin_carstatue = 3;//收到状态改为2后立刻将小车状态变为待命状态
  int jinyin_clientstatue = 0;//（0，3）小车处于待命状态；（1，0）小车处于送餐状态；（1，3）小车处于送餐到达待取餐；（2，0）小车处于返航状态；（2，3）->（0，3）小车转为待命状态；
                            //(3,0)小车处于巡航状态

    QMap<int,int> express_statue;

    QMediaPlayer *player;
    QMediaPlaylist *playlist;
    QMediaPlayer *music_player;

    int port;
    QUdpSocket *udpSocket;

   //apprunstatue
   int app_carrunstatue = 3;
   bool app_initstatue = true;//初始化快递柜标准位
   bool changemap_statue = true;
   bool power_statue = true;
   QByteArray floor_ba;
   
   QTimer *setreturn_timer;
   QTimer *app_setstart_timer;

   bool fan_display_status = false;
   QByteArray current_car_floor;

   QTimer *timer_send_start_muti_navigation_signal;
   QTimer *timer_send_navi_point_signal;
   QTimer *timer_send_car_return_signal;

   QByteArray car_goal_pos_;

   workThread *thread_play_video;

   thread_http *thread_http_transmit;

   QTimer *timer_reconnect_ = nullptr;
   QTimer *timer_send_battery = nullptr;
   QTimer *timer_send_floor_num = nullptr;
   QTimer *timer_send_current_car_point = nullptr;
   QTimer *timer_send_robot_running_status = nullptr;
   QTimer *timer_send_fault = nullptr;

   QTimer *timer_test_mp4_play = nullptr;

   char fault_type = 0x00;
   int laser_fault_count = 0;
   int imu_fault_count = 0;

   int multi_navi_loop_count = 0;
   // 0: stop
   // 1:single point
   // 2:return point
   // 3:multi navigation
   int current_car_status_ = 0;

   QMutex * m_mutex;

   int multi_navi_play_video_flag = 0;
   QVector<double> car_current_point_;

   // [3] 添加对象
   QUrl url;
   QNetworkRequest req;
   QNetworkReply *reply;
   QNetworkAccessManager *manager;
   QFile *downFile;

   QString file_name_;
   QString file_path_;

#ifdef __SMARCOROBOT__
   std::string  yaml_file_path = "/home/smarcorobot/catkin_zy/src/ros_qt_demo-master/config.yaml";
   std::string  yaml_global_path = "/home/smarcorobot/catkin_zy/src/ros_qt_demo-master/global_path.yaml";
#else
   std::string  yaml_file_path = "/home/zhou/catkin_qt/src/ros_qt_demo-master/config.yaml";
   std::string  yaml_global_path = "/home/zhou/catkin_qt/src/ros_qt_demo-master/global_path.yaml";
#endif

   int set_return_flag = 0;

   // int video_download_flag = 0;
   int video_download_start_flag = 0;

   QString prompt_voice_path;

   QPolygonF points_of_path;

   int record_path_flag = 0;

   const double interval_of_path_points = 0.025; // 2.5cm

   thread_trackPath* my_thread_trackPath = nullptr;
};

const uint8_t Connect_signal = 0x00;
const uint8_t battery_signal = 0x01;
const uint8_t robotstatue_signal = 0x02;
const uint8_t left_up_signal = 0x10;
const uint8_t up_signal = 0x11;
const uint8_t right_up_signal = 0x12;
const uint8_t left_signal = 0x13;
const uint8_t stopmove_signal = 0x14;
const uint8_t right_signal = 0x15;
const uint8_t left_down_signal = 0x16;
const uint8_t down_signal = 0x17;
const uint8_t right_down_signal = 0x18;
const uint8_t checkBox_use_all_signal = 0x19;
const uint8_t raw_signal = 0x1A;
const uint8_t line_signal = 0x1B;
const uint8_t map_signal = 0x03;//请求地图数据
const uint8_t centerpointlist_signal = 0x04;
const uint8_t robopos_signal = 0x05;
const uint8_t laserscan_signal = 0x06;
const uint8_t current_pos_signal = 0x07;
const uint8_t goal_pos_signal = 0x08;
const uint8_t planner_signal = 0x09;
const uint8_t carstatue_signal = 0x0A;
const uint8_t clientstatue_signal = 0x0B;
const uint8_t image0_signal = 0x0C;
const uint8_t return_pos_signal = 0x0D;
const uint8_t startreturn_signal = 0x0E;
const uint8_t bendreturn_signal = 0x0F;
const uint8_t multipos_signal = 0x20;
const uint8_t startmulti_signal = 0x21;
const uint8_t pausemulti_signal = 0x22;
const uint8_t stopcar_signal = 0x23;
const uint8_t addline_signal = 0x24;
const uint8_t bendaddline_signal = 0x25;
const uint8_t bendmultipos_signal = 0x26;
const uint8_t cartoserverstatue_signal = 0x27;
const uint8_t cartoservererrorstatue_signal = 0x28;
const uint8_t floorcar_signal = 0x29;
const uint8_t food_carstatue_signal = 0x2A;
const uint8_t food_clientstatue_signal = 0x2B;
const uint8_t foodstatue_signal = 0x2C;
const uint8_t autobackhome_signal = 0x2D;
const uint8_t changemap_signal = 0x2E;
const uint8_t pulsecar_signal = 0x2F;
const uint8_t floornum_signal = 0x30;
const uint8_t carrunstatue_signal = 0x31;
const uint8_t jinyin_statue_signal = 0x32;
const uint8_t open_express_signal = 0x33;
const uint8_t express_statue_signal = 0x34;
const uint8_t openexpress_statue_signal = 0x35;
const uint8_t app_changemap_signal = 0x36;
const uint8_t app_carrunstatue_signal = 0x37;
const uint8_t app_infoinit_signal = 0x38;
const uint8_t app_setreturn_signal = 0x39;

const uint8_t connect_to_cloud_plat = 0x40;
const uint8_t switch_fan_display = 0x41;
const uint8_t get_total_floor_num = 0x42;
const uint8_t receive_new_audio = 0x43;
const uint8_t play_audio_signal = 0x44;
const uint8_t car_login_success_signal = 0x45;
const uint8_t play_video_signal = 0x46;
const uint8_t send_car_fault = 0x47;
const uint8_t add_elec_line = 0x48;
const uint8_t set_currentPoint_to_returnPoint = 0x49;
const uint8_t multi_navi_play_video_switch = 0x50;
const uint8_t receive_new_video = 0x52;
const uint8_t record_path = 0x53;
const uint8_t start_track_path = 0x55;



// bu yong guan
const uint8_t fault_laser = 0x01;
const uint8_t fault_imu = 0x02;



static const unsigned char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const unsigned char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};


}  // namespace class1_ros_qt_demo

#endif // class1_ros_qt_demo_MAIN_WINDOW_H
