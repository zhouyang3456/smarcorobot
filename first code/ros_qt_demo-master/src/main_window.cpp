/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ros_qt_demo/main_window.hpp"
#include <QObject>
#include "../include/ros_qt_demo/workthread.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    ReadSettings();
    //	setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    ui.connect_statue->setText("断开");
    /*********************
    ** Logging
    **********************/
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    //    if ( ui.checkbox_remember_settings->isChecked() ) {
    //        on_button_connect_clicked(true);
    //    }
    //data
    m_server = new Server(this);
    m_server->listen(QHostAddress::Any, 12345);
    connect(m_server,SIGNAL(dataReady(QString,int,int,QByteArray)),this,SLOT(recvData(QString,int,int,QByteArray)));
    connect(m_server,SIGNAL(sendshowconnection(int)),this,SLOT(showConnection(int)));
    connect(m_server,SIGNAL(senddisconnection(int)),this,SLOT(showDisconnection(int)));
    connect(m_server,SIGNAL(sendsockDesc(QList<int>)),this,SLOT(slot_recesockDesc(QList<int>)));
    connect(this,SIGNAL(sendData(int,QByteArray)),m_server,SLOT(rece_date(int,QByteArray)));


    app_server = new Server(this);
    app_server->listen(QHostAddress::Any, 12346);
    connect(app_server,SIGNAL(dataReady(QString,int,int,QByteArray)),this,SLOT(app_recvData(QString,int,int,QByteArray)));
    connect(app_server,SIGNAL(sendshowconnection(int)),this,SLOT(app_showConnection(int)));
    connect(app_server,SIGNAL(senddisconnection(int)),this,SLOT(app_showDisconnection(int)));
    connect(app_server,SIGNAL(sendsockDesc(QList<int>)),this,SLOT(app_slot_recesockDesc(QList<int>)));
    connect(this,SIGNAL(app_sendData(int,QByteArray)),app_server,SLOT(rece_date(int,QByteArray)));

    voice_server = new Server(this);
    voice_server->listen(QHostAddress::Any, 6666);
    connect(voice_server,SIGNAL(dataReady(QString,int,int,QByteArray)),this,SLOT(app_recvData(QString,int,int,QByteArray)));
    connect(voice_server,SIGNAL(sendshowconnection(int)),this,SLOT(app_showConnection(int)));
    connect(voice_server,SIGNAL(senddisconnection(int)),this,SLOT(app_showDisconnection(int)));
    connect(voice_server,SIGNAL(sendsockDesc(QList<int>)),this,SLOT(app_slot_recesockDesc(QList<int>)));
    // connect(this,SIGNAL(app_sendData(int,QByteArray)),voice_server,SLOT(rece_date(int,QByteArray)));


    port =36990;                             //设置UDP的端口号参数，指定在此端口上监听数据
    udpSocket = new QUdpSocket(this);		//创建一个QUdpSocket
    connect(udpSocket,SIGNAL(readyRead()),this,SLOT(dataReceived()));
    bool result=udpSocket->bind(QHostAddress("127.0.0.1"),port);		//绑定到指定的端口上
    if(!result)
    {
        qDebug()<<"UDP error!";
    }


    //连接节点信号槽
    qnode_connections();
    on_button_connect_clicked(false);//自动连接master
    empty_ba.clear();

    //是否控制自动返航定时器
    autoback_timer = new QTimer(this);
    connect(autoback_timer,SIGNAL(timeout()),this,SLOT(slot_autobacktimer()));

    setreturn_timer = new QTimer(this);
    connect(setreturn_timer,SIGNAL(timeout()),this,SLOT(slot_setreturntimer()));

    app_setstart_timer = new QTimer(this);
    connect(app_setstart_timer,SIGNAL(timeout()),this,SLOT(slot_setstarttimer()));

    player = new QMediaPlayer(this);
    player->setVolume(100);

    music_player = new QMediaPlayer(this);
    music_player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/06_music.mp3"));
    music_player->setVolume(70);

    thread_play_video = new workThread;

    //快递柜状态赋初值
    for(int i=1;i<11;i++)
    {
        express_statue.insert(i,0);
    }


    timer_test_mp4_play = new QTimer(this);
    connect(timer_test_mp4_play,SIGNAL(timeout()),this,SLOT(slot_timer_test_mp4_play()));


    // connect to cloud
    car_cloud_connect();
}

void MainWindow::qnode_connections()
{
    //电源的信号
    connect(&qnode,SIGNAL(batteryState(float)),this,SLOT(slot_batteryState(float)));
    //地图信息
    connect(&qnode,SIGNAL(updateMap(QImage)),this,SLOT(slot_getmap(QImage)));
    //获取地图和场景中心点的坐标
    connect(&qnode,SIGNAL(sendcenterpoint(QPolygonF)),this,SLOT(slot_receivewinpointlist(QPolygonF)));
    //获取小车当前位置
    connect(&qnode,SIGNAL(updateRoboPose(QPointF,float)),this,SLOT(slot_updateRoboPose(QPointF,float)));
    connect(&qnode,SIGNAL(updateRoboPose_cloud(QPointF,float)),this,SLOT(slot_updateRoboPose_cloud(QPointF,float)));
    //获取雷达数据
    connect(&qnode,SIGNAL(updateLaserScan(QPolygonF)),this,SLOT(slot_updatalaserscan(QPolygonF)));
    //获取路径信息数据
    connect(&qnode,SIGNAL(plannerPath(QPolygonF)),this,SLOT(paintPlannerPath(QPolygonF)));
    //获取小车运行状态
    connect(&qnode,SIGNAL(updatecarmovestatue(int)),this,SLOT(slot_getcarmovestatue(int)));
    connect(&qnode,SIGNAL(car_achieve_goal_point_signal(int)),this,SLOT(slot_get_car_move_status(int)));
    //获取摄像头数据
    connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_get_image(int,QImage)));
    //获取返回点数据
    connect(&qnode,SIGNAL(sendreturnpos(QPointF,double)),this,SLOT(slot_updatereturnpos(QPointF,double)));
    //获取巡航点数据
    connect(&qnode,SIGNAL(sendmultipos(poselistType)),this,SLOT(slot_updatemultipos(poselistType)));
    //获取电子围栏数据
    connect(&qnode,SIGNAL(sendaddlinepoint(myType)),this,SLOT(slot_updateaddlinepos(myType)));
    //获取小车当前状态
    connect(&qnode,SIGNAL(sendcarstatue(int)),this,SLOT(slot_carstatue(int)));
    //获取小车错误状态
    connect(&qnode,SIGNAL(sendcarerrorstatue(int)),this,SLOT(slot_carerrorstatue(int)));
    //获取小车所在楼层
    connect(&qnode,SIGNAL(sendfloorcar(int)),this,SLOT(slot_floorcar(int)));
    //发送设置返回点信号
    connect(this,SIGNAL(sendsetreturnpossignal()),&qnode,SLOT(slot_getsetreturnpossignal()));

    connect(&qnode,SIGNAL(sendnagstatue(int)),this,SLOT(slot_getnagstatue(int)));

    connect(&qnode,SIGNAL(sendexpressstatue(statuetype)),this,SLOT(getexpressstatue(statuetype)));

    connect(&qnode,SIGNAL(send_laser_fault_signal()),this,SLOT(handle_send_laser_fault_signal()));
    connect(&qnode,SIGNAL(send_imu_fault_signal()),this,SLOT(handle_send_imu_fault_signal()));
    connect(&qnode,SIGNAL(reachPointNum_signal(qint32)),this,SLOT(handle_reachPointNum_signal(qint32)));

    // connect(&qnode,SIGNAL(current_point_signal_(QVector<double>)),this,SLOT(handle_current_point_signal_(QVector<double>)));
     connect(&qnode,SIGNAL(current_point_signal_(double, double, double, double)),this,SLOT(handle_current_point_signal_(double, double, double, double)));

}


//UDP RECEIVE
void MainWindow::dataReceived()
{
    while(udpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(datagram.data(),datagram.size());
        qDebug()<<datagram;
        handle_udpdata(datagram);
    }
}

void MainWindow::slot_send_battery()
{
    // send battery percent
    QByteArray battery_data = battery_str.toLatin1();
    sendcommendwithdata(0x01, robot_ID, battery_data);
    // car_client->flush_tcp_send_buffer();
    // car_client->wait_for_bytes_written();
}

void MainWindow::slot_send_floor_num()
{
    // send current floor number
    QByteArray current_floor_byteArray =  change_int_to_byteArray(floor_num.toInt());
    sendcommendwithdata(floorcar_signal, robot_ID, current_floor_byteArray);  //current floor
    // car_client->flush_tcp_send_buffer();
    // car_client->wait_for_bytes_written();
}

void MainWindow::slot_send_current_car_point()
{
    // send current car point
    sendcommendwithdata(robopos_signal, robot_ID, RoboPose_data_cloud);
}

void MainWindow::slot_send_robot_running_status()
{
    // send robot running status
    QByteArray statue_data;
    statue_data.clear();
    if(ros::master::check())
        statue_data.push_back(0x31);  //char "1"
    else
        statue_data.push_back(0x32);  //char "2"
    sendcommendwithdata(0x02, robot_ID, statue_data);
}

void MainWindow::handle_timeout_send_multi_navi_srart()
{
    qnode.set_datarequest(4);//发送开始巡航命令
    sendcommendwithdata(startmulti_signal, robot_ID, empty_ba);
    qDebug() << "start multi";
    player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/03_start_multi_navigation.mp3"));
    player->play();
    timer_send_start_muti_navigation_signal->stop();
}

void MainWindow::handle_timeout_send_navi_point()
{
    handle_goal_pos(car_goal_pos_);
    sendcommendwithdata(goal_pos_signal, robot_ID, empty_ba);
    player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/01_set_goal_point.mp3"));
    player->play();
    timer_send_navi_point_signal->stop();
}

void MainWindow::handle_timeout_send_car_return()
{
    qnode.set_datarequest(2);//发送开始返航命令
    sendcommendwithdata(startreturn_signal, robot_ID, empty_ba);
    player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/02_start_return.mp3"));
    player->play();
    timer_send_car_return_signal->stop();
}

void MainWindow::handle_timeout_reconnect()
{
    car_client->connect_to_host();
}

void MainWindow::handle_timeout_send_fault()
{
    qDebug() << "handle send fault";
    static int fault_count_threshold = 4;

    laser_fault_count++;
    qDebug() << "laser_count: " << laser_fault_count;
    if(laser_fault_count > fault_count_threshold)
    {
        // qnode.set_datarequest(8);//发送停止小车命令
        QByteArray fault_code;
        fault_code.push_back(fault_laser);
        sendcommendwithdata(send_car_fault, robot_ID, fault_code);
        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/laser_fault.mp3"));
        player->play();
    }

    imu_fault_count++;
    qDebug() << "imu_fault_count: " << imu_fault_count;
    if(imu_fault_count > fault_count_threshold)
    {
        // qnode.set_datarequest(8);//发送停止小车命令
        QByteArray fault_code;
        fault_code.push_back(fault_imu);
        sendcommendwithdata(send_car_fault, robot_ID, fault_code);
        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/imu_fault.mp3"));
        player->play();
    }
}

void MainWindow::handle_send_laser_fault_signal()
{
    laser_fault_count = 0;
}

void MainWindow::handle_send_imu_fault_signal()
{
    imu_fault_count = 0;
}

void MainWindow::slot_get_car_move_status(int status)
{
//    if(status == 3 && current_car_status_ == 3)
//    {
//        multi_navi_loop_count++;
//        qDebug() << "The goal was achieved successfully by the action serve"
//                 << " multi_navi_loop_count % 3 = "
//                 << (multi_navi_loop_count % 3);
//        switch((multi_navi_loop_count % 3))
//        {
//        case 0:
//        {
//            player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/07_first_point.mp3"));
//            player->play();
//            break;
//        }
//        case 1:
//        {
//            player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/08_second_point.mp3"));
//            player->play();
//            break;
//        }
//        case 2:
//        {
//            player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/09_third_point.mp3"));
//            player->play();
//            break;
//        }
//        default:
//            break;
//        }

//    }
}

void MainWindow::handle_reachPointNum_signal(qint32 point_num)
{

    qDebug() << "The goal was achieved successfully by the action serve"
             << " point_num = " << point_num;
   if(multi_navi_play_video_flag == 1)
    {
        switch(point_num)
        {
        case 0:
        {
		    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
    qDebug() << "quit thread run()";
            thread_play_video->file_url = "/home/smarcorobot/video/01_test.mp4";
            qDebug() << "主线程id：" << QThread::currentThreadId();
            thread_play_video->start(); // 启动子线程
            qDebug() << "success to exec thread.";

            player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/07_first_point.mp3"));
            player->play();

        }break;
        case 1:
        {
		    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
    qDebug() << "quit thread run()";
            thread_play_video->file_url = "/home/smarcorobot/video/02_test.mp4";
            qDebug() << "主线程id：" << QThread::currentThreadId();
            thread_play_video->start(); // 启动子线程
            qDebug() << "success to exec thread.";

            player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/08_second_point.mp3"));
            player->play();

        }break;
        case 2:
        {
		    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
    qDebug() << "quit thread run()";
            thread_play_video->file_url = "/home/smarcorobot/video/03_test.mp4";
            qDebug() << "主线程id：" << QThread::currentThreadId();
            thread_play_video->start(); // 启动子线程
            qDebug() << "success to exec thread.";

            player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/09_third_point.mp3"));
            player->play();

        }break;
        default:
            break;
        }
        qDebug() << "play multi navi video switch success";
    }
}

void MainWindow::handle_currentPoint_to_returnPoint()
{
    QMutexLocker locker(m_mutex);
    double return_posx_ = car_current_point_.at(0);
    double return_posy_ = car_current_point_.at(1);
    double return_rawz_ = car_current_point_.at(2);
    double return_raww_ = car_current_point_.at(3);
    qnode.sendreturn_pos(return_posx_,return_posy_,return_rawz_,return_raww_);
    qDebug() << "set return point success.";
    qDebug() << " x = " << car_current_point_.at(0)
             << " y = " << car_current_point_.at(1)
             << " z = " << car_current_point_.at(2)
             << " w = " << car_current_point_.at(3);
}

// void MainWindow::handle_current_point_signal_(QVector<double> current_point_)
void MainWindow::handle_current_point_signal_(double pos_x_, double pos_y_, double pos_z_, double pos_w_)
{
    // car_current_point_ = current_point_;
//    qDebug() << " x = " << car_current_point_.at(0)
//             << " y = " << car_current_point_.at(1)
//             << " z = " << car_current_point_.at(2)
//             << " w = " << car_current_point_.at(3);
//        qDebug() << " x = " << pos_x_
//                 << " y = " << pos_y_
//                 << " z = " << pos_z_
//                 << " w = " << pos_w_;
    car_current_point_.clear();
    car_current_point_.push_back(pos_x_);
    car_current_point_.push_back(pos_y_);
    car_current_point_.push_back(pos_z_);
    car_current_point_.push_back(pos_w_);
}

void MainWindow::handle_udpdata(QByteArray r_data)
{
    int voice_num = ((uint32_t)r_data.at(7)<<24)+((uint32_t)r_data.at(6)<<16)+((uint32_t)r_data.at(5)<<8)+(uint32_t)r_data.at(4);
    if(voice_num == 9000)
    {
        qnode.set_datarequest(8);//发送停止小车命令
        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/catkin_cg/src/ros_qt_demo-master/resources/voice/9000.mp3"));
        player->play();
        return;
    }
    qDebug()<<"voice_numaaaaa:::"<<voice_num;
    player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/catkin_cg/src/ros_qt_demo-master/resources/voice/"+QString::number(voice_num)+".mp3"));
    player->play();
}


void MainWindow::slot_getnagstatue(int nagstatue)
{
    QByteArray nagstatue_ba = QString::number(nagstatue).toLatin1();
    //  sendscok_com_data(sockDesc,carstatue_signal,car_statue_ba);
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),pulsecar_signal,nagstatue_ba);
    }
}


void MainWindow::slot_carstatue(int statue)
{
    QByteArray car_statue_ba = QString::number(statue).toLatin1();
    //  sendscok_com_data(sockDesc,carstatue_signal,car_statue_ba);
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),cartoserverstatue_signal,car_statue_ba);
    }
}

void MainWindow::slot_carerrorstatue(int error_statue)
{
    QByteArray car_error_statue_ba = QString::number(error_statue).toLatin1();
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),cartoservererrorstatue_signal,car_error_statue_ba);
    }
}

void MainWindow::slot_floorcar(int floorcar)
{
    QByteArray floorcar_ba = QString::number(floorcar).toLatin1();
    current_car_floor = QString::number(floorcar).toLatin1();
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),floorcar_signal,floorcar_ba);
        //sendscok_com_data(m_socklist.at(i),floorcar_signal,current_car_floor);
    }
}


void MainWindow::slot_batteryState(float p)//获取电池电压
{
    battery_str = QString::number(static_cast<int>(p));
}

void MainWindow::slot_getmap(QImage map)//获取地图信息
{
    map_ba.clear();//每次传过来先清
    QDataStream ds(&map_ba,QIODevice::ReadWrite);
    //将图片读入array，方便发送
    ds<<map;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),map_signal,map_ba);
    }
    qnode.set_datarequest(1);//获取返回点的值
    qnode.addlineset_datarequest(1);//获取电子围栏的值

    if(app_initstatue)
    {
        for(int i=1;i<11;i++)//地图上来以后对快递柜进行初始化
        {
            qnode.check_express(i);
        }
    }
//    if(power_statue)//上电设置初始位置为返航点位置
//    {
//        //		emit sendsetreturnpossignal();
//        setreturn_timer->start(1000);
//        power_statue = false;
//    }

    //   for (int i=0;i<app_socklist.count();i++)
    //   {
    //      app_sendscok_com_data(app_socklist.at(i),app_changemap_signal,smarcorobot_ID,floor_ba);
    //    }
    
}

void MainWindow::slot_receivewinpointlist(QPolygonF pointlist)//获取地图和场景的中心点
{
    center_ba.clear();
    QDataStream as(&center_ba,QIODevice::ReadWrite);
    as<<pointlist;
}

void MainWindow::slot_updateRoboPose(QPointF point, float raw)//获取小车当前位置
{
    // qDebug() << "send point:" << point;
   
       	RoboPose_x = point.x();
    RoboPose_y = point.y();
    RoboPose_raw = raw;
    RoboPose_data.clear();
    /*
    qint32 int32_RoboPose_x = static_cast<qint32>(RoboPose_x * 10000);
    QString str_int32_RoboPose_x_hex = QString("%1").arg(int32_RoboPose_x, 8, 16, QLatin1Char('0'));
    RoboPose_data[0] = str_int32_RoboPose_x_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data[1] = str_int32_RoboPose_x_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data[2] = str_int32_RoboPose_x_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data[3] = str_int32_RoboPose_x_hex.mid(6, 2).toInt(nullptr, 16);

    qint32 int32_RoboPose_y = static_cast<qint32>(RoboPose_y * 10000);
    QString str_int32_RoboPose_y_hex = QString("%1").arg(int32_RoboPose_y, 8, 16, QLatin1Char('0'));
    RoboPose_data[4] = str_int32_RoboPose_y_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data[5] = str_int32_RoboPose_y_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data[6] = str_int32_RoboPose_y_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data[7] = str_int32_RoboPose_y_hex.mid(6, 2).toInt(nullptr, 16);

    qint32 int32_RoboPose_raw = static_cast<qint32>(RoboPose_raw * 10000);
    QString str_int32_RoboPose_raw_hex = QString("%1").arg(int32_RoboPose_raw, 8, 16, QLatin1Char('0'));
    RoboPose_data[8] = str_int32_RoboPose_raw_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data[9] = str_int32_RoboPose_raw_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data[10] = str_int32_RoboPose_raw_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data[11] = str_int32_RoboPose_raw_hex.mid(6, 2).toInt(nullptr, 16);
    */
    // qDebug() << RoboPose_data.toHex();
    // QString::number(static_cast<int>(p));

     QVector<double> Robo_doubledata;
     Robo_doubledata<<RoboPose_x<<RoboPose_y<<RoboPose_raw;
     QDataStream RoboPose_as(&RoboPose_data,QIODevice::ReadWrite);
     RoboPose_as<<Robo_doubledata;
}

void MainWindow::slot_updateRoboPose_cloud(QPointF point, float raw)
{
    // qDebug() << "send point:" << point;
    RoboPose_x_cloud = point.x();
    RoboPose_y_cloud = point.y();
    RoboPose_raw_cloud = raw;
    RoboPose_data_cloud.clear();
    qint32 int32_RoboPose_x = static_cast<qint32>(RoboPose_x_cloud * 10000);
    QString str_int32_RoboPose_x_hex = QString("%1").arg(int32_RoboPose_x, 8, 16, QLatin1Char('0'));
    RoboPose_data_cloud[0] = str_int32_RoboPose_x_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[1] = str_int32_RoboPose_x_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[2] = str_int32_RoboPose_x_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[3] = str_int32_RoboPose_x_hex.mid(6, 2).toInt(nullptr, 16);

    qint32 int32_RoboPose_y = static_cast<qint32>(RoboPose_y_cloud * 10000);
    QString str_int32_RoboPose_y_hex = QString("%1").arg(int32_RoboPose_y, 8, 16, QLatin1Char('0'));
    RoboPose_data_cloud[4] = str_int32_RoboPose_y_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[5] = str_int32_RoboPose_y_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[6] = str_int32_RoboPose_y_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[7] = str_int32_RoboPose_y_hex.mid(6, 2).toInt(nullptr, 16);

    qint32 int32_RoboPose_raw = static_cast<qint32>(RoboPose_raw_cloud * 10000);
    QString str_int32_RoboPose_raw_hex = QString("%1").arg(int32_RoboPose_raw, 8, 16, QLatin1Char('0'));
    RoboPose_data_cloud[8] = str_int32_RoboPose_raw_hex.mid(0, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[9] = str_int32_RoboPose_raw_hex.mid(2, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[10] = str_int32_RoboPose_raw_hex.mid(4, 2).toInt(nullptr, 16);
    RoboPose_data_cloud[11] = str_int32_RoboPose_raw_hex.mid(6, 2).toInt(nullptr, 16);

    // qDebug() << RoboPose_data.toHex();
    // QString::number(static_cast<int>(p));

    // QVector<double> Robo_doubledata;
    // Robo_doubledata<<RoboPose_x<<RoboPose_y<<RoboPose_raw;
    // QDataStream RoboPose_as(&RoboPose_data,QIODevice::ReadWrite);
    // RoboPose_as<<Robo_doubledata;
}

void MainWindow::slot_updatereturnpos(QPointF point, double raw)//获取返回点位置
{
    returnpos_x = point.x();
    returnpos_y = point.y();
    returnpos_raw = raw;
    returnpos_ba.clear();
    QVector<double> returnposdata;
    returnposdata<<returnpos_x<<returnpos_y<<returnpos_raw;
    QDataStream returnpos_as(&returnpos_ba,QIODevice::ReadWrite);
    returnpos_as<<returnposdata;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),bendreturn_signal,returnpos_ba);
    }
    if(changemap_statue)
    {
        qDebug()<<"changemap_statue";
        //	  emit sendsetreturnpossignal();//设置完成初始位置以后改变状态
        setreturn_timer->start(1000);
        changemap_statue = false;
    }
}

void MainWindow::slot_setreturntimer()
{
    // emit sendsetreturnpossignal();
    setreturn_timer->stop();
    app_setstart_timer->start(2000);
}

void MainWindow::slot_setstarttimer()
{
    QByteArray app_setreturn_ba;
    app_setreturn_ba.clear();
    for (int i=0;i<app_socklist.count();i++)
    {
        app_sendscok_com_data(app_socklist.at(i),app_changemap_signal,smarcorobot_ID,app_setreturn_ba);
    }
    app_setstart_timer->stop();
}

void MainWindow::slot_updatemultipos(poselistType poslist)//获取巡航点数据
{
    QVector<QVector<double>> posedouble_list;
    QByteArray poselist_ba;
    for (int i=0;i<poslist.posearray.count();i++)
    {
        tf::Quaternion quat;
        QVector<double> double_list;
        double_list.clear();
        geometry_msgs::Pose pos;
        pos.orientation.z = poslist.posearray.at(i).at(2);
        pos.orientation.w = poslist.posearray.at(i).at(3);
        tf::quaternionMsgToTF(pos.orientation, quat);
        double roll, pitch, yaw;//定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
        double_list<<poslist.posearray.at(i).at(0)<<poslist.posearray.at(i).at(1)<<yaw;
        posedouble_list.append(double_list);
    }
    QDataStream returnpos_as(&poselist_ba,QIODevice::ReadWrite);
    returnpos_as<<posedouble_list;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),multipos_signal,poselist_ba);
    }

}

void MainWindow::slot_updateaddlinepos(myType poslist)//获取电子围栏数据
{
    QVector<QVector<QPointF>> pointline_list;
    pointline_list = poslist.m_DstIdList;
    QVector<QPolygonF> addlinelist;
    QPolygonF linelist;
    for (int i=0;i<pointline_list.count();i++)
    {
        linelist.clear();
        linelist<<pointline_list.at(i).at(0)<<pointline_list.at(i).at(1);
        addlinelist.push_back(linelist);
    }
    addlinelist_ba.clear();
    QDataStream addline_as(&addlinelist_ba,QIODevice::ReadWrite);
    addline_as<<addlinelist;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),addline_signal,addlinelist_ba);
    }

}

void MainWindow::slot_updatalaserscan(QPolygonF laserscan)//获取雷达数据
{
    laserscan_ba.clear();
    QDataStream as(&laserscan_ba,QIODevice::ReadWrite);
    as<<laserscan;
}

void MainWindow::paintPlannerPath(QPolygonF planner)//获取路径数据
{
    planner_ba.clear();
    QDataStream as(&planner_ba,QIODevice::ReadWrite);
    as<<planner;
}

void MainWindow::slot_getcarmovestatue(int statue)//获取小车运行状态
{
    food_carstatue = statue;

    QVector<int> foodcarstatue;
    QByteArray foodcarstatue_ba;
    foodcarstatue.append(food_clientstatue);
    foodcarstatue.append(food_carstatue);

    QDataStream foodcarstatue_as(&foodcarstatue_ba,QIODevice::ReadWrite);
    foodcarstatue_as<<foodcarstatue;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),foodstatue_signal,foodcarstatue_ba);
    }

    if((statue == 2)&&((client_statue == 1)||(client_statue == 2)||(client_statue == 3)))
    {
        statue = 10;
    }
    if((statue == 2)&&(client_statue == 0))
    {
        statue = 3;
    }
    if(client_statue==3)
    {
        car_statue = 10;
        return;
    }
    car_statue = statue;

    if(goal_statue)
    {
        autoback_timer->start(1000);
        goal_statue = false;
    }
    else {
        autoback_timer->stop();
    }
    qDebug()<<jinyin_clientstatue<<":::"<<jinyin_carstatue;
    //金鹰软件状态判定
    if(jinyin_clientstatue!=2)
    {
        jinyin_carstatue = statue;
        if(((jinyin_clientstatue==4)||(jinyin_clientstatue==3))&&(jinyin_carstatue==3))
        {
            jinyin_clientstatue=0;
        }
    }
    else
    {
        jinyin_carstatue = 0;
    }
    QVector<int> jinyincarstatue_ve;
    QByteArray jinyincarstatue_ba;
    jinyincarstatue_ve.append(jinyin_clientstatue);
    jinyincarstatue_ve.append(jinyin_carstatue);
    QDataStream jinyincarstatue_as(&jinyincarstatue_ba,QIODevice::ReadWrite);
    jinyincarstatue_as<<jinyincarstatue_ve;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),jinyin_statue_signal,jinyincarstatue_ba);
    }
    QByteArray app_carrunstatue_ba;
    app_carrunstatue_ba.clear();
    if((app_carrunstatue == 0)&&(statue == 3))
    {
        app_carrunstatue = 3;
        for (int i=0;i<app_socklist.count();i++)
        {
            app_sendscok_com_data(app_socklist.at(i),app_carrunstatue_signal,smarcorobot_ID,app_carrunstatue_ba);
        }
    }
}

void MainWindow::slot_get_image(int frame_id,QImage image)//获取摄像头数据
{
    image0 = image;
}

void MainWindow::slot_autobacktimer()
{
    timercount++;
    if(timercount == 60)
    {
        qnode.set_datarequest(2);//发送开始返航命令
        timercount = 0;
    }
}


void MainWindow::slot_recesockDesc(QList<int> socklist)
{
    m_socklist = socklist;
}

void MainWindow::getexpressstatue(statuetype expressstatue)//处理返回的状态
{
    if(express_statue.value(expressstatue.statue_type.at(0)) != expressstatue.statue_type.at(1))
    {
        express_statue.insert(expressstatue.statue_type.at(0),expressstatue.statue_type.at(1));
        QByteArray expressstatue_data;
        expressstatue_data.clear();
        uint8_t key = expressstatue.statue_type.at(0);
        uint8_t value = expressstatue.statue_type.at(1);
        expressstatue_data.push_back(key);
        expressstatue_data.push_back(value);
        qDebug()<<expressstatue_data;
        for (int i=0;i<app_socklist.count();i++)
        {
            app_sendscok_com_data(app_socklist.at(i),express_statue_signal,smarcorobot_ID,expressstatue_data);
        }
    }
}

void MainWindow::recvData(QString ip, int port,int sockDesc,QByteArray datagram)
{
    if(!data_sur.isEmpty())
    {
        datagram = data_sur + datagram;
        if(((uint8_t)datagram.at(0)!=0x83)||((uint8_t)datagram.at(1)!=0x84))//包头
        {
            datagram.clear();
            data_sur.clear();
            return;
        }
    }
    else
    {
        if(((uint8_t)datagram.at(0)!=0x83)||((uint8_t)datagram.at(1)!=0x84))//包头
        {
            datagram.clear();
            return;
        }
    }
    int datagarm_count = datagram.count();
    while (datagarm_count) //等待处理完这一帧数据
    {
        if(datagarm_count<7)//剩余数据读不到长度
        {
            data_sur = datagram;//将剩余数据存下，下次继续解析
            return;
        }
        if((uint8_t)datagram.at(0)==0x83)//包头
        {
            if((uint8_t)datagram.at(1)==0x84)//包头
            {
                uint8_t datagram_at6 = (uint8_t)datagram.at(6);//现将二进制数据流转为uint_8型，否则数据转32位会出错
                uint8_t datagram_at5 = (uint8_t)datagram.at(5);
                uint8_t datagram_at4 = (uint8_t)datagram.at(4);
                uint8_t datagram_at3 = (uint8_t)datagram.at(3);
                m_count = ((uint32_t)datagram_at3<<24)+((uint32_t)datagram_at4<<16)+((uint32_t)datagram_at5<<8)+(uint32_t)datagram_at6;//得到发送的数据包的大小
                if(datagarm_count<m_count)//剩余数据小于包数据长度
                {
                    data_sur = datagram;//将剩余数据存下，下次继续解析
                    return;
                }
                m_commend = (uint8_t)datagram.at(2);//取出命令
                if(((uint8_t)datagram.at(m_count-2)!=0xF4)||((uint8_t)datagram.at(m_count-1)!=0x4F))//判断包尾
                {
                    qDebug()<<"receive error!!!";
                    datahandle.clear();//清理处理完的数据，释放内存
                    data_sur.clear();
                    m_count=0;
                    m_commend = 0xFF;
                    return;//接收数据错误，没有接收到包尾
                }
                datahandle = datagram;
                datahandle.remove(m_count-1,datagarm_count-m_count);//先去除尾部本次不处理数据
                datahandle.remove(m_count-2,2);
                datahandle.remove(0,7);
                handledata(sockDesc,m_commend,datahandle);//处理本次取出的数据
                m_commend = 0xFF;//数据处理完将命令还原
                datagram.remove(0,m_count);//去除数据链中处理完的数据
                datagarm_count = datagarm_count - m_count;//记录剩下未处理的数据长度
                datahandle.clear();//清理处理完的数据，释放内存
                data_sur.clear();
            }
        }
    }
}

void MainWindow::handledata(int sockDesc,uint8_t commend,QByteArray r_data)
{
    switch (commend)
    {
    case 0x00:
    {
        on_button_connect_clicked(false);
    }break;
    case 0x01:
    {
        QByteArray battery_data = battery_str.toLatin1();
        sendscok_com_data(sockDesc,0x01,battery_data);
    }break;
    case 0x02:
    {
        QByteArray statue_data;
        statue_data.clear();
        if(ros::master::check())
        {
            statue_data.push_back(0x01);
        }
        else {
            statue_data.push_back(0x02);
        }
        sendscok_com_data(sockDesc,0x02,statue_data);
    }break;
    case left_up_signal:
    {
        qnode.move_base(!use_all_statue?'U':'u',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case up_signal:
    {
        qnode.move_base(!use_all_statue?'I':'i',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case right_up_signal:
    {
        qnode.move_base(!use_all_statue?'O':'o',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case left_signal:
    {
        qnode.move_base(!use_all_statue?'J':'j',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case stopmove_signal:
    {
        qnode.cancel_action();
        qnode.move_base(!use_all_statue?'U':'u',0.0,0.0);
        break;
    }
    case right_signal:
    {
        qnode.move_base(!use_all_statue?'L':'l',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case left_down_signal:
    {
        qnode.move_base(!use_all_statue?'M':'m',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case down_signal:
    {
        qnode.move_base(!use_all_statue?'<':',',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case right_down_signal:
    {
        qnode.move_base(!use_all_statue?'>':'.',line_speed*0.01,raw_speed*0.01);
        break;
    }
    case checkBox_use_all_signal:
    {
        use_all_statue = QString(r_data).toInt();

    }break;
    case raw_signal:
    {
        raw_speed = QString(r_data).toInt();
    }break;
    case line_signal:
    {
        line_speed = QString(r_data).toInt();
    }break;
    case map_signal:
    {
        sendscok_com_data(sockDesc,map_signal,map_ba);
    }break;
    case centerpointlist_signal:
    {
        sendscok_com_data(sockDesc,centerpointlist_signal,center_ba);
    }break;
    case robopos_signal:
    {
        sendscok_com_data(sockDesc,robopos_signal,RoboPose_data);
        /*
	if(!setstartstatue)
        {
            emit sendsetreturnpossignal();//上电后设置初始位置为返航点位置，单机测试没有问题，不能多机，会重复调用此线程
            setstartstatue = true;
        }
	*/
    }break;
    case laserscan_signal:
    {
        sendscok_com_data(sockDesc,laserscan_signal,laserscan_ba);
    }break;
    case current_pos_signal:
    {
        hanglecurrent_pos(r_data);
    }break;
    case goal_pos_signal:
    {
        hanglegoal_pos(r_data);
    }break;
    case planner_signal:
    {
        sendscok_com_data(sockDesc,planner_signal,planner_ba);
    }break;
    case carstatue_signal:
    {
        QString car_statue_str = QString::number(car_statue);
        QByteArray car_statue_ba = car_statue_str.toLatin1();
        sendscok_com_data(sockDesc,carstatue_signal,car_statue_ba);
    }break;
    case clientstatue_signal:
    {
        hangleclientstatue(r_data);
    }break;
    case image0_signal:
    {
        QByteArray image0_ba;
        image0_ba.clear();//每次传过来先清
        QDataStream ds(&image0_ba,QIODevice::ReadWrite);
        //将图片读入array，方便发送
        ds<<image0;
        sendscok_com_data(sockDesc,image0_signal,image0_ba);
    }break;
    case return_pos_signal:
    {
        hanglereturn_pos(r_data);
    }break;
    case startreturn_signal:
    {
        qnode.set_datarequest(2);//发送开始返航命令
    }break;
    case bendreturn_signal:
    {
        sendscok_com_data(sockDesc,bendreturn_signal,returnpos_ba);
    }break;
    case multipos_signal:
    {
        hanglemulti_pos(r_data);
    }break;
    case startmulti_signal:
    {
        qnode.set_datarequest(4);//发送开始巡航命令
    }break;
    case pausemulti_signal:
    {
        qnode.set_datarequest(5);//发送停止巡航命令
    }break;
    case stopcar_signal:
    {
        qnode.set_datarequest(8);//发送停止小车命令
    }break;
    case addline_signal:
    {
        hangleaddlinedata(r_data);
    }break;
    case bendaddline_signal:
    {
        sendscok_com_data(sockDesc,addline_signal,addlinelist_ba);
    }break;
    case bendmultipos_signal:
    {
        qnode.set_datarequest(3);
    }break;
    case food_clientstatue_signal:
    {
        handle_food_carstatue(sockDesc,r_data);
    }break;
    case autobackhome_signal:
    {
        handle_autobackhomestatue(r_data);
    }break;
    case changemap_signal:
    {
        handle_changemapdata(r_data);
    }break;
    case floornum_signal:
    {
        handle_floornum(r_data);
    }break;
    case carrunstatue_signal:
    {
        handle_carrunstatue(r_data);
    }break;
    default:
        break;
    }
}

//处理发送过来的设置初始位置的点位
void MainWindow::hanglecurrent_pos(QByteArray r_data)
{
    QPolygonF current_pos;
    QDataStream current_pos_ds(&r_data,QIODevice::ReadWrite);
    current_pos_ds>>current_pos;
    QPointF startcurrentpos = current_pos.at(0);
    QPointF lastcurrentpos = current_pos.at(1);
    geometry_msgs::Quaternion currentpos_quat;
    double tan_radian = (lastcurrentpos.y()-startcurrentpos.y())/(lastcurrentpos.x()-startcurrentpos.x());
    double radian = atan(tan_radian);
    if(lastcurrentpos.x()>=startcurrentpos.x())
    {
        currentpos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastcurrentpos.x()<startcurrentpos.x()) &&  (lastcurrentpos.y()>=startcurrentpos.y()))
    {
        currentpos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastcurrentpos.x()<startcurrentpos.x()) &&  (lastcurrentpos.y()<startcurrentpos.y()))
    {
        currentpos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    qnode.set_start("map",startcurrentpos.x(),startcurrentpos.y(),currentpos_quat.z,currentpos_quat.w);
}

//QPolygonF MainWindow::handle_received_point(QByteArray r_data)
//{

//}
//处理发送过来的设置目标位置的点位
void MainWindow::hanglegoal_pos(QByteArray r_data)
{
    qDebug()<<"r_data::::"<<r_data;
    QPolygonF goal_pos;
    QDataStream goal_pos_ds(&r_data,QIODevice::ReadWrite);
    goal_pos_ds>>goal_pos;
    qDebug()<<"goal_pos:::"<<goal_pos;
    QPointF startgoalpos = goal_pos.at(0);
    QPointF lastgoalpos = goal_pos.at(1);
    geometry_msgs::Quaternion goal_pos_quat;
    double tan_radian = (lastgoalpos.y()-startgoalpos.y())/(lastgoalpos.x()-startgoalpos.x());
    double radian = atan(tan_radian);
    if(lastgoalpos.x()>=startgoalpos.x())
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastgoalpos.x()<startgoalpos.x()) &&  (lastgoalpos.y()>=startgoalpos.y()))
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastgoalpos.x()<startgoalpos.x()) &&  (lastgoalpos.y()<startgoalpos.y()))
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    qnode.set_goal("map",startgoalpos.x(),startgoalpos.y(),goal_pos_quat.z,goal_pos_quat.w);
    if(autoback_statue==1)
    {
        goal_statue = 1;//单点导航标志置为一
    }

}

//处理发送过来的设置返航点位置的点位
void MainWindow::hanglereturn_pos(QByteArray r_data)
{
    QPolygonF return_pos;
    QDataStream return_pos_ds(&r_data,QIODevice::ReadWrite);
    return_pos_ds>>return_pos;
    // QPointF start_return_pso = QPointF(20, 20);
    QPointF startreturnpos = return_pos.at(0);
    QPointF lastreturnpos = return_pos.at(1);
    geometry_msgs::Quaternion return_pos_quat;
    double tan_radian = (lastreturnpos.y()-startreturnpos.y())/(lastreturnpos.x()-startreturnpos.x());
    double radian = atan(tan_radian);
    if(lastreturnpos.x()>=startreturnpos.x())
    {
        return_pos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastreturnpos.x()<startreturnpos.x()) &&  (lastreturnpos.y()>=startreturnpos.y()))
    {
        return_pos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastreturnpos.x()<startreturnpos.x()) &&  (lastreturnpos.y()<startreturnpos.y()))
    {
        return_pos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    return_posx = startreturnpos.x();
    return_posy = startreturnpos.y();
    return_rawz = return_pos_quat.z;
    return_raww = return_pos_quat.w;
    //剩下向小车发送返航点消息没写
    qnode.sendreturn_pos(return_posx,return_posy,return_rawz,return_raww);
}

//处理发送过来的设置巡航点位置的点位
void MainWindow::hanglemulti_pos(QByteArray r_data)
{
    QVector<QPolygonF> qvmulti_pos;
    QPolygonF multi_pos;
    QDataStream current_pos_ds(&r_data,QIODevice::ReadWrite);
    current_pos_ds>>qvmulti_pos;
    qnode.handlemultiposlist(qvmulti_pos);
}

//处理发送过来的客户端操作状态
void MainWindow::hangleclientstatue(QByteArray r_data)
{
    client_statue = QString(r_data).toInt();
    if((client_statue == 1)||(client_statue == 2)||(client_statue == 3))
    {
        car_statue = 10;
    }
}

//处理发送过来的电子围栏数据
void MainWindow::hangleaddlinedata(QByteArray r_data)
{
    QVector<QPolygonF> addline_pos_vp;
    QPolygonF addline_pos;
    QDataStream addline_pos_ds(&r_data,QIODevice::ReadWrite);
    addline_pos_ds>>addline_pos_vp;
    qnode.sendline_modify(addline_pos_vp);
}

//处理餐车发送过来的状态
void MainWindow::handle_food_carstatue(int sockDesc,QByteArray r_data)
{
    food_clientstatue = QString(r_data).toInt();
    food_carstatue = 0;
    QVector<int> foodcarstatue;
    QByteArray foodcarstatue_ba;
    foodcarstatue.append(food_clientstatue);
    foodcarstatue.append(food_carstatue);

    QDataStream foodcarstatue_as(&foodcarstatue_ba,QIODevice::ReadWrite);
    foodcarstatue_as<<foodcarstatue;
    sendscok_com_data(sockDesc,foodstatue_signal,foodcarstatue_ba);
}

//处理发送过来的是否自动返航的标志
void MainWindow::handle_autobackhomestatue(QByteArray r_data)
{
    int statue = QString(r_data).toInt();
    if(statue==0)
    {
        autoback_statue = 0;
    }
    else
    {
        autoback_statue = 1;
    }
}

//处理发送过来的地图
void MainWindow::handle_changemapdata(QByteArray r_data)
{
    QDataStream ds1(&r_data,QIODevice::ReadWrite);
    ds1>>n_image;
    n_image.save("/home/smarcorobot/smarco_robot/src/turn_on_smarco_robot/map/smarco.pgm","PGM");
}

//处理发送过来的楼层
void MainWindow::handle_floornum(QByteArray r_data)
{
    qDebug()<<r_data;
    floor_num = QString(r_data);
    qnode.sendfloornum(floor_num.toInt());
    setstartstatue = false;
}

//处理发送过来的车辆操控状态
void MainWindow::handle_carrunstatue(QByteArray r_data)
{
    switch (r_data.at(0))
    {
    case 0x00:
    {
        jinyin_clientstatue = 0;
        jinyin_carstatue = 3;
    }break;
    case 0x01:
    {
        jinyin_clientstatue = 1;
        jinyin_carstatue = 0;
    }break;
    case 0x02:
    {
        jinyin_clientstatue = 2;
        jinyin_carstatue = 0;
    }break;
    case 0x03:
    {
        jinyin_clientstatue = 3;
        jinyin_carstatue = 0;
    }break;
    case 0x04:
    {
        jinyin_clientstatue = 4;
        jinyin_carstatue = 0;
    }break;
    default:
        break;
    }
    QVector<int> jinyincarstatue_ve;
    QByteArray jinyincarstatue_ba;
    jinyincarstatue_ve.append(jinyin_clientstatue);
    jinyincarstatue_ve.append(jinyin_carstatue);
    QDataStream jinyincarstatue_as(&jinyincarstatue_ba,QIODevice::ReadWrite);
    jinyincarstatue_as<<jinyincarstatue_ve;
    for (int i=0;i<m_socklist.count();i++)
    {
        sendscok_com_data(m_socklist.at(i),jinyin_statue_signal,jinyincarstatue_ba);
    }
}

//发送描述符+命令+数据
void MainWindow::sendscok_com_data(int sockDesc, uint8_t int_commend,QByteArray mdata)
{
    QByteArray mdata_senddata;
    QByteArray mdata_val;
    int mdata_count = mdata.count() + 9;
    uint8_t data_send[mdata_count];
    data_send[0] = 0x83;
    data_send[1] = 0x84;
    data_send[2] = int_commend;
    data_send[3] = mdata_count>>24;
    data_send[4] = mdata_count>>16;
    data_send[5] = mdata_count>>8;
    data_send[6] = mdata_count;
    for (int j=0;j<7;j++)
    {
        mdata_val.push_back(data_send[j]);
    }
    mdata_senddata = mdata_val + mdata;
    mdata_senddata.push_back(0xF4);
    mdata_senddata.push_back(0x4F);
    emit sendData(sockDesc,mdata_senddata);
}

void MainWindow::showConnection(int sockDesc)
{

}
void MainWindow::showDisconnection(int sockDesc)
{

}

MainWindow::~MainWindow()
{
    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    delete m_server;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    //    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if(connect_statue)
    {
        return;
    }
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.connect_statue->setText("已连接");
        }
    } else {
        if ( ! qnode.init("http://192.168.1.100:11311/",
                          "192.168.1.100") ) {
            //			showNoMasterMessage();
            qDebug()<<"aaaaaaaaaaaaaaaaa";
            qDebug()<<"qnode.init(...) fail!";
            return;
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
            ui.connect_statue->setText("已连接");
            connect_statue = true;
            //      on_connect_cloud_clicked();
            //      qnode.Sub_Image("topics[0]",0);
        }
    }

}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

//查询当前master在线状态
void MainWindow::on_check_statue_btu_clicked()
{
    //    qnode.open_express(2);
    qnode.check_express(2);
    qDebug()<<"open222";

    //  if (ros::master::check())
    //  {
    //    ui.connect_statue->setText("已连接");
    //  }
    //  else
    //  {
    //    ui.connect_statue->setText("未连接");
    //   connect_statue = false;
    //   ui.button_connect->setEnabled(true);
    // }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::car_cloud_connect()
{
    //Ip_str = ui.line_IP->text();
    //port_str = ui.line_port->text().toInt();
    
    Ip_str = "221.231.13.226";
    port_str = 58092;
    car_client = new Client_thread(Ip_str,port_str);
    connect(car_client,SIGNAL(signal_connected(void)),this,SLOT(slot_connected(void)));
    connect(car_client,SIGNAL(signal_disconnected(void)),this,SLOT(slot_disconnected(void)));
    connect(car_client,SIGNAL(signal_senddata(QByteArray)),this,SLOT(slot_receivedata(QByteArray)));
    connect(car_client,SIGNAL(signal_senderror(QAbstractSocket::SocketError)),this,SLOT(slot_geterror(QAbstractSocket::SocketError)));
    mythread = new QThread;
    car_client->moveToThread(mythread);
    mythread->start();
    mtimer = new QTimer(this);
    connect(mtimer,SIGNAL(timeout()),this,SLOT(slot_flash()));
    // mtimer->start(10000);
    login_timer = new QTimer(this);
    connect(login_timer,SIGNAL(timeout()),this,SLOT(slot_loginflash()));
    socket_timer = new QTimer(this);
    connect(socket_timer,SIGNAL(timeout()),this,SLOT(slot_flashconnectTohost()));

    timer_send_battery = new QTimer(this);
    connect(timer_send_battery, SIGNAL(timeout()), this, SLOT(slot_send_battery()));
    // timer_send_battery->start(11234);

    timer_send_floor_num = new QTimer(this);
    connect(timer_send_floor_num, SIGNAL(timeout()), this, SLOT(slot_send_floor_num()));
    // timer_send_floor_num->start(13456);

    timer_send_current_car_point = new QTimer(this);
    connect(timer_send_current_car_point, SIGNAL(timeout()), this, SLOT(slot_send_current_car_point()));
    // 1 seconds interval to send car current point
    // timer_send_current_car_point->start(1234);

    timer_send_robot_running_status = new QTimer(this);
    connect(timer_send_robot_running_status, SIGNAL(timeout()), this, SLOT(slot_send_robot_running_status()));
    // timer_send_robot_running_status->start(17891);

    timer_send_start_muti_navigation_signal = new QTimer(this);
    connect(timer_send_start_muti_navigation_signal, SIGNAL(timeout()), this, SLOT(handle_timeout_send_multi_navi_srart()));

    timer_send_navi_point_signal = new QTimer(this);
    connect(timer_send_navi_point_signal, SIGNAL(timeout()), this, SLOT(handle_timeout_send_navi_point()));

    timer_send_car_return_signal = new QTimer(this);
    connect(timer_send_car_return_signal, SIGNAL(timeout()), this, SLOT(handle_timeout_send_car_return()));

    timer_reconnect_ = new QTimer(this);
    connect(timer_reconnect_, SIGNAL(timeout()), this, SLOT(handle_timeout_reconnect()));

    timer_send_fault = new QTimer(this);
    connect(timer_send_fault, SIGNAL(timeout()), this, SLOT(handle_timeout_send_fault()));

 //   emit sendsetreturnpossignal();//上电后设置初始位置为返航点位置，单机测试没有问题，不能多机，会重复调用此线程
}


void MainWindow::on_connect_cloud_clicked()
{
    Ip_str = ui.line_IP->text();
    port_str = ui.line_port->text().toInt();
    car_client = new Client_thread(Ip_str,port_str);
    connect(car_client,SIGNAL(signal_connected(void)),this,SLOT(slot_connected(void)));
    connect(car_client,SIGNAL(signal_disconnected(void)),this,SLOT(slot_disconnected(void)));
    connect(car_client,SIGNAL(signal_senddata(QByteArray)),this,SLOT(slot_receivedata(QByteArray)));
    connect(car_client,SIGNAL(signal_senderror(QAbstractSocket::SocketError)),this,SLOT(slot_geterror(QAbstractSocket::SocketError)));
    mythread = new QThread;
    car_client->moveToThread(mythread);
    mythread->start();
    mtimer = new QTimer(this);
    connect(mtimer,SIGNAL(timeout()),this,SLOT(slot_flash()));
    // mtimer->start(10000);
    login_timer = new QTimer(this);
    connect(login_timer,SIGNAL(timeout()),this,SLOT(slot_loginflash()));
    socket_timer = new QTimer(this);
    connect(socket_timer,SIGNAL(timeout()),this,SLOT(slot_flashconnectTohost()));

    timer_send_battery = new QTimer(this);
    connect(timer_send_battery, SIGNAL(timeout()), this, SLOT(slot_send_battery()));
    // timer_send_battery->start(11234);

    timer_send_floor_num = new QTimer(this);
    connect(timer_send_floor_num, SIGNAL(timeout()), this, SLOT(slot_send_floor_num()));
    // timer_send_floor_num->start(13456);

    timer_send_current_car_point = new QTimer(this);
    connect(timer_send_current_car_point, SIGNAL(timeout()), this, SLOT(slot_send_current_car_point()));
    // 1 seconds interval to send car current point
    // timer_send_current_car_point->start(1234);

    timer_send_robot_running_status = new QTimer(this);
    connect(timer_send_robot_running_status, SIGNAL(timeout()), this, SLOT(slot_send_robot_running_status()));
    // timer_send_robot_running_status->start(17891);

    timer_send_start_muti_navigation_signal = new QTimer(this);
    connect(timer_send_start_muti_navigation_signal, SIGNAL(timeout()), this, SLOT(handle_timeout_send_multi_navi_srart()));

    timer_send_navi_point_signal = new QTimer(this);
    connect(timer_send_navi_point_signal, SIGNAL(timeout()), this, SLOT(handle_timeout_send_navi_point()));

    timer_send_car_return_signal = new QTimer(this);
    connect(timer_send_car_return_signal, SIGNAL(timeout()), this, SLOT(handle_timeout_send_car_return()));

    timer_reconnect_ = new QTimer(this);
    connect(timer_reconnect_, SIGNAL(timeout()), this, SLOT(handle_timeout_reconnect()));

    timer_send_fault = new QTimer(this);
    connect(timer_send_fault, SIGNAL(timeout()), this, SLOT(handle_timeout_send_fault()));
}

void MainWindow::slot_connected()
{
    ui.label_cloud->setText("已连接云端");
    if(socket_timer->isActive())
    {
        socket_timer->stop();
        connect_count = 0;
    }
    login_timer->start(1000);
    // mtimer->start(10000);
    timer_reconnect_->stop();
    // send login message
    sendcommendwithdata(connect_to_cloud_plat, robot_ID, empty_ba);
}

void MainWindow::slot_disconnected()
{
    mtimer->stop();
    timer_send_battery->stop();
    timer_send_floor_num->stop();
    timer_send_current_car_point->stop();
    timer_send_robot_running_status->stop();
    timer_send_fault->stop();

    ui.label_cloud->setText("云端断开");
    // car_client->connect_to_host();//掉线后立即重新连接
    timer_reconnect_->start(10000);
}

void MainWindow::slot_geterror(QAbstractSocket::SocketError error)
{
    socket_timer->start(connect_count * 1000+1000);
    if(connect_count<179)
        connect_count++;
}

void MainWindow::slot_flashconnectTohost()
{
    qDebug() << "reconnecting ...";
    car_client->connect_to_host();
}

void MainWindow::slot_receivedata(QByteArray datagram)
{
    if(!cloud_data_sur.isEmpty())
    {
        datagram = cloud_data_sur + datagram;
        if(((uint8_t)datagram.at(0)!=0x83)||((uint8_t)datagram.at(1)!=0x84))//包头
        {
            datagram.clear();
            cloud_data_sur.clear();
            return;
        }
    }
    else
    {
        if(((uint8_t)datagram.at(0)!=0x83)||((uint8_t)datagram.at(1)!=0x84))//包头
        {
            datagram.clear();
            return;
        }
    }
    int datagarm_count = datagram.count();
    while (datagarm_count) //等待处理完这一帧数据
    {
        if(datagarm_count<7)//剩余数据读不到长度
        {
            cloud_data_sur = datagram;//将剩余数据存下，下次继续解析
            return;
        }
        if((uint8_t)datagram.at(0)==0x83)//包头
        {
            if((uint8_t)datagram.at(1)==0x84)//包头
            {
                uint8_t datagram_at6 = (uint8_t)datagram.at(6);//现将二进制数据流转为uint_8型，否则数据转32位会出错
                uint8_t datagram_at5 = (uint8_t)datagram.at(5);
                uint8_t datagram_at4 = (uint8_t)datagram.at(4);
                uint8_t datagram_at3 = (uint8_t)datagram.at(3);
                cloud_m_count = ((uint32_t)datagram_at3<<24)+((uint32_t)datagram_at4<<16)+((uint32_t)datagram_at5<<8)+(uint32_t)datagram_at6;//得到发送的数据包的大小
                if(datagarm_count<cloud_m_count)//剩余数据小于包数据长度
                {
                    cloud_data_sur = datagram;//将剩余数据存下，下次继续解析
                    return;
                }
                cloud_m_commend = (uint8_t)datagram.at(2);//取出命令
                uint8_t datagram_at10 = (uint8_t)datagram.at(10);//现将二进制数据流转为uint_8型，否则数据转32位会出错
                uint8_t datagram_at9 = (uint8_t)datagram.at(9);
                uint8_t datagram_at8 = (uint8_t)datagram.at(8);
                uint8_t datagram_at7 = (uint8_t)datagram.at(7);
                robot_ID = ((uint32_t)datagram_at7<<24)+((uint32_t)datagram_at8<<16)+((uint32_t)datagram_at9<<8)+(uint32_t)datagram_at10;//得到机器人ID
                if(((uint8_t)datagram.at(cloud_m_count-2)!=0xF4)||((uint8_t)datagram.at(cloud_m_count-1)!=0x4F))//判断包尾
                {
                    qDebug()<<"receive error!!!";
                    cloud_datahandle.clear();//清理处理完的数据，释放内存
                    cloud_data_sur.clear();
                    cloud_m_count=0;
                    datagram.clear();
                    datagarm_count = 0;
                    robot_ID = 0;
                    //                    m_commend = 0xFF;//将命令复位
                    return;//接收数据错误，没有接收到包尾
                }
                cloud_datahandle = datagram;
                cloud_datahandle.remove(cloud_m_count-1,datagarm_count-cloud_m_count);//先去除尾部本次不处理数据
                // cloud_datahandle.remove(cloud_m_count-4,4);
                cloud_datahandle.remove(cloud_m_count-2,2);   // no CRC
                cloud_datahandle.remove(0,11);
                cloud_handledata(cloud_m_commend,robot_ID,cloud_datahandle);//处理本次取出的数据
                cloud_m_commend = 0xFF;//数据处理完将命令还原
                datagram.remove(0,cloud_m_count);//去除数据链中处理完的数据
                datagarm_count = datagarm_count - cloud_m_count;//记录剩下未处理的数据长度
                cloud_datahandle.clear();//清理处理完的数据，释放内存
                cloud_data_sur.clear();
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }
    }
}

void MainWindow::cloud_handledata(uint8_t commend,uint32_t roboid, QByteArray r_data)
{
    switch (commend)
    {
    case Connect_signal:    //0x00
    {
        //    on_button_connect_clicked(false);
        sendcommendwithdata(Connect_signal, roboid, empty_ba);
        //app_sendscok_com_data(sockDesc,Connect_signal,smarcorobot_ID,empty_ba);
    }break;
    case battery_signal:// 0x01发送电压数据
    {
        qDebug()<<"收到0x01数据";
        QByteArray battery_data = battery_str.toLatin1();
        sendcommendwithdata(0x01, roboid, battery_data);
    }break;
    case robotstatue_signal:    //0x02
    {
        QByteArray statue_data;
        statue_data.clear();
        if(ros::master::check())
        {
            statue_data.push_back(0x31);  //char "1"
        }
        else {
            statue_data.push_back(0x32);  //char "2"
        }
        sendcommendwithdata(0x02, roboid, statue_data);
        // sendscok_com_data(sockDesc,0x02,statue_data);
    }break;
    case map_signal:    //0x03
    {
        QByteArray current_floor_byteArray_ =  change_int_to_byteArray(floor_num.toInt());
        QByteArray total_floor_byteArray_ =  change_int_to_byteArray(total_floor.toInt());
        QByteArray map_ba_ = current_floor_byteArray_ + \
                             total_floor_byteArray_ + \
                             map_ba;
        sendcommendwithdata(map_signal, roboid, map_ba_);
        // sendscok_com_data(sockDesc,map_signal,map_ba);
	qDebug() << "send map success";
    }break;
    case robopos_signal:    //0x05
    {
        sendcommendwithdata(robopos_signal, roboid, RoboPose_data_cloud);
        // sendscok_com_data(sockDesc,robopos_signal,RoboPose_data);
        /*
	if(!setstartstatue)
        {
           // emit sendsetreturnpossignal();//上电后设置初始位置为返航点位置，单机测试没有问题，不能多机，会重复调用此线程
            setstartstatue = true;
        }
	*/
    }break;
    case startreturn_signal:   // 0x0E
    {
        qnode.set_datarequest(8);//发送停止小车命令
        qnode.set_datarequest(8);//发送停止小车命令
        current_car_status_ = 2;
        timer_send_car_return_signal->start(1000);
//        qnode.set_datarequest(2);//发送开始返航命令
//        sendcommendwithdata(startreturn_signal, roboid, empty_ba);
//        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/02_start_return.mp3"));
//        player->play();
    }break;

    case connect_to_cloud_plat: //0x40
    {
        // mtimer->start(1000);//收到0x0a后发送心跳包
        login_timer->stop();
        mtimer->start(10000);
        timer_send_battery->start(11234);
        timer_send_floor_num->start(13456);
        timer_send_robot_running_status->start(17891);
        timer_send_current_car_point->start(1234);
        timer_send_fault->start(5000);
        qDebug() << "success to login!";
        // sendcommendwithdata(car_login_success_signal, roboid, empty_ba);
        //上电后设置初始位置为返航点位置，单机测试没有问题，不能多机，会重复调用此线程
        // emit sendsetreturnpossignal();
        if(!setstartstatue)
        {
            emit sendsetreturnpossignal();//上电后设置初始位置为返航点位置，单机测试没有问题，不能多机，会重复调用此线程
            setstartstatue = true;
        }
    }break;

    case car_login_success_signal: //0x45
    {
        // mtimer->start(1000);//收到0x0a后发送心跳包
        login_timer->stop();
        sendcommendwithdata(car_login_success_signal, roboid, empty_ba);
    }break;
//    case 0x02://巡航数据
//    {
//        qDebug()<<"收到0x02数据";
//        if(roboid!=100)
//        {
//            qDebug()<<"ID错误";
//            return;
//        }
//        if((uint8_t)r_data.at(0) == 0x01)
//        {
//            qDebug()<<"收到开始巡航数据";
//            qnode.set_datarequest(4);//开始巡航
//        }
//        else{
//            qDebug()<<"收到关闭巡航数据";
//            qnode.set_datarequest(5);//关闭巡航
//        }

//    }break;
//    case 0x0B:
//    {
//        qnode.set_datarequest(2);//发送开始返航命令
//    }break;
    case return_pos_signal: // 0x0D
    {
        handle_return_pos(r_data);
        sendcommendwithdata(return_pos_signal, roboid, empty_ba);
        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/04_set_return_point.mp3"));
        player->play();
    }break;
    case left_up_signal:    // 0x10
    {
        qnode.move_base(!use_all_statue?'U':'u',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"left_up_signal";
        sendcommendwithdata(left_up_signal, roboid, empty_ba);
    }break;
    case up_signal: // 0x11
    {
        qnode.move_base(!use_all_statue?'I':'i',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"up_signal";
        sendcommendwithdata(up_signal, roboid, empty_ba);
    }break;
    case right_up_signal:   // 0x12
    {
        qnode.move_base(!use_all_statue?'O':'o',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"right_up_signal";
        sendcommendwithdata(right_up_signal, roboid, empty_ba);
        break;
    }
    case left_signal:   // 0x13
    {
        qnode.move_base(!use_all_statue?'J':'j',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"left_signal";
        sendcommendwithdata(left_signal, roboid, empty_ba);
        break;
    }
    case stopmove_signal:   // 0x14
    {
        qnode.set_datarequest(8);//发送停止小车命令
        sendcommendwithdata(stopmove_signal, roboid, empty_ba);
        break;
    }
    case right_signal:   // 0x15
    {
        qnode.move_base(!use_all_statue?'L':'l',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"right_signal";
        sendcommendwithdata(right_signal, roboid, empty_ba);
        break;
    }
    case left_down_signal:   // 0x16
    {
        qnode.move_base(!use_all_statue?'M':'m',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"left_down_signal";
        sendcommendwithdata(left_down_signal, roboid, empty_ba);
        break;
    }
    case down_signal:
    {
        qnode.move_base(!use_all_statue?'<':',',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"down_signal";
        sendcommendwithdata(down_signal, roboid, empty_ba);
        break;
    }
    case right_down_signal:   // 0x18
    {
        qnode.move_base(!use_all_statue?'>':'.',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"right_down_signal";
        sendcommendwithdata(right_down_signal, roboid, empty_ba);
        break;
    }
    case checkBox_use_all_signal:   // 0x19
    {
        use_all_statue = QString(r_data).toInt();
        qDebug()<<"checkBox_use_all_signal: " << use_all_statue;
        sendcommendwithdata(checkBox_use_all_signal, roboid, empty_ba);
    }break;
    case raw_signal:    //0x1A
    {
        raw_speed = QString(r_data).toInt();
        qDebug()<<"raw_speed: " << raw_speed;
        sendcommendwithdata(raw_signal, robot_ID, empty_ba);
    }break;
    case line_signal:   //0x1B
    {
        line_speed = QString(r_data).toInt();
        qDebug()<<"line_speed: " << line_speed;
        sendcommendwithdata(line_signal, robot_ID, empty_ba);
    }break;
    case current_pos_signal: //0x07
    {
        handle_current_pos(r_data);
        sendcommendwithdata(current_pos_signal, robot_ID, empty_ba);
    }break;
    case goal_pos_signal:   //0X08
    {
        qnode.set_datarequest(8);//发送停止小车命令
        qnode.set_datarequest(8);//发送停止小车命令
        current_car_status_ = 1;
        timer_send_navi_point_signal->start(1000);
        car_goal_pos_ = r_data;
//        handle_goal_pos(car_goal_pos_);
//        sendcommendwithdata(goal_pos_signal, robot_ID, empty_ba);
//        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/01_set_goal_point.mp3"));
//        player->play();

    }break;
    case stopcar_signal: // 0x23
    {
        qnode.set_datarequest(8);//发送停止小车命令
        sendcommendwithdata(stopcar_signal, robot_ID, empty_ba);
        qDebug() << "stop car";
    }break;
    case multipos_signal:   //0x20
    {
        handle_multi_pos(r_data);
        sendcommendwithdata(multipos_signal, robot_ID, empty_ba);
        // hanglemulti_pos(r_data);
        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/05_set_multi_navigation_points.mp3"));
        player->play();
    }break;
    case startmulti_signal: //0x21
    {
        qnode.set_datarequest(8);//发送停止小车命令
        current_car_status_ = 3;
        multi_navi_loop_count = 0;
        timer_send_start_muti_navigation_signal->start(1000);
    }break;
    case pausemulti_signal: //0x22
    {
        qnode.set_datarequest(5);//发送停止巡航命令
        sendcommendwithdata(pausemulti_signal, robot_ID, empty_ba);
        qDebug() << "stop multi";
    }break;
    case floorcar_signal:  // 0x29
    {
        // int current_floor_num = floor_num.toInt();
        QByteArray current_floor_byteArray =  change_int_to_byteArray(floor_num.toInt());
        // sendcommendwithdata(floorcar_signal, robot_ID, floor_ba);  //current floor
        sendcommendwithdata(floorcar_signal, robot_ID, current_floor_byteArray);  //current floor
    }break;

    case app_changemap_signal:  // 0x36
    {
        handle_appchangemap(r_data);
        sendcommendwithdata(app_changemap_signal, robot_ID, empty_ba);
    }break;

    case switch_fan_display:  // 0x41
    {
        handle_switch_fan_display(r_data);
        sendcommendwithdata(switch_fan_display, robot_ID, empty_ba);
    }break;
    case get_total_floor_num:  // 0x42
    {
        QByteArray total_floor_byteArray =  change_int_to_byteArray(total_floor.toInt());
        // total floor
        sendcommendwithdata(get_total_floor_num, robot_ID, total_floor_byteArray);
    }break;

    case receive_new_audio:  // 0x43
    {
        handle_receive_new_audio(r_data);
        sendcommendwithdata(receive_new_audio, robot_ID, empty_ba);
    }break;

    case play_audio_signal:  // 0x44
    {
        handle_play_audio(r_data);
        sendcommendwithdata(play_audio_signal, robot_ID, empty_ba);
    }break;

    case play_video_signal:  // 0x46
    {
        handle_play_video(r_data);
        sendcommendwithdata(play_video_signal, robot_ID, empty_ba);
    }break;

    case add_elec_line:  // 0x48
    {
        handle_addlinedata(r_data);
        sendcommendwithdata(add_elec_line, robot_ID, empty_ba);
    }break;

    case set_currentPoint_to_returnPoint:  // 0x49
    {
        handle_currentPoint_to_returnPoint();
        sendcommendwithdata(set_currentPoint_to_returnPoint, robot_ID, empty_ba);
    }break;

    case multi_navi_play_video_switch:  // 0x50
    {
        handle_multi_navi_play_video_switch(r_data);
        sendcommendwithdata(multi_navi_play_video_switch, robot_ID, empty_ba);
    }break;



    default:
        break;
    }
}


uint16_t MainWindow::modbus_crc16(QByteArray pucFrame)//crc校验
{
    uint8_t ucCRCHi = 0xFF;
    uint8_t  ucCRCLo = 0xFF;
    int iIndex;
    int puc_count = pucFrame.count();
    for (int i=0;i<puc_count;i++)
    {
        iIndex = ucCRCLo ^ (uint8_t)pucFrame.at(i);
        ucCRCLo = ucCRCHi ^ aucCRCHi[iIndex];
        ucCRCHi = aucCRCLo[iIndex];
    }
    return (((uint16_t)ucCRCHi) << 8) | ucCRCLo;
}

void MainWindow::sendcommendwithdata(uint8_t int_commend,uint32_t robo_ID,QByteArray mdata)//发送命令+数据
{
    QByteArray mdata_senddata;
    QByteArray mdata_val;
    // int mdata_count = mdata.count() + 15;
    int mdata_count = mdata.count() + 13;
    uint8_t data_send[11];
    data_send[0] = 0x83;
    data_send[1] = 0x84;
    data_send[2] = int_commend;
    data_send[3] = mdata_count>>24;
    data_send[4] = mdata_count>>16;
    data_send[5] = mdata_count>>8;
    data_send[6] = mdata_count;
    data_send[7] = robo_ID>>24;
    data_send[8] = robo_ID>>16;
    data_send[9] = robo_ID>>8;
    data_send[10] = robo_ID;
    for (int j=0;j<11;j++)
    {
        mdata_val.push_back(data_send[j]);
    }
    mdata_senddata = mdata_val + mdata;
    uint16_t crc16 = modbus_crc16(mdata_senddata);
    uint8_t crc_high = (uint8_t)(crc16 >>8 &0x0FF);
    uint8_t crc_low = (uint8_t)(crc16 & 0x0FF);
    // mdata_senddata.push_back(crc_high);
    // mdata_senddata.push_back(crc_low);
    mdata_senddata.push_back(0xF4);
    mdata_senddata.push_back(0x4F);
    // add 0x7c 0x7c 0x7c as flag to Separate packets
    mdata_senddata.push_back(0x7c);
    mdata_senddata.push_back(0x7c);
    mdata_senddata.push_back(0x7c);
    car_client->receive_senddata(mdata_senddata);
}

void MainWindow::slot_flash()
{
    // send heart signal
    sendcommendwithdata(Connect_signal, robot_ID, empty_ba);
    // car_client->flush_tcp_send_buffer();
    // car_client->wait_for_bytes_written();
/*
    // send battery percent
    QByteArray battery_data = battery_str.toLatin1();
    sendcommendwithdata(0x01, robot_ID, battery_data);
    car_client->flush_tcp_send_buffer();
    car_client->wait_for_bytes_written();

    // send current floor number
    QByteArray current_floor_byteArray =  change_int_to_byteArray(floor_num.toInt());
    sendcommendwithdata(floorcar_signal, robot_ID, current_floor_byteArray);  //current floor
    car_client->flush_tcp_send_buffer();
    car_client->wait_for_bytes_written();

    // send current car point
    sendcommendwithdata(robopos_signal, robot_ID, RoboPose_data_cloud);
    car_client->flush_tcp_send_buffer();
    car_client->wait_for_bytes_written();

    // send robot running status
    QByteArray statue_data;
    statue_data.clear();
    if(ros::master::check())
        statue_data.push_back(0x31);  //char "1"
    else
        statue_data.push_back(0x32);  //char "2"
    sendcommendwithdata(0x02, robot_ID, statue_data);
    car_client->flush_tcp_send_buffer();
    car_client->wait_for_bytes_written();
    */
}

void MainWindow::slot_loginflash()
{
//    QByteArray login_info;
//    login_info.clear();
//    login_info.push_back(0x01);
//    login_info.push_back(0x01);
//    login_info.push_back(0x01);
//    sendcommendwithdata(0x0A,100,login_info);
    sendcommendwithdata(connect_to_cloud_plat, robot_ID, empty_ba);
    qDebug() << "wait to login...";
}

//app相关
void MainWindow::app_recvData(QString ip, int port,int sockDesc,QByteArray datagram)
{
    qDebug()<<datagram;
    if(!currency_data_sur.isEmpty())
    {
        datagram = currency_data_sur + datagram;
        if(((uint8_t)datagram.at(0)!=0x83)||((uint8_t)datagram.at(1)!=0x84))//包头
        {
            datagram.clear();
            currency_data_sur.clear();
            return;
        }
    }
    else
    {
        if(((uint8_t)datagram.at(0)!=0x83)||((uint8_t)datagram.at(1)!=0x84))//包头
        {
            datagram.clear();
            return;
        }
    }
    int datagarm_count = datagram.count();
    while (datagarm_count) //等待处理完这一帧数据
    {
        if(datagarm_count<7)//剩余数据读不到长度
        {
            currency_data_sur = datagram;//将剩余数据存下，下次继续解析
            return;
        }
        if((uint8_t)datagram.at(0)==0x83)//包头
        {
            if((uint8_t)datagram.at(1)==0x84)//包头
            {
                uint8_t datagram_at6 = (uint8_t)datagram.at(6);//现将二进制数据流转为uint_8型，否则数据转32位会出错
                uint8_t datagram_at5 = (uint8_t)datagram.at(5);
                uint8_t datagram_at4 = (uint8_t)datagram.at(4);
                uint8_t datagram_at3 = (uint8_t)datagram.at(3);
                currency_m_count = ((uint32_t)datagram_at3<<24)+((uint32_t)datagram_at4<<16)+((uint32_t)datagram_at5<<8)+(uint32_t)datagram_at6;//得到发送的数据包的大小
                if(datagarm_count<currency_m_count)//剩余数据小于包数据长度
                {
                    currency_data_sur = datagram;//将剩余数据存下，下次继续解析
                    return;
                }
                currency_m_commend = (uint8_t)datagram.at(2);//取出命令
                if(((uint8_t)datagram.at(currency_m_count-2)!=0xF4)||((uint8_t)datagram.at(currency_m_count-1)!=0x4F))//判断包尾
                {
                    qDebug()<<"receive error!!!";
                    currency_datahandle.clear();//清理处理完的数据，释放内存
                    currency_data_sur.clear();
                    currency_m_count=0;
                    currency_m_commend = 0xFF;
                    return;//接收数据错误，没有接收到包尾
                }
                uint8_t datagram_at10 = (uint8_t)datagram.at(10);//现将二进制数据流转为uint_8型，否则数据转32位会出错
                uint8_t datagram_at9 = (uint8_t)datagram.at(9);
                uint8_t datagram_at8 = (uint8_t)datagram.at(8);
                uint8_t datagram_at7 = (uint8_t)datagram.at(7);
                smarcorobot_ID = ((uint32_t)datagram_at7<<24)+((uint32_t)datagram_at8<<16)+((uint32_t)datagram_at9<<8)+(uint32_t)datagram_at10;//得到机器人ID
                currency_datahandle = datagram;
                currency_datahandle.remove(currency_m_count-1,datagarm_count-currency_m_count);//先去除尾部本次不处理数据
                currency_datahandle.remove(currency_m_count-2,2);
                currency_datahandle.remove(0,11);
                app_handledata(sockDesc,currency_m_commend,currency_datahandle);//处理本次取出的数据
                currency_m_commend = 0xFF;//数据处理完将命令还原
                datagram.remove(0,currency_m_count);//去除数据链中处理完的数据
                datagarm_count = datagarm_count - currency_m_count;//记录剩下未处理的数据长度
                currency_datahandle.clear();//清理处理完的数据，释放内存
                currency_data_sur.clear();
            }
        }
    }
}
void MainWindow::app_showConnection(int sockDesc)
{

}

void MainWindow::app_showDisconnection(int sockDesc)
{

}

void MainWindow::app_slot_recesockDesc(QList<int> socklist)
{
    app_socklist = socklist;
}

void MainWindow::app_handledata(int sockDesc, uint8_t commend, QByteArray r_data)
{
    switch (commend)
    {
    case 0x51:
    {
        handle_volice_quiet(r_data);
        qDebug() << "FUCK MOTHER!!!!!!!!!!!!!!!!!!!!;";
    }break;
    case 0x00:
    {
        app_sendscok_com_data(sockDesc,Connect_signal,smarcorobot_ID,empty_ba);
    }break;
    case battery_signal://发送电压数据
    {
        QByteArray battery_data;
        battery_data = battery_str.toLatin1();
        app_sendscok_com_data(sockDesc,battery_signal,smarcorobot_ID,battery_data);
    }break;
    case 0x02:
    {
        QByteArray statue_data;
        statue_data.clear();
        QString statue_str;
        if(ros::master::check())
        {
            statue_str = "1";
            statue_data.push_back(statue_str.toLatin1());
        }
        else
        {
            statue_str = "2";
            statue_data.push_back(statue_str.toLatin1());
        }
        // sendscok_com_data(sockDesc,0x02,statue_data);
        app_sendscok_com_data(sockDesc,robotstatue_signal,smarcorobot_ID,statue_data);
    }break;
    case left_up_signal:
    {
        qnode.move_base(!use_all_statue?'U':'u',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"left_up_signal";
        break;
    }
    case up_signal:
    {
        qnode.move_base(!use_all_statue?'I':'i',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"up_signal";
        break;
    }
    case right_up_signal:
    {
        qnode.move_base(!use_all_statue?'O':'o',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"right_up_signal";
        break;
    }
    case left_signal:
    {
        qnode.move_base(!use_all_statue?'J':'j',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"left_signal";
        break;
    }
    case stopmove_signal:
    {
        qnode.set_datarequest(8);//发送停止小车命令
        break;
    }
    case right_signal:
    {
        qnode.move_base(!use_all_statue?'L':'l',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"right_signal";
        break;
    }
    case left_down_signal:
    {
        qnode.move_base(!use_all_statue?'M':'m',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"left_down_signal";
        break;
    }
    case down_signal:
    {
        qnode.move_base(!use_all_statue?'<':',',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"down_signal";
        break;
    }
    case right_down_signal:
    {
        qnode.move_base(!use_all_statue?'>':'.',line_speed*0.01,raw_speed*0.01);
        qDebug()<<"right_down_signal";
        break;
    }
    case checkBox_use_all_signal:
    {
        use_all_statue = QString(r_data).toInt();
        qDebug()<<"checkBox_use_all_signal";
    }break;
    case raw_signal:
    {
        raw_speed = QString(r_data).toInt();
    }break;
    case line_signal:
    {
        line_speed = QString(r_data).toInt();
    }break;
    case goal_pos_signal:
    {
        handle_goalpos(r_data);
    }break;
    case open_express_signal:
    {
        handle_openexpress(r_data);
    }break;
    case express_statue_signal:
    {
        handle_expressstatue(sockDesc,r_data);
    }break;
    case openexpress_statue_signal:
    {
        handle_openexpressvoice(r_data);
    }break;
    case app_changemap_signal:
    {
        handle_appchangemap(r_data);
    }break;
    case app_infoinit_signal:
    {
        handle_appinfoinit(sockDesc,r_data);
    }break;
    case startreturn_signal:
    {
        app_carrunstatue = 0;
        qnode.set_datarequest(2);//发送开始返航命令
    }break;
    default:
        break;
    }
}



// 发送描述符+命令+数据
void MainWindow::app_sendscok_com_data(int sockDesc, uint8_t int_commend,uint32_t robo_ID,QByteArray mdata)
{
    QByteArray mdata_senddata;
    QByteArray mdata_val;
    int mdata_count = mdata.count() + 13;
    uint8_t data_send[mdata_count];
    data_send[0] = 0x83;
    data_send[1] = 0x84;
    data_send[2] = int_commend;
    data_send[3] = mdata_count>>24;
    data_send[4] = mdata_count>>16;
    data_send[5] = mdata_count>>8;
    data_send[6] = mdata_count;
    data_send[7] = robo_ID>>24;
    data_send[8] = robo_ID>>16;
    data_send[9] = robo_ID>>8;
    data_send[10] = robo_ID;
    for (int j=0;j<11;j++)
    {
        mdata_val.push_back(data_send[j]);
    }
    mdata_senddata = mdata_val + mdata;
    mdata_senddata.push_back(0xF4);
    mdata_senddata.push_back(0x4F);
    emit app_sendData(sockDesc,mdata_senddata);
}

QPolygonF MainWindow::rdatatopolygen(QByteArray r_data)
{
    qDebug() << r_data.toHex();
    QPolygonF poly;
    uint8_t datagram_at3 = (uint8_t)r_data.at(3);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t datagram_at2 = (uint8_t)r_data.at(2);
    uint8_t datagram_at1 = (uint8_t)r_data.at(1);
    uint8_t datagram_at0 = (uint8_t)r_data.at(0);
    int start_x = ((uint32_t)datagram_at0<<24)+((uint32_t)datagram_at1<<16)+((uint32_t)datagram_at2<<8)+(uint32_t)datagram_at3;
    uint8_t datagram_at7 = (uint8_t)r_data.at(7);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t datagram_at6 = (uint8_t)r_data.at(6);
    uint8_t datagram_at5 = (uint8_t)r_data.at(5);
    uint8_t datagram_at4 = (uint8_t)r_data.at(4);
    int start_y = ((uint32_t)datagram_at4<<24)+((uint32_t)datagram_at5<<16)+((uint32_t)datagram_at6<<8)+(uint32_t)datagram_at7;
    uint8_t datagram_at11 = (uint8_t)r_data.at(11);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t datagram_at10 = (uint8_t)r_data.at(10);
    uint8_t datagram_at9 = (uint8_t)r_data.at(9);
    uint8_t datagram_at8 = (uint8_t)r_data.at(8);
    int last_x = ((uint32_t)datagram_at8<<24)+((uint32_t)datagram_at9<<16)+((uint32_t)datagram_at10<<8)+(uint32_t)datagram_at11;
    uint8_t datagram_at15 = (uint8_t)r_data.at(15);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t datagram_at14 = (uint8_t)r_data.at(14);
    uint8_t datagram_at13 = (uint8_t)r_data.at(13);
    uint8_t datagram_at12 = (uint8_t)r_data.at(12);
    int last_y = ((uint32_t)datagram_at12<<24)+((uint32_t)datagram_at13<<16)+((uint32_t)datagram_at14<<8)+(uint32_t)datagram_at15;
    poly.clear();
    poly.push_back(QPointF(start_x/10000.0,start_y/10000.0));
    poly.push_back(QPointF(last_x/10000.0,last_y/10000.0));
    return poly;
}

void MainWindow::handle_goalpos(QByteArray r_data)
{
    QPolygonF goal_pos = rdatatopolygen(r_data);
    qDebug()<<goal_pos;
    QPointF startgoalpos = goal_pos.at(0);
    QPointF lastgoalpos = goal_pos.at(1);
    geometry_msgs::Quaternion goal_pos_quat;
    double tan_radian = (lastgoalpos.y()-startgoalpos.y())/(lastgoalpos.x()-startgoalpos.x());
    double radian = atan(tan_radian);
    if(lastgoalpos.x()>=startgoalpos.x())
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastgoalpos.x()<startgoalpos.x()) &&  (lastgoalpos.y()>=startgoalpos.y()))
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastgoalpos.x()<startgoalpos.x()) &&  (lastgoalpos.y()<startgoalpos.y()))
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    app_carrunstatue = 0;
    qnode.set_goal("map",startgoalpos.x(),startgoalpos.y(),goal_pos_quat.z,goal_pos_quat.w);
}

void MainWindow::handle_expressstatue(int sockDesc,QByteArray r_data)
{
    QByteArray expressstatue_data;
    expressstatue_data.clear();
    uint8_t key = r_data.at(0);
    uint8_t value = express_statue.find(key).value();
    expressstatue_data.push_back(key);
    expressstatue_data.push_back(value);
    app_sendscok_com_data(sockDesc,express_statue_signal,smarcorobot_ID,expressstatue_data);
}

void MainWindow::handle_openexpress(QByteArray r_data)
{
    qDebug()<<"handle_openexpress:::"<<r_data;
    int express_id = r_data.at(0);
    qDebug()<<"express_id:::"<<express_id;
    qnode.open_express(express_id);
}

void MainWindow::handle_openexpressvoice(QByteArray r_data)
{ 
    player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/catkin_cg/src/ros_qt_demo-master/resources/voice/hello_openexpress.mp3"));
    qDebug()<<"handle_openexpressvoiceaa";
    //    emit signal_play();
    player->play();
}

void MainWindow::handle_appchangemap(QByteArray r_data)
{
    changemap_statue = true;
    floor_ba.clear();
    floor_ba = r_data;
    switch (r_data.at(0))
    {
    case 0x00:
    {
        qnode.sendfloornum(0);
        floor_num = "0";
    }break;
    case 0x01:
    {
        qnode.sendfloornum(1);
        floor_num = "1";
    }break;
    case 0x02:
    {
        qnode.sendfloornum(2);
        floor_num = "2";
    }break;
    case 0x03:
    {
        qnode.sendfloornum(3);
        floor_num = "3";
    }break;
    case 0x04:
    {
        qnode.sendfloornum(4);
        floor_num = "4";
    }break;
    case 0x05:
    {
        qnode.sendfloornum(5);
        floor_num = "5";
    }break;
    case 0x06:
    {
        qnode.sendfloornum(6);
        floor_num = "6";
    }break;
    case 0x07:
    {
        qnode.sendfloornum(7);
        floor_num = "7";
    }break;
    case 0x08:
    {
        qnode.sendfloornum(8);
        floor_num = "8";
    }break;
    case 0x09:
    {
        qnode.sendfloornum(9);
        floor_num = "9";
    }break;
    case 0x0A:
    {
        qnode.sendfloornum(10);
    }break;
    default:
        break;
    }
}

void MainWindow::handle_appinfoinit(int sockDesc,QByteArray r_data)
{
    for(int i=1;i<express_statue.lastKey()+1;i++)
    {
        QByteArray expressstatue_data;
        expressstatue_data.clear();
        uint8_t key = i;
        uint8_t value = express_statue.value(i);
        expressstatue_data.push_back(key);
        expressstatue_data.push_back(value);
        app_sendscok_com_data(sockDesc,express_statue_signal,smarcorobot_ID,expressstatue_data);
    }
}

void MainWindow::handle_current_pos(QByteArray r_data)
{
    //QPolygonF goal_pos = rdatatopolygen(r_data);
    // qDebug()<<goal_pos;
    QPolygonF current_pos = rdatatopolygen(r_data);
    qDebug() << current_pos;
//    QDataStream current_pos_ds(&r_data,QIODevice::ReadWrite);
//    current_pos_ds>>current_pos;
    QPointF startcurrentpos = current_pos.at(0);
    QPointF lastcurrentpos = current_pos.at(1);
    geometry_msgs::Quaternion currentpos_quat;
    double tan_radian = (lastcurrentpos.y()-startcurrentpos.y())/(lastcurrentpos.x()-startcurrentpos.x());
    double radian = atan(tan_radian);
    if(lastcurrentpos.x()>=startcurrentpos.x())
    {
        currentpos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastcurrentpos.x()<startcurrentpos.x()) &&  (lastcurrentpos.y()>=startcurrentpos.y()))
    {
        currentpos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastcurrentpos.x()<startcurrentpos.x()) &&  (lastcurrentpos.y()<startcurrentpos.y()))
    {
        currentpos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    qnode.set_start("map",startcurrentpos.x(),startcurrentpos.y(),currentpos_quat.z,currentpos_quat.w);
}

void MainWindow::handle_goal_pos(QByteArray r_data)
{
    // qDebug()<<"r_data::::"<<r_data;
//    QPolygonF goal_pos;
//    QDataStream goal_pos_ds(&r_data,QIODevice::ReadWrite);
//    goal_pos_ds>>goal_pos;
//    qDebug()<<"goal_pos:::"<<goal_pos;
    QPolygonF goal_pos = rdatatopolygen(r_data);
    qDebug() << goal_pos;
    QPointF startgoalpos = goal_pos.at(0);
    QPointF lastgoalpos = goal_pos.at(1);
    geometry_msgs::Quaternion goal_pos_quat;
    double tan_radian = (lastgoalpos.y()-startgoalpos.y())/(lastgoalpos.x()-startgoalpos.x());
    double radian = atan(tan_radian);
    if(lastgoalpos.x()>=startgoalpos.x())
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastgoalpos.x()<startgoalpos.x()) &&  (lastgoalpos.y()>=startgoalpos.y()))
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastgoalpos.x()<startgoalpos.x()) &&  (lastgoalpos.y()<startgoalpos.y()))
    {
        goal_pos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    qnode.set_goal("map",startgoalpos.x(),startgoalpos.y(),goal_pos_quat.z,goal_pos_quat.w);
    if(autoback_statue==1)
    {
        goal_statue = 1;//单点导航标志置为一
    }
}

void MainWindow::handle_return_pos(QByteArray r_data)
{
    // QPolygonF return_pos;
    // QDataStream return_pos_ds(&r_data,QIODevice::ReadWrite);
    // return_pos_ds>>return_pos;
    // QPointF start_return_pso = QPointF(20, 20);
    QPolygonF return_pos = rdatatopolygen(r_data);
    qDebug() << return_pos;
    QPointF startreturnpos = return_pos.at(0);
    QPointF lastreturnpos = return_pos.at(1);
    geometry_msgs::Quaternion return_pos_quat;
    double tan_radian = (lastreturnpos.y()-startreturnpos.y())/(lastreturnpos.x()-startreturnpos.x());
    double radian = atan(tan_radian);
    if(lastreturnpos.x()>=startreturnpos.x())
    {
        return_pos_quat = tf::createQuaternionMsgFromYaw(radian);
    }
    else if((lastreturnpos.x()<startreturnpos.x()) &&  (lastreturnpos.y()>=startreturnpos.y()))
    {
        return_pos_quat = tf::createQuaternionMsgFromYaw(radian + 3.1415);
    }
    else if((lastreturnpos.x()<startreturnpos.x()) &&  (lastreturnpos.y()<startreturnpos.y()))
    {
        return_pos_quat = tf::createQuaternionMsgFromYaw(radian - 3.1415);
    }
    return_posx = startreturnpos.x();
    return_posy = startreturnpos.y();
    return_rawz = return_pos_quat.z;
    return_raww = return_pos_quat.w;
    //剩下向小车发送返航点消息没写
    qnode.sendreturn_pos(return_posx,return_posy,return_rawz,return_raww);
}

void MainWindow::handle_multi_pos(QByteArray r_data)
{
    geometry_msgs::Pose pose;
    geometry_msgs::PoseArray pose_list;

    qDebug() << r_data.toHex();
    uint32_t count;
    uint8_t datagram_at3 = (uint8_t)r_data.at(3);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t datagram_at2 = (uint8_t)r_data.at(2);
    uint8_t datagram_at1 = (uint8_t)r_data.at(1);
    uint8_t datagram_at0 = (uint8_t)r_data.at(0);
    count = ((uint32_t)datagram_at0<<24)+((uint32_t)datagram_at1<<16)+((uint32_t)datagram_at2<<8)+(uint32_t)datagram_at3;
    if(count <2)
    {
        qDebug() << "multi navigation pos less than 2";
        return;
    }
    r_data.remove(0, 4);
    for(uint32_t i = 0; i < count; i++)
    {
        QPolygonF multi_pos = rdatatopolygen(r_data);
        qDebug() << i <<" point:" << multi_pos;
        QPointF startmultipos = multi_pos.at(0);
        QPointF lastmultipos = multi_pos.at(1);
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
        r_data.remove(0, 16);
    }

//    QVector<QPolygonF> qvmulti_pos;
//    QPolygonF multi_pos;
//    QDataStream current_pos_ds(&r_data,QIODevice::ReadWrite);
//    current_pos_ds>>qvmulti_pos;
      // qnode.handlemultiposlist(qvmulti_pos);
      // multipos_pub.publish(pose_list);
    qnode.handle_multipos_list(pose_list);
}

void MainWindow::handle_addlinedata(QByteArray r_data)
{
    geometry_msgs::Polygon point_polygen;
    geometry_msgs::Point32 point_32;

    qDebug() << r_data.toHex();
    uint32_t count;
    uint8_t datagram_at3 = (uint8_t)r_data.at(3);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t datagram_at2 = (uint8_t)r_data.at(2);
    uint8_t datagram_at1 = (uint8_t)r_data.at(1);
    uint8_t datagram_at0 = (uint8_t)r_data.at(0);
    count = ((uint32_t)datagram_at0<<24)+((uint32_t)datagram_at1<<16)+((uint32_t)datagram_at2<<8)+(uint32_t)datagram_at3;
    qDebug() << "get " << count << " elec_line point pairs.";
    r_data.remove(0, 4);
    for(uint32_t i = 0; i < count; i++)
    {
        QPolygonF multi_pos = rdatatopolygen(r_data);
        qDebug() << i <<" point:" << multi_pos;
        QPointF startmultipos = multi_pos.at(0);
        QPointF lastmultipos = multi_pos.at(1);

        point_32.x = startmultipos.x();
        point_32.y = startmultipos.y();
        point_32.z = 0.0;
        point_polygen.points.push_back(point_32);
        point_32.x = lastmultipos.x();
        point_32.y = lastmultipos.y();
        point_32.z = 0.0;
        point_polygen.points.push_back(point_32);
        r_data.remove(0, 16);
    }
    qnode.send_elec_line(point_polygen);
    // electronicfenceclient.publish(point_polygen);
}

void MainWindow::handle_switch_fan_display(QByteArray r_data)
{
    switch (r_data.at(0))
    {
    case 0x00:
    {
        // close display
        fan_display_status = false;
        qnode.handle_fanDisplay(0);
        break;
    }
    case 0x01:
    {
        // open display
        fan_display_status = true;
        qnode.handle_fanDisplay(1);
        break;
    }
    }
}

void MainWindow::handle_send_current_floor(QByteArray r_data)
{

}

void MainWindow::handle_play_audio(QByteArray r_data)
{
    switch (r_data.at(0))
    {
    case 0x00:
    {
        player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/00_hello.mp3"));
        player->play();
        break;
    }
    default:break;
    }
    return;

}

void MainWindow::handle_receive_new_audio(QByteArray r_data)
{
    // filename
    uint8_t r_data_at3 = (uint8_t)r_data.at(3);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    uint8_t r_data_at2 = (uint8_t)r_data.at(2);
    uint8_t r_data_at1 = (uint8_t)r_data.at(1);
    uint8_t r_data_at0 = (uint8_t)r_data.at(0);
    uint32_t file_name_length = ((uint32_t)r_data_at0<<24)+((uint32_t)r_data_at1<<16)+((uint32_t)r_data_at2<<8)+(uint32_t)r_data_at3;
    r_data.remove(0, 4);
    QByteArray file_name_array;
    for(int i = 0; i < file_name_length; i++)
    {
        file_name_array.append(r_data.at(i));
    }
    QString file_name_str = file_name_array;
    qDebug() << "file_name_str: " << file_name_str;
    r_data.remove(0, file_name_length);

    // url
    r_data_at3 = (uint8_t)r_data.at(3);//现将二进制数据流转为uint_8型，否则数据转32位会出错
    r_data_at2 = (uint8_t)r_data.at(2);
    r_data_at1 = (uint8_t)r_data.at(1);
    r_data_at0 = (uint8_t)r_data.at(0);
    uint32_t url_length = ((uint32_t)r_data_at0<<24)+((uint32_t)r_data_at1<<16)+((uint32_t)r_data_at2<<8)+(uint32_t)r_data_at3;
    r_data.remove(0, 4);
    QByteArray url_array;
    for(int i = 0; i < url_length; i++)
    {
        url_array.append(r_data.at(i));
    }
    QString url_str = url_array;
    qDebug() << "url_str: " << url_str;
    r_data.remove(0, url_length);
    QUrl url_str_ = url_str;
    // [5]发送http请求
    //startRequest( QUrl("https://hitc.org.cn:30001/file/robot/upload/2023-04-20/mp3/84F39796-02E7-46F1-B079-3BDD97C74671.mp3"));
    startRequest(url_str_);

  //  thread_http_transmit = new thread_http(url_str_);
//    thread_http_transmit->start();

//    QFile file2("/home/zhou/" + file_name_str);
//    qDebug() << "/home/zhou/" + file_name_str;
//    if(file2.open(QIODevice::ReadWrite) == true)
//    {
//        /*常见数据流和file文件关联
//             * 往数据流输出数据=文件读数据
//            */
//        // QDataStream stream(&file);
//        /*读的时候按写的顺序取数据*/
//        file2.write(r_data);
//        qDebug() << r_data.size() << "file2 success";
//        file2.close();
//    }
}

void MainWindow::handle_play_video(QByteArray r_data)
{
    switch (r_data.at(0))
    {
    case 0x00:
    {
        // close display
        close_play_video_thread();
        break;
    }
    case 0x01:
    {
        // open display
        open_play_video_thread();
        break;
    }
    }

}

void MainWindow::open_play_video_thread()
{
    qDebug() << "主线程id：" << QThread::currentThreadId();
    thread_play_video->start(); // 启动子线程
    qDebug() << "start thread_play_video";
}

void MainWindow::close_play_video_thread()
{
    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
}

QByteArray MainWindow::change_int_to_byteArray(int num)
{
//    RoboPose_x_cloud = point.x();
//    RoboPose_y_cloud = point.y();
//    RoboPose_raw_cloud = raw;
    // RoboPose_data_cloud.clear();
    //qint32 int32_RoboPose_x = static_cast<qint32>(RoboPose_x_cloud * 10000);
    QByteArray byte_array_;
    QString str_num = QString("%1").arg(num, 8, 16, QLatin1Char('0'));
    byte_array_[0] = str_num.mid(0, 2).toInt(nullptr, 16);
    byte_array_[1] = str_num.mid(2, 2).toInt(nullptr, 16);
    byte_array_[2] = str_num.mid(4, 2).toInt(nullptr, 16);
    byte_array_[3] = str_num.mid(6, 2).toInt(nullptr, 16);
    return byte_array_;
}

void MainWindow::handle_multi_navi_play_video_switch(QByteArray r_data)
{
    switch (r_data.at(0))
    {
    case 0x00:
    {
        // close multi navi play
        multi_navi_play_video_flag = 0;
    }break;
    case 0x01:
    {
        // open multi navi play
        multi_navi_play_video_flag = 1;
        qDebug() << "multi_navi_play_video_flag = " << multi_navi_play_video_flag;
    }break;
    default:
        break;
    }
}

void MainWindow::handle_volice_quiet(QByteArray r_data)
{
    switch (r_data.at(0))
    {
    case 0x00:
    {
        QProcess *process_voice = new QProcess();
        process_voice->start("amixer set -c 0 Master 0");
        // qDebug() << front + file_url + end;
        process_voice->waitForFinished();
    }break;
    case 0x01:
    {
        QProcess *process_voice = new QProcess();
        process_voice->start("amixer set -c 0 Master 100");
        // qDebug() << front + file_url + end;
        process_voice->waitForFinished();
    }break;
    default:
        break;
    }
}

}  // namespace class1_ros_qt_demo


void class1_ros_qt_demo::MainWindow::on_pushButton_play_music_clicked()
{
    // static bool flag_music = true;
    // music_player = new QMediaPlayer;
    // connect(player, SIGNAL(positionChanged(qint64)), this, SLOT(positionChanged(qint64)));
    // music_player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/1.mp3"));
    // player->setMedia(QUrl::fromLocalFile("/home/zhou/1.mp3"));
    //music_player->setMedia(QUrl::fromLocalFile("/home/smarcorobot/video/06_music.mp3"));
    // music_player->setVolume(70);

        music_player->play();

      qDebug() << "play 1.mp3";
//    QByteArray array_;
//        QFile file("/home/zhou/1.mp3");
//        if(file.open(QIODevice::ReadWrite) == true)
//        {
//            /*常见数据流和file文件关联
//             * 往数据流输出数据=文件读数据
//            */
//            // QDataStream stream(&file);
//            /*读的时候按写的顺序取数据*/
//            array_ = file.readAll();
//            qDebug() << array_.size() << "success";
//            sendcommendwithdata(receive_new_audio, robot_ID, array_);
//            qDebug() << array_.toHex() << "success";
//            file.close();
//        }
//        QFile file2("/home/zhou/2.mp3");
//        if(file2.open(QIODevice::ReadWrite) == true)
//        {
//            /*常见数据流和file文件关联
//             * 往数据流输出数据=文件读数据
//            */
//            // QDataStream stream(&file);
//            /*读的时候按写的顺序取数据*/
//            file2.write(array_);
//            qDebug() << array_.size() << "file2 success";
//            file2.close();
//        }

}

void class1_ros_qt_demo::MainWindow::on_pushButton_stop_music_clicked()
{
    music_player->stop();
}

void class1_ros_qt_demo::MainWindow::on_pushButton_stop_video_clicked()
{
    qDebug() << "主线程id：" << QThread::currentThreadId();
    thread_play_video->start(); // 启动子线程
    qDebug() << "success to exec thread.";
}

void class1_ros_qt_demo::MainWindow::on_pushButton_quit_video_clicked()
{
    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
    qDebug() << "quit thread run()";
    // thread_play_video->deleteLater();
}

void class1_ros_qt_demo::MainWindow::on_pushButton_set_video_url_clicked()
{
    thread_play_video->file_url = ui.lineEdit_video_url->text();
    qDebug() << "file_url: " << thread_play_video->file_url;
}

void class1_ros_qt_demo::MainWindow::slot_timer_test_mp4_play()
{
    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
    qDebug() << "quit thread run()";

    static int count_mp4_num = 0;
    switch (count_mp4_num % 3) {
    case 0:{
        thread_play_video->file_url = "/home/zhou/video/01_test.mp4";
        qDebug() << "主线程id：" << QThread::currentThreadId();
        thread_play_video->start(); // 启动子线程
        qDebug() << "success to exec thread.";
    }break;
    case 1:{
        thread_play_video->file_url = "/home/zhou/video/02_test.mp4";
        qDebug() << "主线程id：" << QThread::currentThreadId();
        thread_play_video->start(); // 启动子线程
        qDebug() << "success to exec thread.";
    }break;
    case 2:{
        thread_play_video->file_url = "/home/zhou/video/03_test.mp4";
        qDebug() << "主线程id：" << QThread::currentThreadId();
        thread_play_video->start(); // 启动子线程
        qDebug() << "success to exec thread.";
    }break;
    default:
        break;
    }
    count_mp4_num++;
}


//void class1_ros_qt_demo::MainWindow::on_pushButton_clicked()
//{
//    timer_test_mp4_play->start(10000);
//}

void class1_ros_qt_demo::MainWindow::on_pushButton_test_switch_mp4_play_stop_clicked()
{
    timer_test_mp4_play->stop();
    if(thread_play_video->mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -15 ";
        QString str_process_ID = QString::number(thread_play_video->mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    thread_play_video->mplayer_process_ID = 0;
    thread_play_video->quit();
    thread_play_video->wait(); // 等待子线程退出，然后回收资源
    qDebug() << "quit thread run()";

}

void class1_ros_qt_demo::MainWindow::on_pushButton_test_switch_mp4_play_clicked()
{
    timer_test_mp4_play->start(10000);
}

// [6]发起HTTP请求
/*
 * 功能描述：发送HTTP请求，并与请求响应的槽绑定
 *  @param requestedUrl：请求需要的URL地址
*/
void class1_ros_qt_demo::MainWindow::startRequest(const QUrl &requestedUrl)
{
    url = requestedUrl;
    manager = new QNetworkAccessManager(this);

    reply = manager->get(QNetworkRequest(url));
    connect(reply,&QNetworkReply::finished,this,&MainWindow::replyFinished);
    connect(reply,&QNetworkReply::readyRead,this,&MainWindow::reply_readyRead);
    connect(reply,&QNetworkReply::downloadProgress,this,&MainWindow::reply_downloadProgress);

}

/*
 * 功能描述：HTTP请求后，接收服务器的请求信息
 * 1 检测请求响应是否有错误
 * 2 获取请求响应的状态码
 * 3 判断是否需要重定向
 *   - 不需要，则保存数据
 *   - 需要重定向，则获取重定向的URL，然后通过这个URL再次发起请求
*/
void class1_ros_qt_demo::MainWindow::replyFinished()
{

    // <1>判断有没有错误
    if (reply->error()){
        qDebug()<<reply->errorString();
        reply->deleteLater();
        return;
    }

    // <2>检测状态码
    int statusCode  = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    qDebug() << "statusCode:" << statusCode;

    // <3>判断是否需要重定向
    if (statusCode >= 200 && statusCode <300){

        QString file_path = "/home/zhou/Music";
        QString current_date_time = QDateTime::currentDateTime().toString("yyyy_MM_dd hh_mm_ss");
        QString file_name = file_path
                            + "/"
                            + current_date_time
                            + "_test.mp3";

        downFile = new QFile(file_name);
        if(!downFile->open(QIODevice::WriteOnly))
            return;

        downFile->write(reply->readAll());

        // <1> close file
        downFile->close();

        // 数据读取完成之后，清除reply
        reply->deleteLater();
        reply = nullptr;
    }

}

void class1_ros_qt_demo::MainWindow::reply_readyRead()
{

}

void class1_ros_qt_demo::MainWindow::reply_downloadProgress(qint64 bytesRead, qint64 bytesTotal)
{
    ui.progressBar->setMaximum(bytesTotal);
    ui.progressBar->setValue(bytesRead);
}


