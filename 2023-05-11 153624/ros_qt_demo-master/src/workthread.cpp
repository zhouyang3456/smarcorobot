#include "../include/ros_qt_demo/workthread.hpp"

workThread::workThread()
{
    // 当监听到线程结束时（finished），就调用deleteLater回收内存
//    connect(this,&workThread::finished,this,[=](){
//        this->deleteLater();
//        qDebug()<<"线程结束";
//    });
}

workThread::~workThread()
{
    if(mplayer_process_ID != 0){
        QProcess *process_kill = new QProcess();
        QString str_command = "kill -9 ";
        QString str_process_ID = QString::number(mplayer_process_ID);
        process_kill->start(str_command + str_process_ID);
        process_kill->waitForFinished();
        qDebug() << "kill mplayer success";
    }
    mplayer_process_ID = 0;
//    requestInterruption();
//    wait();
    qDebug()<<"析构函数";
}

void workThread::run()
{
    qDebug() << "当前子线程ID:" << QThread::currentThreadId();
    QProcess *process = new QProcess();
    QString front = "mplayer -fs -loop -3 ";
    QString end = " -fixed-vo";
    process->start(front + file_url + end);
    qDebug() << front + file_url + end;

    mplayer_process_ID = process->processId();
    qDebug() << "mplayer_process_ID: " << mplayer_process_ID;
    process->waitForFinished(-1);
    //this->exec();
    mplayer_process_ID = 0;
    qDebug() << "线程结束, mplayer_process_ID = " << mplayer_process_ID;
}
