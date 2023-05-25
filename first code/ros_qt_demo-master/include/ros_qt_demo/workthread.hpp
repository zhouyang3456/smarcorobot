#ifndef WORKTHREAD_H
#define WORKTHREAD_H

#include <QObject>
#include <QThread>
#include <QProcess>
#include <QDebug>

class workThread : public QThread
{
    Q_OBJECT
public:
    workThread();
    ~workThread();
    void run() override;
    int mplayer_process_ID = 0;
    QString file_url = "/home/smarcorobot/video/01_test.mp4";
    // QString file_url;
};

#endif // WORKTHREAD_H
