#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H

#include <QThread>
#include <QObject>
#include <QtNetwork>
#include <QDebug>

#include "mysocket.h"

class Socket;

class serverThread : public QThread
{
    Q_OBJECT
public:
    serverThread(int sockDesc, QObject *parent = nullptr);
    ~serverThread();

private:
    void run(void);

public slots:
    void sendDataSlot(int sockDesc, const QByteArray &data);

signals:
    void dataReady(QString ip,int port,int sockDesc,QByteArray data);
    void sendData(int sockDesc,QByteArray data);
    void disconnectTCP(int sockDesc);

public slots:
    void recvDataSlot(QString ip, int port,int sockDesc,QByteArray data);
    void disconnectToHost(void);

private:
    MySocket *m_socket;

    int m_sockDesc;
};

#endif // SERVERTHREAD_H
