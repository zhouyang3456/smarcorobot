#ifndef MYSOCKET_H
#define MYSOCKET_H

#include <QTcpSocket>
#include <QHostAddress>
#include <QObject>
#include <QDebug>

class MySocket : public QTcpSocket
{
    Q_OBJECT
public:
    explicit MySocket(int sockDesc, QObject *parent = nullptr);
    ~MySocket();
signals:
    void dataReady(QString ip,int port,int sockDesc,QByteArray data);

public slots:
    void recvData(void);
    void sendData(int id,QByteArray data);

private:
    int m_sockDesc;
};

#endif // MYSOCKET_H
