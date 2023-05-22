#include "../include/ros_qt_demo/mysocket.h"
#include <QDebug>
#include <QThread>
MySocket::MySocket(int sockDesc, QObject *parent) :
    QTcpSocket(parent),
    m_sockDesc(sockDesc)
{
    connect(this, SIGNAL(readyRead()), this, SLOT(recvData()));
}

MySocket::~MySocket()
{
    
}

void MySocket::sendData(int id, QByteArray data)
{
    if (id == m_sockDesc && !data.isEmpty()) {
        this->write(data,data.size());
    }
}

void MySocket::recvData(void)
{
    QString ip = peerAddress().toString();
    int port = peerPort();
    QByteArray data = readAll();
    emit dataReady(ip,port,m_sockDesc,data);
}
