#include "../include/ros_qt_demo/serverthread.h"

serverThread::serverThread(int sockDesc, QObject *parent) :
    QThread(parent),
    m_sockDesc(sockDesc)
{

}

serverThread::~serverThread()
{
    m_socket->close();
}

void serverThread::run(void)
{
    m_socket = new MySocket(m_sockDesc);

    if (!m_socket->setSocketDescriptor(m_sockDesc)) {
        return ;
    }
    qDebug()<<"newconnect";
    connect(m_socket, SIGNAL(disconnected(void)), this,SLOT(disconnectToHost(void)));
    connect(m_socket, SIGNAL(dataReady(QString,int,int,QByteArray)),
            this, SLOT(recvDataSlot(QString,int,int,QByteArray)));
    connect(this, SIGNAL(sendData(int,QByteArray)),
            m_socket, SLOT(sendData(int,QByteArray)));

    this->exec();

}

void serverThread::sendDataSlot(int sockDesc, const QByteArray &data)
{
    if (data.isEmpty()) {
        return ;
    }

    emit sendData(sockDesc, data);
}

void serverThread::recvDataSlot(QString ip, int port, int sockDesc, QByteArray data)
{
    emit dataReady(ip,port,sockDesc,data);
}

void serverThread::disconnectToHost(void)
{
    qDebug()<<"disconnect";
    emit disconnectTCP(m_sockDesc);
    m_socket->disconnectFromHost();
    this->quit();
}
