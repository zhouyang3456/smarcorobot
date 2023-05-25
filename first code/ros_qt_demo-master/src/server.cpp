#include "../include/ros_qt_demo/server.h"

Server::Server(QObject *parent) :
    QTcpServer(parent)
{
    /* get current dialog object */
//    m_dialog = dynamic_cast<Dialog *>(parent);
  qDebug()<<"server";
}

Server::~Server()
{

}


void Server::incomingConnection(qintptr sockDesc)
{
    m_socketList.append(sockDesc);

    serverThread *thread = new serverThread(sockDesc);
    emit sendshowconnection(sockDesc);
    connect(thread, SIGNAL(disconnectTCP(int)), this, SLOT(clientDisconnected(int)));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    connect(thread, SIGNAL(dataReady(QString,int,int,QByteArray)),
            this, SLOT(recvData(QString,int,int,QByteArray)));

    connect(this, SIGNAL(sendData(int,QByteArray)),
            thread, SLOT(sendDataSlot(int,QByteArray)));

    thread->start();
    emit sendsockDesc(m_socketList);
}

void Server::rece_date(int Ip,QByteArray Data)
{
    emit sendData(Ip,Data);
}


void Server::recvData(QString ip,int port,int sockDesc,QByteArray Date)
{
    emit dataReady(ip,port,sockDesc,Date);
}

void Server::clientDisconnected(int sockDesc)
{
      m_socketList.removeOne(sockDesc);
      emit senddisconnection(sockDesc);
      emit sendsockDesc(m_socketList);
}
