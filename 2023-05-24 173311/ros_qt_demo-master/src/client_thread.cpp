#include "../include/ros_qt_demo/client_thread.h"

Client_thread::Client_thread(QString ip, int port)
{
    serverIP = ip;
    serverPort = port;
    myClient = new QTcpSocket;
    connect(myClient,SIGNAL(connected()),this,SLOT(doProcessConnected()));
    connect(myClient,SIGNAL(readyRead()),this,SLOT(doProcessReadyRead()));
    connect(myClient,SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(doProcessError(QAbstractSocket::SocketError)));
    connect(myClient,SIGNAL(disconnected()),this,SLOT(doProcessDisconnected()));
    myClient->abort();
    myClient->connectToHost(QHostAddress(serverIP),serverPort);  //连接服务器
    myClient->setSocketOption(QAbstractSocket::LowDelayOption, 1);
}

void Client_thread::connect_to_host()
{
  myClient->abort();
  myClient->connectToHost(QHostAddress(serverIP),serverPort);  //连接服务器
}

void Client_thread::flush_tcp_send_buffer()
{
    myClient->flush();
}

void Client_thread::wait_for_bytes_written()
{
    myClient->waitForBytesWritten();
}

void Client_thread::doProcessConnected()
{
    emit signal_connected();
}

void Client_thread::doProcessReadyRead()
{
    while(myClient->bytesAvailable()>0)
    {
        QByteArray datagram;
        datagram.resize(myClient->bytesAvailable());
        myClient->read(datagram.data(),datagram.size());
        qDebug() << datagram.toHex();
        emit signal_senddata(datagram);
    }
//    QByteArray datagram = myClient->readAll();
//    emit signal_senddata(datagram);
}
void Client_thread::doProcessError(QAbstractSocket::SocketError error)
{
     qDebug()<<error;
     emit signal_senderror(error);
}

void Client_thread::doProcessDisconnected()
{
    myClient->abort();
    emit signal_disconnected();
}

void Client_thread::receive_senddata(QByteArray mdata)
{
  if(myClient->state()==QAbstractSocket::ConnectedState)
  {
      myClient->write(mdata);
     // myClient->flush();
  }
}


