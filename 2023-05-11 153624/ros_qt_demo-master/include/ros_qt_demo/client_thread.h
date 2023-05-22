#ifndef CLIENT_THREAD_H
#define CLIENT_THREAD_H
/* 多线程实现socket客户端 */
#include <QThread>

#include <QTcpSocket>
#include <QHostAddress>
#include <QAbstractSocket>
#include <qiodevice.h>
#include <QIODevice>

class Client_thread : public QObject
{
    Q_OBJECT
public:
    Client_thread(QString ip,int port);
    void receive_senddata(QByteArray mdata);
    int type1=qRegisterMetaType<QAbstractSocket::SocketError>("SocketError");//使用多线程前注册一下，否则信号槽没用
    void connect_to_host();
    void flush_tcp_send_buffer();
    void wait_for_bytes_written();
public slots:

protected:
//    void run();
signals:
    void signal_connected();
    void signal_disconnected();
    void signal_senddata(QByteArray datagram);
    void signal_senderror(QAbstractSocket::SocketError error);
private slots:
    void doProcessConnected();
    void doProcessReadyRead();
    void doProcessError(QAbstractSocket::SocketError error);
    void doProcessDisconnected();

private:
    QTcpSocket *myClient = nullptr;
    int serverPort;
    QString serverIP;
};

#endif // CLIENT_THREAD_H
