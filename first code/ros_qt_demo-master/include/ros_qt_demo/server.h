#ifndef SERVER_H
#define SERVER_H

#include <QTcpServer>

#include "serverthread.h"

//class Dialog;

class Server : public QTcpServer
{
    Q_OBJECT
public:
    explicit Server(QObject *parent = nullptr);
    ~Server();

signals:
    void sendData(int Ip,QByteArray Data);
    void dataReady(QString Ip,int port,int sockDesc,QByteArray Date);
    void sendshowconnection(int sockDesc);
    void senddisconnection(int sockDesc);
    void sendsockDesc(QList<int> socklist);
private:
    void incomingConnection(qintptr sockDesc);

public slots:
    void clientDisconnected(int sockDesc);
    void recvData(QString ip, int port,int sockDesc,QByteArray Date);
    void rece_date(int Ip,QByteArray Data);

private:
//    Dialog *m_dialog;

    QList<int> m_socketList;
    int m_sockDesc;
};

#endif // SERVER_H
