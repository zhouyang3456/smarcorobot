#ifndef THREAD_HTTP_HPP
#define THREAD_HTTP_HPP

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QtNetwork>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTextCodec>

#include <QFile>
#include <QTextStream>
#include <QDateTime>

class thread_http : public QThread
{
    Q_OBJECT
public:
    thread_http(const QUrl &requestedUrl);

    void run() override;
private:
    // [3] 添加对象
    QUrl url;
    QNetworkRequest req;
    QNetworkReply *reply;
    QNetworkAccessManager *manager;
    QFile *downFile;
    // [4]声明一个槽
private slots:
    void startRequest(const QUrl &requestedUrl);
    void replyFinished();
    void reply_readyRead();
    void reply_downloadProgress(qint64 ,qint64);
    // void on_pushButton_clicked();
};

#endif // THREAD_HTTP_HPP
