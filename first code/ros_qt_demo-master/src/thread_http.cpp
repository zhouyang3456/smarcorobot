#include "../include/ros_qt_demo/thread_http.hpp"

thread_http::thread_http(const QUrl &requestedUrl):
    url(requestedUrl)
{

}

void thread_http::run()
{
    // [5]发送http请求
//    startRequest( QUrl("https://hitc.org.cn:30001/file/robot/upload/2023-04-20/mp3/84F39796-02E7-46F1-B079-3BDD97C74671.mp3"));
      startRequest(url);
      this->exec();

}

// [6]发起HTTP请求
/*
 * 功能描述：发送HTTP请求，并与请求响应的槽绑定
 *  @param requestedUrl：请求需要的URL地址
*/
void thread_http::startRequest(const QUrl &requestedUrl)
{
    url = requestedUrl;
    manager = new QNetworkAccessManager(this);

    reply = manager->get(QNetworkRequest(url));
    connect(reply,&QNetworkReply::finished,this,&thread_http::replyFinished);
    connect(reply,&QNetworkReply::readyRead,this,&thread_http::reply_readyRead);
    connect(reply,&QNetworkReply::downloadProgress,this,&thread_http::reply_downloadProgress);
}

// [7]获取HTTP的请求响应
/*
 * 功能描述：HTTP请求后，接收服务器的请求信息
 * 1 检测请求响应是否有错误
 * 2 获取请求响应的状态码
 * 3 判断是否需要重定向
 *   - 不需要，则保存数据
 *   - 需要重定向，则获取重定向的URL，然后通过这个URL再次发起请求
*/
void thread_http::replyFinished()
{
    // <1>判断有没有错误
    if (reply->error()){
        qDebug()<<reply->errorString();
        reply->deleteLater();
        return;
    }

    // <2>检测状态码
    int statusCode  = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    qDebug() << "statusCode:" << statusCode;

    // <3>判断是否需要重定向
    if (statusCode >= 200 && statusCode <300){

        QString file_path = "/home/zhou/Music";
        QString current_date_time = QDateTime::currentDateTime().toString("yyyy_MM_dd hh_mm_ss");
        QString file_name = file_path
                            + "/"
                            + current_date_time
                            + "_test.mp3";

        downFile = new QFile(file_name);
        if(!downFile->open(QIODevice::WriteOnly))
            return;

        downFile->write(reply->readAll());

        // <1> close file
        downFile->close();

        // 数据读取完成之后，清除reply
        reply->deleteLater();
        reply = nullptr;
    }

}

void thread_http::reply_readyRead()
{

}

void thread_http::reply_downloadProgress(qint64, qint64)
{

}
