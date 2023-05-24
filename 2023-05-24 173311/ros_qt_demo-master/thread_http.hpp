#ifndef THREAD_HTTP_HPP
#define THREAD_HTTP_HPP

#include <QObject>

class thread_http : public QThread
{
    Q_OBJECT
public:
    thread_http();
};

#endif // THREAD_HTTP_HPP
