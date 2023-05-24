#ifndef THREAD_TRACKPATH_H
#define THREAD_TRACKPATH_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <vector>
#include "qnode.hpp"

using namespace std;

class thread_trackPath : public QThread
{
    Q_OBJECT
public:
    thread_trackPath(std::string track_file_path);
    void run() override;
    bool is_distance_overflow(double next_pos_x, double next_pos_y);

    std::string track_file_path;

    bool pose_update_flag = false;
    double current_x = 0;
    double current_y = 0;
    double current_z = 0;
    double current_w = 0;
    double distance_bias = 0.3;  // m

    std::vector<geometry_msgs::PoseStamped>* path_;

Q_SIGNALS:
    void SET_GOAL_(QString, double, double, double, double);
    void PUB_GLOBAL_PATH();

public slots:
    void handle_current_pose_(double x_, double y_, double z_, double w_);
};

#endif // THREAD_TRACKPATH_H
