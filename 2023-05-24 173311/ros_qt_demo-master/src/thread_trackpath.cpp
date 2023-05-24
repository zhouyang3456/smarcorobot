#include "../include/ros_qt_demo/thread_trackpath.h"
#include "/usr/local/include/yaml-cpp/yaml.h"
#include <vector>
using namespace std;

thread_trackPath::thread_trackPath(std::string track_file_path):
    track_file_path(track_file_path)
{
    path_ = new std::vector<geometry_msgs::PoseStamped>();

    // YAML::Node global_path = YAML::LoadFile("/home/smarcorobot/catkin_zy/src/ros_qt_demo-master/global_path.yaml");
    YAML::Node global_path = YAML::LoadFile(track_file_path);
    vector<vector<double>> poses = global_path["poses"].as<vector<vector<double>>>();
    for(int i = 0; i < poses.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        // set the frame ID
        pose.header.frame_id = "map";
        pose.pose.position.x = poses.at(i).at(0);
        pose.pose.position.y = poses.at(i).at(1);
        pose.pose.orientation.z = poses.at(i).at(2);
        pose.pose.orientation.w = poses.at(i).at(3);
        path_->push_back(pose);
    }
}

void thread_trackPath::run()
{
    qDebug() << "thread_trackPath Function: run() success";

    // show path in rviz
    emit PUB_GLOBAL_PATH();

    while(pose_update_flag != true);

    qDebug() << "start track path";

    // navigate to first point
    emit SET_GOAL_("map",
                   path_->at(0).pose.position.x,
                   path_->at(0).pose.position.y,
                   path_->at(0).pose.orientation.z,
                   path_->at(0).pose.orientation.w);

    qDebug() << " set goal, "
             << " pose_x: " << path_->at(0).pose.position.x
             << " pose_y: " << path_->at(0).pose.position.y
             << " pose_z: " << path_->at(0).pose.orientation.z
             << " pose_w: " << path_->at(0).pose.orientation.w
             << " success!";

    for(int i = 1; i < path_->size(); i++){
        while(is_distance_overflow(path_->at(i).pose.position.x,
                                   path_->at(i).pose.position.y));
        emit SET_GOAL_("map",
                       path_->at(i).pose.position.x,
                       path_->at(i).pose.position.y,
                       path_->at(i).pose.orientation.z,
                       path_->at(i).pose.orientation.w);

        qDebug() << " set goal,"
                 << " pose_x: " << current_x
                 << " pose_y: " << current_y
                 << " pose_z: " << current_z
                 << " pose_w: " << current_w
                 << " success!";
    }

    qDebug() << "exit thread_trackPath";

}

bool thread_trackPath::is_distance_overflow(double next_pos_x, double next_pos_y)
{
    double distance_now =
            qPow((next_pos_x - current_x), 2)
            + qPow((next_pos_y - current_y), 2);

    if(sqrt(distance_now) > distance_bias)
        return true;
    else
        return false;
}


void thread_trackPath::handle_current_pose_(double x_, double y_, double z_, double w_)
{
    current_x = x_;
    current_y = y_;
    current_z = z_;
    current_w = w_;
    pose_update_flag = true;
    /*
    qDebug() << " pose_x: " << current_x
             << " pose_y: " << current_y
             << " pose_z: " << current_z
             << " pose_w: " << current_w;
    */
}
