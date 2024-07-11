#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "plan_env/lec4.h"
#include "ego_planner/TutorialGoal.h"
#include <vector>

using namespace std;

ros::Subscriber odom_sub;
ros::Publisher param_goal_pub;
ros::ServiceClient client;
int waypoint_num_;
vector<vector<double>> waypoints_;
double spin_rate;

// 解析字符串为二维vector
vector<vector<double>> parseCoords(const string& coord_string) {
    vector<vector<double>> coords;
    stringstream ss(coord_string);
    string vector_str;

    // 使用';'分隔各个子vector
    while (getline(ss, vector_str, ';')) {
        vector<double> vector;
        stringstream vector_ss(vector_str);
        string element;

        // 使用','分隔每个子vector中的元素
        while (getline(vector_ss, element, ',')) {
            vector.push_back(stod(element));
        }

        coords.push_back(vector);
    }

    return coords;
}

void OdomCallback(const nav_msgs::Odometry& msg) {
    ROS_WARN_ONCE("odom CB");
    static int way_point_count = 0;

    if (way_point_count >= waypoint_num_) {
        ROS_WARN_ONCE("all points pub");
        return;
    }

    float dist = std::sqrt(std::pow(waypoints_[way_point_count][0] - msg.pose.pose.position.x, 2) + 
                           std::pow(waypoints_[way_point_count][1] - msg.pose.pose.position.y, 2) + 
                           std::pow(waypoints_[way_point_count][2] - msg.pose.pose.position.z, 2));


        ego_planner::TutorialGoal goal;
        goal.goal_x = waypoints_[way_point_count][0];
        goal.goal_y = waypoints_[way_point_count][1];
        goal.goal_z = waypoints_[way_point_count][2];
        param_goal_pub.publish(goal);
        if(dist<1){
            way_point_count++;
            ROS_INFO("next point: (%.2f, %.2f, %.2f)", waypoints_[way_point_count][0], waypoints_[way_point_count][1], waypoints_[way_point_count][2]);
        } 
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "exercise2_param_node");
    ros::NodeHandle n("~");

    odom_sub = n.subscribe("/odom", 10, OdomCallback);
    param_goal_pub = n.advertise<ego_planner::TutorialGoal>("/waypoint_generator/tutorial_goal", 10);

    // 读取参数
    std::string coord_string;
    n.getParam("coords", coord_string);
    ROS_INFO("param: %s", coord_string.c_str());

    waypoints_ = parseCoords(coord_string);
    waypoint_num_ = waypoints_.size();

    n.param("spin_rate", spin_rate, 10.0);
    ros::Duration(0.5).sleep();

    ros::Rate loop_rate(spin_rate);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
