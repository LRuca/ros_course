#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <std_msgs/Int32.h>

geometry_msgs::Twist twist_msg; 
ros::Publisher twist_pub;

ros::Publisher path_pub;
nav_msgs::Path path_msg;
int type;
double x_start, y_start, theta_start;
bool drawing = true;
int step = 0;
int step_old=-1;
void move(double x, double y, double z, double rx, double ry, double rz) {
    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = z;
    twist_msg.angular.x = rx;
    twist_msg.angular.y = ry;
    twist_msg.angular.z = rz;
    twist_pub.publish(twist_msg);
}

void stop() {
    move(0, 0, 0, 0, 0, 0);
}

void msg(const turtlesim::Pose& pose){
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.stamp = ros::Time::now();
    pose_stamp.header.frame_id = "turtle_frame";  // 设置路径点的参考坐标系
    pose_stamp.pose.position.x = pose.x;
    pose_stamp.pose.position.y = pose.y;
    path_msg.header.frame_id="turtle_frame";
    path_msg.poses.push_back(pose_stamp);
    path_pub.publish(path_msg);
}

void circle(const turtlesim::Pose& pose) {
    double radius = 1.0;
    double linear_speed = 1.0;
    double angular_speed = linear_speed / radius;

    static double start_theta = pose.theta;

    msg(pose);
    

    static double initial_x,initial_y;
    if(step_old!=step){
    initial_x = pose.x;
    initial_y = pose.y;
    step_old=step;
    }
    double dist = std::sqrt(std::pow(pose.x - initial_x, 2) + std::pow(pose.y - initial_y, 2));
    static int flag =0;
    if(dist>radius) flag=1;
    ROS_INFO("%lf,%d", dist,flag);
    if (dist<=0.01 && flag) {
        stop();
        drawing = false;
    } else {
        move(linear_speed, 0, 0, 0, 0, angular_speed);
    }
}

void moveInDirection(const turtlesim::Pose& pose, double angle, double distance) {
    static double initial_x,initial_y;
    if(step_old!=step){
    initial_x = pose.x;
    initial_y = pose.y;
    step_old=step;
    }
    double dist = std::sqrt(std::pow(pose.x - initial_x, 2) + std::pow(pose.y - initial_y, 2));
    
    msg(pose);

    if (dist >= distance) {
        stop();
        drawing = false;
        step++;
    } else {
        double move_x = cos(angle) * 2.0;
        double move_y = sin(angle) * 2.0;
        move(move_x, move_y, 0, 0, 0, 0);
    }
}

void square(const turtlesim::Pose& pose) {
    switch (step) {
        case 0: moveInDirection(pose, 0, 2); break;
        case 1: moveInDirection(pose, M_PI / 2, 2); break;
        case 2: moveInDirection(pose, M_PI, 2); break;
        case 3: moveInDirection(pose, -M_PI / 2, 2); break;
        case 4: stop(); drawing = false; break;
    }
}

void draw86(const turtlesim::Pose& pose) {
    switch (step) {
        case 0: moveInDirection(pose, 0, 2); break;
        case 1: moveInDirection(pose, M_PI / 2, 2); break;
        case 2: moveInDirection(pose, M_PI, 2); break;
        case 3: moveInDirection(pose, -M_PI / 2, 4); break;
        case 4: moveInDirection(pose, 0, 2); break;
        case 5: moveInDirection(pose, M_PI / 2, 2); break;
        case 6: moveInDirection(pose, 0, 3); break;
        case 7: moveInDirection(pose, -M_PI / 2, 2); break;
        case 8: moveInDirection(pose, M_PI, 2); break;
        case 9: moveInDirection(pose, M_PI / 2, 4); break;
        case 10: moveInDirection(pose, 0, 2); break;
        case 11: stop(); drawing = false; break;
            
    }
}

void poseCallback(const turtlesim::Pose& pose) {
    switch(type){
    case 0:circle(pose);break;
    case 1:square(pose);break;
    case 2:draw86(pose);break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_controller");
    ros::NodeHandle nh;
    twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    path_pub = nh.advertise<nav_msgs::Path>("turtle_path", 1);  // 设置路径消息的发布器
    nh.param<int>("turtle_controller/type", type, 0);
    ROS_INFO("Type: %d", type);  // 添加调试信息
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
    ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");
    std_srvs::Empty empty;
    reset.call(empty);

    // 设置路径消息的初始参数
    path_msg.header.frame_id = "turtle_frame";  // 设置路径消息的参考坐标系
    
    ros::spin();
    return 0;
}

