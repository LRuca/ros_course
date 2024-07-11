#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <plan_env/lec4.h>
#include "ego_planner/TutorialGoal.h"

using namespace std;
ros::Publisher param_goal_pub;
ros::Publisher goal_vis_pub;
double goal_key_x, goal_key_y, goal_key_z;
double move_distance = 0.1;
ros::ServiceClient client;

// 检查坐标点是否占据，非占据则发送目标点给无人机
void CheckAndPubGoal(const double& goal_x,
                     const double& goal_y,
                     const double& goal_z){

    // 调用服务，判断目标点是否占据
    plan_env::lec4 srv;
    srv.request.x = goal_x;
    srv.request.y = goal_y;
    srv.request.z = goal_z;

    if (client.call(srv)) { //检测服务是否正常运行
        if (srv.response.is_valid) {
            if (srv.response.is_valid) {
                // 发送目标点给无人机执行
                geometry_msgs::PoseStamped goal_msg;
                ego_planner::TutorialGoal goal;
                goal.goal_x = goal_x;
                goal.goal_y = goal_y;
                goal.goal_z = goal_z;
                param_goal_pub.publish(goal);
                ROS_INFO("Published goal: (%.2f, %.2f, %.2f)", goal_x, goal_y, goal_z);
            } else {
                ROS_INFO("Goal position is occupied.");//被占据则发布消息
            }
        } else {
            ROS_ERROR("Failed to call service /call_valid");
        }
    } else {
        ROS_ERROR("Failed to call service /call_valid");
    }
}

// 发布键盘设定的目标点供RViz可视化
void VisKeyGoal(const double& goal_x,
                const double& goal_y,
                const double& goal_z){

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.pose.position.x = goal_x;
    goal_msg.pose.position.y = goal_y;
    goal_msg.pose.position.z = goal_z;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "world";  // 使用世界坐标系

    goal_vis_pub.publish(goal_msg);
    ROS_INFO("Published visualization goal: (%.2f, %.2f, %.2f)", goal_x, goal_y, goal_z);
}

// 键盘回调函数，处理键盘输入设定目标点
void KeyCallback(const sensor_msgs::Joy& msg) {

    goal_key_x += msg.axes.at(4) * move_distance;   // 前后
    goal_key_y += msg.axes.at(3) * move_distance;   // 左右
    goal_key_z += msg.axes.at(1) * move_distance;   // 上下

    VisKeyGoal(goal_key_x, goal_key_y, goal_key_z);

    if (msg.buttons.at(6)) {  // 如果按下N键
        CheckAndPubGoal(goal_key_x, goal_key_y, goal_key_z);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "exercise2_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle n("~");
    
    // 注册 Subscriber 和 Publisher
    ros::Subscriber key_sub = nh.subscribe("/joy", 10, KeyCallback);
    goal_vis_pub = nh.advertise<geometry_msgs::PoseStamped>("/key_goal", 10);
    client = nh.serviceClient<plan_env::lec4>("/call_valid");
    param_goal_pub = n.advertise<ego_planner::TutorialGoal>("/waypoint_generator/tutorial_goal", 10);

    // 键盘目标点坐标初始化
    goal_key_x = goal_key_y = goal_key_z = 0;

    // 设置自旋频率
    double spin_rate;
    nh_priv.param("/spin_rate", spin_rate, 10.0);
    ros::Rate loop_rate(spin_rate);

    // 循环处理ROS回调函数
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
