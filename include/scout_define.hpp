#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>

class Scout_ctrl{
public:
    geometry_msgs::Twist Robot_velocity;
    geometry_msgs::Pose Robot_pose;

    //定义 角加速度
    float ang_acc_max = 50;
    float dt = 0.01;

    //定义 加速度
    float acc_max = 50;

    //定义 目标最大速度
    float Vtar = 1.2;

    const double SCOUT_WHEELBASE = 0.498;
    const double SCOUT_WHEEL_RADIUS = 0.16459;

    //定义 最大速度
    float Vmax = 1.5;

};