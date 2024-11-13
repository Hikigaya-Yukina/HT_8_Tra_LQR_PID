#include "scout_define.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <string>

using namespace std;

//定义 模型状态
Scout_ctrl Scout_chassis;

//定义 路径节点
nav_msgs::Path Path_nodes;

//路径点长度
int path_length;

//处理订阅的路径节点
void nodesCallback(const nav_msgs::Path::ConstPtr &msg)
{   
    //获取信息
    Path_nodes = *msg;
    path_length = msg->poses.size();
    // ROS_INFO("Path receiving work ends.");
    // ROS_INFO("Path length = %d", path_length);
}

void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // ROS_INFO("Getting the model state.");
    // 获取小车速度、位置和姿态
    Scout_chassis.Robot_velocity = msg->twist[1];
    Scout_chassis.Robot_pose = msg->pose[1];
}

float angle_normalized(float angle);
float getRadius(float x_0, float y_0, float x_1, float y_1, float theta, float yaw);

int main(int argc, char *argv[])
{   

    setlocale(LC_ALL, "");

    ros::init(argc,argv,"scout_ctrl");
    ros::NodeHandle nh;

    // 订阅路径节点和模型状态
    ros::Subscriber path_nodes_sub = nh.subscribe("/rrt_final_path",1000, nodesCallback);
    ros::Subscriber model_state_sub = nh.subscribe("/gazebo/model_states",1000, modelCallback);

    //发布速度指令
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::Rate loop_rate(100);
    ROS_INFO("Starting to publish cmd.");
    int i;
    i = 0;
    //发布处理循环
    while (ros::ok())
    {   
        ros::spinOnce();
        // ROS_INFO("Path length 等于 %d", path_length);
        //定义 输出指令
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;

        if (i == (path_length-1))
        {
            ROS_INFO("It reached the end");
            break;
        }

        //得到首个地图点
        //获取该地图点的顺序和坐标
        if(path_length != 0)
        {
            geometry_msgs::PoseStamped First_node;
            First_node.pose.position = Path_nodes.poses[i].pose.position;
            //计算小车的偏航角
            tf::Quaternion q(Scout_chassis.Robot_pose.orientation.x, Scout_chassis.Robot_pose.orientation.y, 
            Scout_chassis.Robot_pose.orientation.z, Scout_chassis.Robot_pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, model_yaw;
            m.getRPY(roll, pitch, model_yaw);
            model_yaw = angle_normalized(model_yaw);

            //计算小车到地图点的角度
            float theta = atan2(First_node.pose.position.y - Scout_chassis.Robot_pose.position.y, 
            First_node.pose.position.x - Scout_chassis.Robot_pose.position.x);

            //计算两个差角的大小
            float angle_diff = theta - model_yaw;
            angle_diff = angle_normalized(angle_diff);

            //判断 运动类型
            float deg_max = 20.0/360.0 * M_PI;
            float deg_min = 3.0/360.0 * M_PI;
            float angle_vel = 0;
            float center_vel = 0;
            float line_vel = Scout_chassis.Robot_velocity.linear.x;
            float ang_vel = Scout_chassis.Robot_velocity.angular.z;
        
            //如果 角度差大于5度，则先停下来自转
            //判断 开始旋转调整位置（Vmax=0.5 即 角速度 = 2*Vmax/L)
            if( abs(angle_diff) >= deg_max )
            {
                //控制转速
                if(abs(ang_vel)<(0.5 *2 / Scout_chassis.SCOUT_WHEELBASE))
                    angle_vel = ang_vel + Scout_chassis.ang_acc_max * (angle_diff/abs(angle_diff)) * Scout_chassis.dt;
                else
                    angle_vel = ang_vel - ang_vel/abs(ang_vel) * Scout_chassis.ang_acc_max * Scout_chassis.dt;
                //控制中心速度,大于0.02m/s的时候
                if( abs(angle_vel) > 0.01 )
                    center_vel = line_vel - Scout_chassis.acc_max * Scout_chassis.dt;
            }

            //判断 直接走直线
            else if (abs(angle_diff) <= deg_min)
            {
                
                if(abs(ang_vel) > 0.02)
                    angle_vel = ang_vel - ang_vel/abs(ang_vel) * Scout_chassis.ang_acc_max * Scout_chassis.dt;
                
                if(abs(line_vel)<Scout_chassis.Vtar)
                {
                    center_vel =line_vel + Scout_chassis.acc_max * Scout_chassis.dt;
                }
                else if(abs(line_vel)>=Scout_chassis.Vtar)
                {
                    center_vel =line_vel - line_vel/abs(line_vel) * Scout_chassis.acc_max * Scout_chassis.dt;
                }
            }

            //判断 小虎转弯
            else
            {
                //获取圆弧半径
                float x_0 = Scout_chassis.Robot_pose.position.x;
                float y_0 = Scout_chassis.Robot_pose.position.y;
                float x_1 = First_node.pose.position.x;
                float y_1 = First_node.pose.position.y;
                float R = getRadius(x_0,y_0,x_1,y_1,theta,model_yaw);
                center_vel = line_vel * ang_vel;

                //控制转速
                if(abs(ang_vel)<(0.5 *2 / Scout_chassis.SCOUT_WHEELBASE))
                    angle_vel = ang_vel + Scout_chassis.ang_acc_max * (angle_diff/abs(angle_diff)) * Scout_chassis.dt;
                else
                    angle_vel = ang_vel - ang_vel/abs(ang_vel) * Scout_chassis.ang_acc_max * Scout_chassis.dt;
            }
            
            // 发出指令
            cmd.linear.x = center_vel;
            cmd.angular.z = angle_vel;
            vel_pub.publish(cmd);

            //计算小车到该点的距离
            float distance = sqrt(pow(First_node.pose.position.x - Scout_chassis.Robot_pose.position.x,2) + 
            pow(First_node.pose.position.y - Scout_chassis.Robot_pose.position.y,2));

            //进入最近地图点多少米之后(0.1m)，目标设定为下一个点。
            if(distance < 0.1)
                i++;
        }
        loop_rate.sleep();
    }
    return 0;
}

//对角度作一个修正
float angle_normalized(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

float getRadius(float x_0, float y_0, float x_1, float y_1, float theta, float yaw)
{
    float x_m;
    x_m = (x_0 + x_1)/2;

    float y_m;
    y_m = (y_0 + y_1)/2;

    float x_c = (y_m - y_0 + x_0 * tan(yaw) - x_m * tan(theta))/(tan(yaw) - tan(theta));
    float y_c = (x_m - x_0 + y_0 / tan(yaw) - y_m / tan(theta))/(1/tan(yaw) - 1/tan(theta));

    float R = sqrt(pow(x_c - x_0,2) + pow(y_c - y_0,2));
    return R;
}


