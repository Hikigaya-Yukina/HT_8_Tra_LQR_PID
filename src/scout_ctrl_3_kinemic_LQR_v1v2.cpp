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
#include "LQR_2.hpp"

using namespace std;

//定义 模型状态
Scout_ctrl Scout_chassis;

//定义 路径节点
nav_msgs::Path Path_nodes;

//路径点长度
int path_length;
//角度范围
float deg_max = 20.0/360.0 * M_PI;
float deg_min = 3.0/360.0 * M_PI;
float vel_error =0;
float angvel_error =0;

//定义迭代次数
int N = 200;

//定义LQR函数
LQRControl robot_motion_LQR(N);

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
float pid(float Pid_n[], float dt, float vel, float vel_state, float error_sum);

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

        if (i == (path_length)&& i != 0)
        {   
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            vel_pub.publish(cmd);
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
            float dt = Scout_chassis.dt;

            //发送的速度
            float cmd_avel = 0;
            float cmd_lvel = 0;

            //预测的速度
            float pre_avel = 0;
            float pre_lvel = 0;

            //gazebo里的速度
            float line_vel = Scout_chassis.Robot_velocity.linear.x;
            float ang_vel = Scout_chassis.Robot_velocity.angular.z;
            float theta;

            //在追踪首个点时怎么走
            if(i == 0)
            {
                theta = atan2(First_node.pose.position.y - Scout_chassis.Robot_pose.position.y, 
                First_node.pose.position.x - Scout_chassis.Robot_pose.position.x);

                theta = angle_normalized(theta);
            }
            else //不在首个点的时候怎么走
            {
                //计算参考角度
                theta = atan2(First_node.pose.position.y - Path_nodes.poses[i-1].pose.position.y, 
                First_node.pose.position.x - Path_nodes.poses[i-1].pose.position.x);
                theta = angle_normalized(theta);
            }

            //获取参考速度和参考角度
            float ref_vel = Scout_chassis.Vtar;
            float ref_angle = angle_normalized(theta);

            //计算 A,B矩阵
            
            vector<MatrixXd>A_B = robot_motion_LQR.stateSpace(ref_angle, dt, line_vel);

            //获取Q，R矩阵

            vector<MatrixXd>Q_R = robot_motion_LQR.getQR(1.0, 1.0);

            //获取角速度指令
            vector<double>state = {Scout_chassis.Robot_pose.position.x, Scout_chassis.Robot_pose.position.y, model_yaw};
            vector<double>ref_path = {First_node.pose.position.x, First_node.pose.position.y, ref_angle};
            pre_lvel, pre_avel = robot_motion_LQR.lqrControl(state, ref_path, A_B[0], A_B[1], Q_R[0], Q_R[1]);

            float distance = sqrt(pow(First_node.pose.position.x - Scout_chassis.Robot_pose.position.x,2) + 
            pow(First_node.pose.position.y - Scout_chassis.Robot_pose.position.y,2));
            ROS_INFO("pre_lvel = %f \n", pre_lvel);

            float pid_n[] = {20.0,0.5,0.5};
            float pid_n2[] = {20.0,0.5,0.5};
            
            
            float Vtar;
            if(distance < 0.5)
            {
                Vtar = Scout_chassis.Vtar/2;
            }
            else
            {
                Vtar = Scout_chassis.Vtar;
            }

            //用P更新角速度和线速度
            float pre_a = pid(pid_n, dt, pre_lvel, Scout_chassis.Robot_velocity.linear.x, vel_error);
            float pre_aa = pid(pid_n2, dt, pre_avel, Scout_chassis.Robot_velocity.angular.z, angvel_error);

            if(pre_a > Scout_chassis.acc_max)
            {
                pre_a = Scout_chassis.acc_max;
            }
            else if(pre_a < -Scout_chassis.acc_max)
            {
                pre_a = -Scout_chassis.acc_max;
            }
            else
            {
                pre_a = pre_a;
            }

            if(pre_aa > Scout_chassis.ang_acc_max)
            {
                pre_aa = Scout_chassis.ang_acc_max;
            }
            else if(pre_aa < -Scout_chassis.ang_acc_max)
            {
                pre_aa = -Scout_chassis.ang_acc_max;
            }
            else
            {
                pre_aa = pre_aa;
            }


            cmd_lvel = line_vel + 10*pre_a*dt;
            cmd_avel = ang_vel + 100*pre_aa*dt;

            if(cmd_lvel > Scout_chassis.Vtar)
            {
                cmd_lvel = Scout_chassis.Vtar;
            }
            else if(cmd_lvel < -Scout_chassis.Vtar)
            {
                cmd_lvel = -Scout_chassis.Vtar;
            }
            else
            {
                cmd_lvel = cmd_lvel;
            }

            if(cmd_avel > 6)
            {
                cmd_avel = 6;
            }
            else if(cmd_avel < -6)
            {
                cmd_avel = -6;
            }
            else
            {
                cmd_avel = cmd_avel;
            }
            
            // ROS_INFO("目前的速度%f 指令的速度%f 期待的角速度%f 跟踪的路径点序号%d 距离目的点的距离 %f\n",line_vel,cmd_lvel,pre_avel,i, distance);
            // 发出指令
            cmd.linear.x = cmd_lvel;
            cmd.angular.z = cmd_avel;
            vel_pub.publish(cmd);

            //计算小车到该点的距离
            
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

//获取圆的半径
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

float pid(float Pid_n[], float dt, float vel, float vel_state, float error_sum)
{
    float error = vel - vel_state;
    error_sum += error;

    return Pid_n[0]*error + Pid_n[1]*error_sum + Pid_n[2]/dt*error;
}

