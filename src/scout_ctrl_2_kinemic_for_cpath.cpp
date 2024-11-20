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
#include "LQR_cpath.hpp"

using namespace std;

//定义 模型状态
Scout_ctrl Scout_chassis;

//定义 路径节点
nav_msgs::Path Ori_path;
nav_msgs::Path Path_nodes;
nav_msgs::Path passed_by;
//路径点长度
int path_length;
int path_length_o = 0;
//角度范围
float deg_max = 10.0/180.0 * M_PI;
float deg_min = 3.0/360.0 * M_PI;
float vel_error =0;
float angvel_error =0;
float g = 9.8;
//定义迭代次数
int N = 200;

//定义LQR函数
LQRControl robot_motion_LQR(N);
int q1 = 1;
int q2 = 1;
int q3 = 1;
int r1 = 1;
int r2 = 1;
//路径处理标志
bool path_flag = 0;
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
int isGravityCompensation(double roll, double pitch, double yaw, double ref_yaw);
bool isTurnModified(double yaw, double ref_yaw);
bool isStuck();
bool isFrictionCompensation();
bool isrugged();
bool isPlained();

int main(int argc, char *argv[])
{   

    setlocale(LC_ALL, "");

    ros::init(argc,argv,"scout_ctrl");
    ros::NodeHandle nh;

    // 订阅路径节点和模型状态
    ros::Subscriber path_nodes_sub = nh.subscribe("/rrt_final_path",1000, nodesCallback);
    ros::Subscriber model_state_sub = nh.subscribe("/gazebo/model_states",1000, modelCallback);
    // 发布途经的路径
    ros::Publisher passed_by_pub = nh.advertise<nav_msgs::Path>("/passed_by",100);
    //发布最终的路径节点
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_pub",100);
    //发布速度指令
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::Rate loop_rate(100);
    ROS_INFO("Starting to publish cmd.");
    int i;
    i = 0;
    int j = 0;
    //发布处理循环
    while (ros::ok())
    {   
        ros::spinOnce();

        //定义 输出指令
        geometry_msgs::Twist cmd;

        //停止指令
        if (i == (path_length)&& i != 0)
        {   
            
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            vel_pub.publish(cmd);
            j++;
        }

        //得到首个地图点
        //获取该地图点的顺序和坐标
        if(path_length!= 0 && i < (path_length))
        {

            geometry_msgs::PoseStamped First_node;
            First_node.pose.position = Path_nodes.poses[i].pose.position;

            //计算小车的偏航角
            tf::Quaternion q(Scout_chassis.Robot_pose.orientation.x, Scout_chassis.Robot_pose.orientation.y, 
            Scout_chassis.Robot_pose.orientation.z, Scout_chassis.Robot_pose.orientation.w);
            tf::Matrix3x3 m(q);
            
            // 计算baselink的位置 + l/2
            tf::Vector3 v(Scout_chassis.SCOUT_WHEELBASE/2, 0, 0);
            v = m * v;
            //重新更改跟踪的位置
            Scout_chassis.Robot_pose.position.x += v.getX();
            Scout_chassis.Robot_pose.position.y += v.getY();
            //计算偏航角
            double roll, pitch, model_yaw;
            m.getRPY(roll, pitch, model_yaw);
            model_yaw = angle_normalized(model_yaw);
            roll = angle_normalized(roll);
            pitch = angle_normalized(pitch);
            //重力加速度分量
            float g_x = -sin(pitch) * g;
            float g_y = cos(pitch) * sin(roll) * g;

            //步长时间
            float dt = Scout_chassis.dt;

            //发送的速度
            float cmd_avel = 0;
            float cmd_lvel = 0;

            //预测的速度
            float pre_avel = 0;
            float pre_lvel = 0;
            double pre_vel[2];
            //gazebo里的速度
            float line_vel = Scout_chassis.Robot_velocity.linear.x;
            float ang_vel = Scout_chassis.Robot_velocity.angular.z;
            float theta;

            //在追踪首个点时怎么走
            theta = atan2(First_node.pose.position.y - Scout_chassis.Robot_pose.position.y, 
            First_node.pose.position.x - Scout_chassis.Robot_pose.position.x);
            theta = angle_normalized(theta);

            //获取参考速度和参考角度
            float ref_vel = Scout_chassis.Vtar;
            float ref_angle = angle_normalized(theta);
            float angdiff = angle_normalized(theta - model_yaw);
            //计算 A,B矩阵
            
            vector<MatrixXd>A_B = robot_motion_LQR.stateSpace(ref_angle, dt, line_vel);
            
            //获取Q，R矩阵参数
            nh.param("q1", q1, q1); 
            nh.param("q2", q2, q2);
            nh.param("q3", q3, q3);
            nh.param("r1", r1, r1);
            nh.param("r2", r2, r2);
            //获取Q，R矩阵

            vector<MatrixXd>Q_R = robot_motion_LQR.getQR(q1,q2,q3,r1,r2);

            //获取角速度指令
            vector<double>state = {Scout_chassis.Robot_pose.position.x, Scout_chassis.Robot_pose.position.y, model_yaw};
            vector<double>ref_path = {First_node.pose.position.x, First_node.pose.position.y, ref_angle};
            
            robot_motion_LQR.lqrControl(state, ref_path, A_B[0], A_B[1], Q_R[0], Q_R[1], pre_vel);//预测的角速度
            pre_avel = pre_vel[1];
        
            float distance = sqrt(pow(First_node.pose.position.x - Scout_chassis.Robot_pose.position.x,2) + 
            pow(First_node.pose.position.y - Scout_chassis.Robot_pose.position.y,2));

            float pid_n[] = {20.0,10,0.5};
            float pid_n2[] = {100.0,5,0.2};
            
            
            float Vtar;
            if(distance < 0.4)
            {
                Vtar = Scout_chassis.Vtar/2;
            }
            else
            {
                Vtar = Scout_chassis.Vtar;
            }
        
            //用Pid更新角速度和线速度 pitch 上坡是负的
            float pre_a;
            if(abs(angdiff)>deg_max)//差不多15度
            {
                pre_a = pid(pid_n, dt, Vtar/2, Scout_chassis.Robot_velocity.linear.x, vel_error);
            }//单纯的原地旋转是有问题的。
            else{
                pre_a = pid(pid_n, dt, Vtar, Scout_chassis.Robot_velocity.linear.x, vel_error);
            }
            
            float pre_aa = pid(pid_n2, dt, pre_avel, Scout_chassis.Robot_velocity.angular.z, angvel_error);//角加速度

            if(pre_a > Scout_chassis.acc_max)
            {
                pre_a = Scout_chassis.acc_max;
            }
            else if(pre_a < -Scout_chassis.acc_max)
            {
                pre_a = -Scout_chassis.acc_max;
            }//判断有没有超过最大值

            if(pre_aa > Scout_chassis.ang_acc_max)
            {
                pre_aa = Scout_chassis.ang_acc_max;
            }
            else if(pre_aa < -Scout_chassis.ang_acc_max)
            {
                pre_aa = -Scout_chassis.ang_acc_max;
            }//判断有没有超过最大值

            cmd_lvel = line_vel + pre_a*dt;
            cmd_avel = ang_vel + pre_aa*dt;

            if(cmd_lvel > Scout_chassis.Vtar)
            {
                cmd_lvel = Scout_chassis.Vtar;
            }
            else if(cmd_lvel < -Scout_chassis.Vtar)
            {
                cmd_lvel = -Scout_chassis.Vtar;
            }

            if(cmd_avel > 3)
            {
                cmd_avel = 3;
            }
            else if(cmd_avel < -3)
            {
                cmd_avel = -3;
            }
            
            ROS_INFO("C速度%f P速度%f 加速度%f",line_vel,cmd_lvel,pre_a);
            ROS_INFO("P角速度%f C角速度%f 角加速度%f",pre_avel, ang_vel, pre_aa);
            ROS_INFO("角差%f 序号%d 距离 %f", angdiff, i, distance);
            ROS_INFO("R:%f, P:%f, Y:%f",roll,pitch,model_yaw);
            ROS_INFO("路径长度%d ,原长度%d \n",path_length_o,path_length);
            // 发出指令
            cmd.linear.x = cmd_lvel;
            // cmd.linear.x = 1;
            cmd.angular.z = cmd_avel;
            vel_pub.publish(cmd);

            //定义发布的路径
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map"; // 设置参考坐标系
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = Scout_chassis.Robot_pose.position.x;
            pose.pose.position.y = Scout_chassis.Robot_pose.position.y;
            pose.pose.position.z = 0;
            pose.pose.orientation.w = 1.0;
            passed_by.poses.push_back(pose);
        
            passed_by.header.stamp = ros::Time::now(); // 更新路径的时间戳
            passed_by.header.frame_id = "map";
            passed_by_pub.publish(passed_by);

            //进入最近地图点多少米之后(0.1m)，目标设定为下一个点。
            if(distance < 0.2 && i <(path_length-1)) i++;
            else if(distance < 0.03 && i == (path_length-1)) i++;
        }
        loop_rate.sleep();
        
        if(j>1000){
            ROS_INFO("抵达目的地，跟踪结束");
            break;
        } 
    }
    return 0;
    
}

//对角度作一个修正


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
    // if(error_sum > 0 && error <0) error_sum = 0;
    // else if(error_sum< 0 && error >0) error_sum = 0;
    return Pid_n[0]*error + Pid_n[1]*error_sum + Pid_n[2]/dt*error;
}

