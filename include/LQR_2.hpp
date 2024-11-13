#pragma once
# include<cmath>
# include<iostream>
# include<vector>
# include<eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

constexpr double EPS = 1.0e-4;

class LQRControl{
private:
    int N;     // 可能是迭代步长

public:
    LQRControl(int n):N(n){};

    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R){
    MatrixXd Qf = Q;
    MatrixXd P_old = Qf;
    MatrixXd P_new;

    for(int i = 0;i<N;i++){
    P_new = Q + A.transpose() * P_old * A - A.transpose() * P_old * B * (R + B.transpose() * P_old * B).inverse() * B.transpose() * P_old * A;
    //  if((P_new - P_old).cwiseAbs().maxCoeff()<EPS) break;
    if((P_new-P_old).maxCoeff()<EPS&&(P_old-P_new).maxCoeff()<EPS)break;

    P_old= P_new;
    }

    return P_new;
}



    double lqrControl(vector<double>robot_state,vector<double>refer_path, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R){
    MatrixXd X(3,1);
    X<< robot_state[0] - refer_path[0],
        robot_state[1] - refer_path[1],
        robot_state[2] - refer_path[2];//  x是当前位置和预瞄点的偏差  x,y,yaw三个偏差

    MatrixXd P = calRicatti(A,B,Q,R);    //   3*3
    
    MatrixXd K = (R+B.transpose()*P*B).inverse()*B.transpose()*P*A;  // 反馈增益 2*3
    MatrixXd u = -K*X;   // 2*1
    return u(1,0);  // return delta_theta   角速度偏差值
}

vector<MatrixXd> stateSpace(float ref_yaw, float dt , float v){   
    MatrixXd A(3,3);
    MatrixXd B(3,2);
    A<<1.0,0.0,-v*dt*sin(ref_yaw),
        0.0,1.0,v*dt*cos(ref_yaw),
        0.0,0.0,1.0;
    B<<dt*cos(ref_yaw),0,
        dt*sin(ref_yaw),0,
        0,dt;
    return {A,B};

}

vector<MatrixXd> getQR(float q, float r){
    MatrixXd Q(3,3);
    MatrixXd R(2,2);
    Q<<q,0,0,
        0,q,0,
        0,0,q;
    R<<r,0,
        0,r;
    return {Q,R};
}

};

