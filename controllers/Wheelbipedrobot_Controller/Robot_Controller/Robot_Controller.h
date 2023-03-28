/**
/ @file cxx.h
/ @brief 
/ @author 
/ @version 
/ @date 
/ @details 
*/

#pragma one

#include <cstdint>
#include <Eigen/Dense>
#include <PID.h>
#include <fstream>

#define USER_PI 3.1415926536

/**
 * /TODO1: 转换数据的参考系
 * /TODO2: 计算虚拟腿长L，腿偏角theta
 * /TODO3: 计算状态变量
 * /TODO4: 计算期望腿长
 * /TODO5：LQR计算
 * /TODO6：腿长PID控制
 * /TODO7：转向PID控制
 * /TODO8：两腿角差PID控制
 * /TODO9：VMC力空间转换
 * /TODO10：输出控制量
 * TODO11: 提供设置目标值的函数
 * 
 */

class Robot_Controller
{
    public:
        Robot_Controller(){}
        ~Robot_Controller(){}

        void Controller_Init(const double *, int );
        void Controller_dataUpdate(const double *);
        double* Get_MotorTorque();
        void Load_KData(std::string);

    private:
        void wheel_dataUpdata(uint8_t, const double *);
        void forward_kinematics_solution(uint8_t );
        void get_LegForce();
        void LQR(uint8_t );
        void yawSpeedcontrol();
        void phiDiffcontrol();
        void VMC(uint8_t);

        double samplingPeriod;
        double joing_angle[4];  //!单位是rad 依次为RF,RB,LF,LB
        double wheel_angle_single[2]; // used to count
        const double *Euler_angle;    //!单位是rad 依次为roll,pitch,yaw
        const double *Euler_gyro;     //!单位是rad/s

        double Leg_length[2];   // R,L

        Eigen::Vector<double, 6> State[2]; // row:R,W; column:theta,Dtheta,x,Dx,pitch,Dpitch
        Eigen::Vector<double, 6> StateD[2];

        double Leg_length_target = 0.18; //target,R,L
        double state_target[6];
        double yaw_speed_target;

        double Leg_Force[2]; //R,L
        double Torque[6]; //* 作为最终关节电机力矩 R1,R2,Rw,L1,L2,Lw

        PID rightLegController;
        PID leftLegController;
        PID yawspeedPIDController;
        PID phiDiffPIDController;

        // 用于VMC计算的矩阵
        Eigen::Matrix2d Jaccobian[2];
        Eigen::Vector2d VirtualForce[2]; // R,L;F,Tk
        Eigen::Vector2d HipForce[2];     // R,L

        // 用于LQR计算的矩阵
        std::ifstream MatrixDataFile;
        Eigen::Matrix<double, 2, 6> K_polyn[4];
        Eigen::Matrix<double, 2, 6> KMatrix[2]; // R,L
        Eigen::Vector2d UVector[2];             // R,L;Tw,Tk

        double temp_phi[2],temp_newphi,temp_Dphi;//R,L
};


