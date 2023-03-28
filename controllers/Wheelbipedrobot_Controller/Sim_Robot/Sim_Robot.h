/**
/ @file Sim_Robot.h
/ @brief 机器人与Webots仿真平台之间的传感器与执行器信号处理
/ @author 
/ @version 
/ @date 
/ @details 1.获取必要的传感器数据；2.向执行器传递控制信号
*/

#pragma one

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>

using namespace webots;


class Sim_Robot : public Robot{
    public:
        Sim_Robot():Robot(){};
        virtual ~Sim_Robot(){};

        void Get_Device(const std::string *);
        void Sensor_Init(int );
        void Sensor_Update();
        const double *Sensor_Transmit() const { return Sensor_data;}
        void Set_Motor_Torque(double *);

    private:
        Motor *LB_motor;
        Motor *LF_motor;
        Motor *LW_motor;
        Motor *RB_motor;
        Motor *RF_motor;
        Motor *RW_motor;
        PositionSensor *LB_ps;
        PositionSensor *LF_ps;
        PositionSensor *LW_ps;
        PositionSensor *RB_ps;
        PositionSensor *RF_ps;
        PositionSensor *RW_ps;
        Gyro *Body_gyro;
        InertialUnit *Body_IMU;

        const double *ptr;
        double Sensor_data[12]; // 前三个Body_IMU，前四到六个Body_gyro，后六个依次为RF,RB,RW,LF,LB,LW
};
