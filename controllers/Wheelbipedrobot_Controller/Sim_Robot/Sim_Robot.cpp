/**
/ @file Sim_Robot.cpp
/ @author 
/ @date 
*/

#include <Sim_Robot.h>
#include <iostream>

/**
/ @brief    获取机器人设备
/ @param    string* str 字符串指针 14个元素 参数内容具体为:电机，电机编码器，陀螺仪，IMU，前两个具体依次为RF,RB,RW,LF,LB,LW
/ @return   void 
*/
void Sim_Robot::Get_Device(const std::string *str)
{
    RF_motor = getMotor(*(str + 0));
    RB_motor = getMotor(*(str + 1));
    RW_motor = getMotor(*(str + 2));
    LF_motor = getMotor(*(str + 3));
    LB_motor = getMotor(*(str + 4));
    LW_motor = getMotor(*(str + 5));

    RF_ps = getPositionSensor(*(str + 6));
    RB_ps = getPositionSensor(*(str + 7));
    RW_ps = getPositionSensor(*(str + 8));
    LF_ps = getPositionSensor(*(str + 9));
    LB_ps = getPositionSensor(*(str + 10));
    LW_ps = getPositionSensor(*(str + 11));

    Body_gyro = getGyro(*(str + 12));
    Body_IMU = getInertialUnit(*(str + 13));
}

/**
/ @brief  使能机器人传感器
/ @param  int samplingPeriod 采样周期
/ @return void
*/
void Sim_Robot::Sensor_Init(int samplingPeriod)
{
    RF_ps->enable(samplingPeriod);
    RB_ps->enable(samplingPeriod);
    RW_ps->enable(samplingPeriod);
    LF_ps->enable(samplingPeriod);
    LB_ps->enable(samplingPeriod);
    LW_ps->enable(samplingPeriod);
    
    Body_gyro->enable(samplingPeriod);
    Body_IMU->enable(samplingPeriod);
    // !使能后要等一个采样周期后才能读数据
}

/**
/ @brief 更新传感器数据 
/ @param None
/ @retval None
*/
void Sim_Robot::Sensor_Update()
{
    ptr = Body_IMU->getRollPitchYaw();
    Sensor_data[0] = *ptr;
    Sensor_data[1] = *(ptr + 1);
    Sensor_data[2] = *(ptr + 1);

    ptr = Body_gyro->getValues();
    Sensor_data[3] = *ptr;
    Sensor_data[4] = *(ptr + 1);
    Sensor_data[5] = *(ptr + 2);

    Sensor_data[6] = RF_ps->getValue();
    Sensor_data[7] = RB_ps->getValue();
    Sensor_data[8] = RW_ps->getValue();
    Sensor_data[9] = LF_ps->getValue();
    Sensor_data[10] = LB_ps->getValue();
    Sensor_data[11] = LW_ps->getValue();
}

/**
/ @brief 设置机器人电机力矩 
/ @param double* torque 力矩数组依次为RF,RB,RW,LF,LB,LW
/ @retval None
*/
void Sim_Robot::Set_Motor_Torque(double *torque)
{
    // * 这里有转换轮电机力矩方向
    RF_motor->setTorque(*(torque + 0));
    RB_motor->setTorque(*(torque + 1));
    RW_motor->setTorque(*(torque + 2));
    std::cout << "again Rwheel_torue = " << *(torque + 2) << "end!" << std::endl;
    LF_motor->setTorque(*(torque + 3));
    LB_motor->setTorque(*(torque + 4));
    LW_motor->setTorque(*(torque + 5));
}