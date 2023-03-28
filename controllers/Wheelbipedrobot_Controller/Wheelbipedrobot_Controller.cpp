// File:          Wheelbipedrobot_Controller.cpp
// Date:          2023.3.28
// Description:   基于LQR的五连杆轮腿式机器人平衡控制
// Author:        Lutoo


#include <Sim_Robot.h>
#include <Robot_Controller.h>

// 内容具体为:电机，电机编码器，陀螺仪，IMU，前两个具体依次为RF,RB,RW,LF,LB,LW
const std::string DEVICE_NAME[14] = {"RFM","RBM","RWM","LFM","LBM","LWM",
                                      "RFM_sensor","RBM_sensor","RWM_sensor","LFM_sensor","LBM_sensor","LWM_sensor",
                                      "Gyro","IMU"};

// This is the main program of the controller.
int main(int argc, char **argv) {
  // create the Robot instance.
  Sim_Robot *robot = new Sim_Robot();
  Robot_Controller *controller = new Robot_Controller();
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  // robot init
  controller->Load_KData("./MatlabFile/K_polyn.csv");
  robot->Get_Device(DEVICE_NAME);
  robot->Sensor_Init(timeStep);
  // data init
  robot->step(timeStep);
  controller->Controller_Init(robot->Sensor_dataUpdate(), timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {

    controller->Controller_dataUpdate(robot->Sensor_dataUpdate());
  
    robot->Set_Motor_Torque(controller->Get_MotorTorque());

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
