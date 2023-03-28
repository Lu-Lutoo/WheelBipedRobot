// File:          Wheelbipedrobot_Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes


#include <Sim_Robot.h>
#include <Robot_Controller.h>

// All the webots classes are defined in the "webots" namespace

// 内容具体为:电机，电机编码器，陀螺仪，IMU，前两个具体依次为RF,RB,RW,LF,LB,LW
const std::string DEVICE_NAME[14] = {"RFM","RBM","RWM","LFM","LBM","LWM",
                                      "RFM_sensor","RBM_sensor","RWM_sensor","LFM_sensor","LBM_sensor","LWM_sensor",
                                      "Gyro","IMU"};

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Sim_Robot *robot = new Sim_Robot();
  Robot_Controller *controller = new Robot_Controller();
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  controller->Load_KData("./MatlabFile/K_polyn.csv");
  robot->Get_Device(DEVICE_NAME);
  robot->Sensor_Init(timeStep);

  robot->step(timeStep);
  robot->Sensor_Update();
  controller->Controller_Init(robot->Sensor_Transmit(), timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    robot->Sensor_Update();
    
    // Process sensor data here.
    controller->Controller_dataUpdate(robot->Sensor_Transmit());
  
    // Enter here functions to send actuator commands, like:
    robot->Set_Motor_Torque(controller->Get_MotorTorque());

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
