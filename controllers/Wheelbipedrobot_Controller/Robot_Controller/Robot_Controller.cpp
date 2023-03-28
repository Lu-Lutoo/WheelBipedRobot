/**
/ @file Robot_Controller.cpp
/ @author Lutoo
/ @date 
*/

#include <Robot_Controller.h>
#include <cmath>
#include <config.h>
#include <iostream>

#if(USER_DEBUG == 1)
#include <assert.h>
#endif

const double link_len[5] = {0.15,0.288,0.288,0.15,0.15};
const double wheel_radius = 0.065;
const double body_width = 0.4;
const double M_P = (0.032+0.138) * 2;
const double M = 11.983 / 2;
const double G = 9.8;

/**
/ @brief 初始化数据
/ @param double* sensor_data 12个元素 前三个Body_IMU，前四到六个Body_gyro，后六个依次为RF,RB,RW,LF,LB,LW
/ @param int& t 传感器采样周期 ms
/ @retval None
*/
void Robot_Controller::Controller_Init(const double* sensor_data, int t){
    State[0].setZero();
    StateD[0].setZero();
    State[1].setZero();
    StateD[1].setZero();

    samplingPeriod = t/1000.0;
    Euler_angle = sensor_data;
    Euler_gyro = sensor_data + 3;

    joing_angle[0] = *(sensor_data + 6) + USER_PI; //RF //检查无误
    joing_angle[1] = *(sensor_data + 7); // RB   
    joing_angle[2] = *(sensor_data + 9) + USER_PI; // LF //!反向
    joing_angle[3] = *(sensor_data + 10); //LB      //! 反向

    wheel_angle_single[0] = *(sensor_data + 8); //RW
    wheel_angle_single[1] = *(sensor_data + 11); //LW

    // 初始化状态变量中的腿偏角和俯仰角，其他都为零
    forward_kinematics_solution(0);
    forward_kinematics_solution(1);
    // 初始化状态变量
    State[0](4) = *(Euler_angle + 1);
    State[1](4) = *(Euler_angle + 1);
    // 初始化PID控制器参数
    leftLegController.Init(100.0, 10, 0, 50, samplingPeriod, 0.25);
    rightLegController.Init(100.0, 10, 0, 50, samplingPeriod, 0.25);
    phiDiffPIDController.Init(10.0, 0.0, 1, 50, samplingPeriod, 0);
    yawspeedPIDController.Init(1.0, 0, 0, 20, samplingPeriod, 0);
}

/**
/ @brief 控制器数据更新,更新状态变量
/ @param double* sensor_data 12个元素 前三个Body_IMU，前四到六个Body_gyro，后六个依次为RF,RB,RW,LF,LB,LW
/ @retval None
*/
void Robot_Controller::Controller_dataUpdate(const double* sensor_data){
    // 更新IMU，陀螺仪数据
    Euler_angle = sensor_data;
    Euler_gyro = sensor_data + 3;

    // ! 更新状态变量pitch,Dpitch
    State[0](4) = *(Euler_angle + 1);
    State[1](4) = *(Euler_angle + 1);
    State[0](5) = *(Euler_gyro + 1);
    State[1](5) = *(Euler_gyro + 1);

    // 更新关节角度数据，关节数据预处理
    joing_angle[0] = *(sensor_data + 6) + USER_PI; //RF //检查无误
    joing_angle[1] = *(sensor_data + 7); // RB   
    joing_angle[2] = *(sensor_data + 9) + USER_PI; // LF //!反向
    joing_angle[3] = *(sensor_data + 10); //LB      //! 反向

    // 更新轮电机数据
    // ! 更新状态变量x,Dx
    wheel_dataUpdata(0, sensor_data + 8);//RW
    wheel_dataUpdata(1, sensor_data + 11);//LW

    // 更新虚拟腿偏角，虚拟腿长数据
    // ! 更新状态变量theta,Dtheta
    forward_kinematics_solution(0);
    forward_kinematics_solution(1);

    // !debug
    //std::cout << "rx: " << State[0](2) << std::endl;
    //std::cout << "rb joint angle:" << joing_angle[1] << std::endl;
}

/**
/ @brief 计算出电机力矩
/ @param None
/ @retval None
*/
double* Robot_Controller::Get_MotorTorque(){
    
    LQR(0);
    LQR(1);
    get_LegForce();

    // !debug
    //std::cout << UVector[0] << std::endl;

    //* 以下步骤必须在LQR计算完后执行
    //yawSpeedcontrol();
    phiDiffcontrol();
    VMC(0);
    VMC(1);

    //std::cout << "right tk: " << VirtualForce[0](1) << std::endl;
    //std::cout << "State" << State[0] << std::endl;
    //std::cout << "RFM_torque: " << Torque[0] << std::endl;
    //std::cout << "RBM_torque: " << Torque[1] << std::endl;
    std::cout << "right state: " << State[0] << std::endl;
    std::cout << "RFM_torque: " << Torque[0] << std::endl;
    std::cout << "RBM_torque: " << Torque[1] << std::endl;
    std::cout << "RWM_torque: " << Torque[2] << std::endl;
    std::cout << "LWM_torque: " << Torque[5] << std::endl;

    std::cout << "right leg force: " << Leg_Force[0] << std::endl;
    std::cout << "right leg length " << Leg_length[0] << std::endl << "end" << std::endl ;
    //std::cout << "left leg force: " << Leg_Force[1] << std::endl;
    //std::cout << "right tk: " << UVector[0](1) << std::endl;
    //std::cout << "left tk: " << UVector[1](1) << std::endl;
    //std::cout << "right phi: " << temp_phi[0] << std::endl;
    //std::cout << "left phi: " << temp_phi[1] << std::endl << "end" << std::endl;
    /*
        for (int i = 0; i < 6;i++){
            std::cout << Torque[i] << ",  ";
        }
        std::cout << std::endl;
    */
    return Torque;
}


/**
/ @brief 更新电机数据，考虑多圈角度,同时跟新状态变量中的x,Dx
/ @param uint8_t n 表示右轮或左轮
/ @param double* newdata 电机编码器数据指针
/ @retval None
*/
void Robot_Controller::wheel_dataUpdata(uint8_t n,const double* newdata){
    #if(USER_DEBUG == 1)
        assert((n == 0 || n == 1) && "ERROR: n should be 0 or 1 !;<wheel_dataUpdata>");
    #endif

    double temp = *newdata - wheel_angle_single[n];
    
    wheel_angle_single[n] = *newdata;
    if(temp < -USER_PI){
        temp += USER_PI*2;
    }
    else if(temp > USER_PI){
        temp -= USER_PI*2;
    }
    temp *= wheel_radius;       // 换成位移
    // * 更新状态变量中的x,Dx
    State[n](2) += temp;                    //x
    State[n](3) = temp / samplingPeriod;    //Dx
}

/**
/ @brief 五连杆正运动学解算，更新状态变量theta,Dtheta,计算Jaccobian,Rotation,Scale矩阵
/ @param uint8_t n 表示右腿(0)还是左腿(1)
/ @retval None
/ @attention 在更新传感器数据之后再调用
*/
void Robot_Controller::forward_kinematics_solution(uint8_t n){
#if(USER_DEBUG == 1)
    assert(n == 0 || n == 1 && "ERROR: n should be 0 or 1 !;<forward_kinematics>");
#endif

    uint8_t offset = n * 2;
    double phi_1 = joing_angle[0 + offset];
    double phi_4 = joing_angle[1 + offset];
    double xb = link_len[0] * cos(phi_1);               // xb = l1*cos(phi_1)
    double yb = link_len[0] * sin(phi_1);               // yb = l1*sin(phi_1)
    double xd = link_len[4] + link_len[4] * cos(phi_4); // xd = l5 + l4*cos(phi_4)
    double yd = link_len[3] * sin(phi_4);               // yd = l4*sin(phi_4)

    double temp_A = 2 * link_len[1] * (xd - xb);
    double temp_B = 2 * link_len[1] * (yd - yb);
    double temp_C = pow(link_len[1], 2) - pow(link_len[2], 2) + pow((xd - xb), 2) + pow((yd - yb), 2);

    double phi_2 = 2 * atan2((temp_B + sqrt(pow(temp_A, 2) + pow(temp_B, 2) - pow(temp_C, 2))), (temp_A + temp_C));

    double xc = xb + link_len[1] * cos(phi_2);
    double yc = yb + link_len[1] * sin(phi_2);

    double phi_3 = acos((xc - xd)/link_len[2]); // cos(phi_3) = (xc - xd)/l3

    // * 更新虚拟腿长
    Leg_length[n] = sqrt(pow((xc - link_len[4] / 2), 2) + pow(yc, 2));

    // * 更新phi,Dphi
    temp_newphi = atan2(yc, (xc - link_len[4] / 2));
    temp_Dphi = (temp_newphi - temp_phi[n]) / samplingPeriod;
    temp_phi[n] = temp_newphi;
    // ! 计算Jaccobian矩阵
    double temp_J0 = sin(phi_2 - phi_3);
    double temp_J1 = link_len[0] * sin(phi_1 - phi_2) / temp_J0; // l1*sin(phi1-phi2)/sin(phi2-phi3)
    double temp_J2 = link_len[3] * sin(phi_3 - phi_4) / temp_J0; // l4*sin(phi3-phi4)/sin(phi2-phi3)
    Jaccobian[n](0, 0) = sin(phi_3 - temp_phi[n]) * temp_J1;
    Jaccobian[n](0, 1) = sin(phi_2 - temp_phi[n]) * temp_J2;
    Jaccobian[n](1, 0) = -cos(phi_3 - temp_phi[n]) * temp_J1 / Leg_length[n];
    Jaccobian[n](1, 1) = -cos(phi_2 - temp_phi[n]) * temp_J2 / Leg_length[n];
    // * 更新状态变量theta,Dtheta
    State[n](0) = USER_PI / 2 - temp_phi[n] - *(Euler_angle + 1);
    State[n](1) = -temp_Dphi - *(Euler_gyro + 1);
}

/**
/ @brief 根据横滚角计算两个虚构腿的期望腿长,并通过PID计算出支撑力
/ @param None
/ @retval None
/ @attention 请先设置好腿的期望值，在所有数据更新完后之后调用
*/
void Robot_Controller::get_LegForce(){
    // 地面倾角,正数
    double varphi = *Euler_angle + atan2((Leg_length[1] - Leg_length[0]), body_width);

/*
    if(varphi > 0 ){
        leftLegController.set_target(Leg_length_target);
        rightLegController.set_target(Leg_length_target - body_width * tan(varphi));
    }
    else{
        rightLegController.set_target(Leg_length_target); 
        leftLegController.set_target(Leg_length_target + body_width * tan(varphi));
    }
    */
   leftLegController.set_target(Leg_length_target);
   rightLegController.set_target(Leg_length_target);
    //计算两条虚构腿分别需要的支持力
    //! 忘记加重力补偿了,
    rightLegController.control(Leg_length[0], Leg_Force[0]);
    Leg_Force[0] += (M + 0.032*2) * G * cos(State[0](0));
    leftLegController.control(Leg_length[1], Leg_Force[1]);
    Leg_Force[1] += (M + 0.032*2) * G * cos(State[1](0));

    //!debug

}


/**
/ @brief 根据拟合多项式系数计算K增益矩阵
/ @param uint8_t n 表示右腿(0)还是左腿(1)
/ @retval None
/ @attention 请在更新虚构腿长数据之后调用
*/
void Robot_Controller::LQR(uint8_t n){
    #if(USER_DEBUG == 1)
        assert((n == 0 || n == 1) && "ERROR: n should be 0 or 1 !;<LQR>");
    #endif
    // 根据腿长计算K增益矩阵
    //!要修改
        KMatrix[n] = K_polyn[0] * pow(Leg_length[n], 3) + K_polyn[1] * pow(Leg_length[n], 2) + K_polyn[2] * Leg_length[n] + K_polyn[3];
        // 误差
        Eigen::Vector<double, 6> error = State[n] - StateD[n];

        //! 暂时为0
        //error(2) = 0;

        // 计算控制量 
        UVector[n] = -KMatrix[n] * error;

        //std::cout <<  UVector[1] << std::endl;

        // 更新轮电机力矩
        // !这步更新了轮电机力矩，后面的双腿协调控制和转向控制必须要在这步之后进行
        Torque[2 + n * 3] = UVector[n](0); // Tw
};

void Robot_Controller::Load_KData(std::string file_path){
    MatrixDataFile.open(file_path);

    assert(MatrixDataFile.is_open() && "Could not find such file");

    std::string MatrixRow;
    std::string MatrixEntry;
    int MatrixRowNum = 0;
    int MatrixColNum = 0;

    while(getline(MatrixDataFile,MatrixRow)){
        std::stringstream MatrixRowStream(MatrixRow);

        while(getline(MatrixRowStream,MatrixEntry,',')){
            int n=0,num = 0;
            if( MatrixColNum < 6){
                num = MatrixColNum;
                n = 0;
            }
            else if(MatrixColNum < 12){
                num = MatrixColNum-6;
                n = 1;
            }
            else if(MatrixColNum < 18){
                num = MatrixColNum-12;
                n = 2;
            }
            else{
                num = MatrixColNum-18;
                n = 3;
            }
            K_polyn[n](MatrixRowNum,num) = std::stod(MatrixEntry);
            MatrixColNum++;
        }
        MatrixColNum = 0;
        MatrixRowNum++;
    }
}

/**
/ @brief 转向控制
/ @param None
/ @retval None
*/
void Robot_Controller::yawSpeedcontrol(){
    double temp_torque = 0;
    yawspeedPIDController.control(*(Euler_gyro + 2), temp_torque);

    Torque[2] += temp_torque;
    Torque[5] -= temp_torque;
};

/**
/ @brief 双腿协调控制
/ @param None
/ @retval None
*/
void Robot_Controller::phiDiffcontrol(){
    // 被控量
    double diff = temp_phi[0] - temp_phi[1]; //右腿phi减左腿phi
    double temp_torque = 0;
    phiDiffPIDController.control(diff, temp_torque);

    //std::cout << "LB_joint_angle: " << joing_angle[3] << std::endl;
    //std::cout << "theta: " << temp_phi[0] << std::endl;
    // std::cout << "diff: " << diff << "; temp_torque: " << temp_torque << std::endl;


    

    UVector[0](1) -= temp_torque;
    UVector[1](1) += temp_torque;
}

/**
/ @brief  通过VMC方法根据虚构力解算出髋关节电机力矩
/ @param  uint8_t n 表示右腿(0)还是左腿(1)
/ @retval None
/ @attention 在LQR和虚构腿支撑力计算完之后调用
*/
void Robot_Controller::VMC(uint8_t n){
    #if(USER_DEBUG == 1)
        assert((n == 0 || n == 1) && "ERROR: n should be 0 or 1 !;<VMC>");
    #endif

        //!正负
        VirtualForce[n](1) = -UVector[n](1); //Tk
        VirtualForce[n](0) = Leg_Force[n];
        HipForce[n] = Jaccobian[n].transpose() * VirtualForce[n];
        // 传递给关节电机力矩数组
        Torque[0 + n * 3] = HipForce[n](0);
        Torque[1 + n * 3] = HipForce[n](1);

}