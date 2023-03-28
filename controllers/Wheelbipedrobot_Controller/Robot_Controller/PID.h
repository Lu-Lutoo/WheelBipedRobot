/**
/ @file PID.h
/ @brief 
/ @author 
/ @version 
/ @date 
/ @details 
*/

#pragma one


// 离散PID实现
class PID
{
    public:
        PID(){};
        ~PID(){}

        void control(double, double &);
        void set_target(double);
        void Init(double, double, double, double, double, double);

    private:
        double Kp, Ki, Kd, I_out_Max,TimeStep;
        double target;
        double P_out = 0, I_out = 0, D_out = 0;
        double error = 0, error_preval = 0, error_sum = 0;
};