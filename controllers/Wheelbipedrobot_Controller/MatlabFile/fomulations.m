 clc;
clear;
% 定义符号变量
syms phi_p x phi t_w t_k r m_w m_p M I_w I_p I_M N_h N_v P_h P_v L1 L2 g;

syms Dx D2x Dphi D2phi Dphi_p D2phi_p

% 参数赋值

param      = [r m_w I_w m_p M I_M g ];
param_real = [65e-3, 0.528, 1121.549e-6, (0.032+0.138)*2, 11.983/2, (110033.523e-6)/2, 9.8 ];
% 列举微分方程
f1 = (I_w/r + m_w*r)*D2x - t_w + N_h*r;% 1.用于消N_h
f2 = N_h - P_h - m_p*D2x + m_p*L1*sin(phi)*Dphi^2 - m_p*L1*cos(phi)*D2phi;% 3.用于消P_h
f3 = N_v - P_v - m_p*g + m_p*L1*cos(phi)*Dphi^2 + m_p*L1*sin(phi)*D2phi;%2.用于消N_v
f4 = I_p*D2phi + t_w - t_k -(N_h*L1+P_h*L2)*sin(phi) + (N_v*L1+P_v*L2)*cos(phi);
f5 = P_h - M*D2x + M*(L1+L2)*sin(phi)*Dphi^2 - M*(L1+L2)*cos(phi)*D2phi;
f6 = P_v - M*g + M*(L1+L2)*cos(phi)*Dphi^2 + M*(L1+L2)*sin(phi)*D2phi;%4.用于消P_v
f7 = I_M*D2phi_p - t_k;


% f1 = (I_w/r+m_w*r)*D2x - (t_w-N_h*r); % 1.用于消N_h
% f2 = -N_h + P_h + m_p*D2x + m_p*L1*D2sinphi;% 3.用于消P_h
% f3 = -N_v + P_v + m_p*g + m_p*L1*D2cosphi;% 2.用于消N_v
% f4 = -I_p*D2phi - t_w + t_k + (N_h*L1+P_h*L2)*sinphi - (N_v*L1+P_v*L2)*cosphi;%
% f5 = -P_h + M*D2x + M*(L1+L2)*D2sinphi;%
% f6 = -P_v + M*g+ M*(L1+L2)*D2cosphi;%4.用于消P_v
% f7 = -I_M*D2phi_p + t_k;

% 消除N_h
temp = solve(f1,N_h);
f2 = subs(f2,N_h,temp);
f4 = subs(f4,N_h,temp);

% 消除N_v
temp = solve(f3,N_v);
f4 = subs(f4,N_v,temp);

% 消除P_h
temp = solve(f2,P_h);
f4 = subs(f4,P_h,temp);
f5 = subs(f5,P_h,temp);

% 消除P_v
temp = solve(f6,P_v);
f4 = subs(f4,P_v,temp);

% 求解 phi x phi_p
q6 = solve(f7,D2phi_p);%解D2phi_p
temp = solve(f4,D2phi);
temp_f = subs(f5,D2phi,temp);
q4 = solve(temp_f,D2x);%解D2x
temp_f = subs(f4,D2x,q4);
q2 = solve(temp_f,D2phi);%解D2phi

% 求雅可比矩阵
A_s = sym('A',[6,6]);
B_s = sym('B',[6,2]);

q = [phi Dphi x Dx phi_p Dphi_p];
u = [t_w t_k];
q_fun = [q2 q4 q6];

for i = [1 3 5]
    B_s(i,:) = [0 0];
    for j = 1:6
        if j == i+1
            A_s(i,j) = 1;
        else 
            A_s(i,j) = 0;
        end
    end
end

for i = [2 4 6]
    for j = 1:6
        A_s(i,j) = diff(q_fun(i/2),q(j));
    end
    for j = 1:2
        B_s(i,j) = diff(q_fun(i/2),u(j));
    end
end

% 求雅可比矩阵在平衡态的值
A_s = subs(A_s,q,[0 0 x 0 0 0]);
A_s = subs(A_s,u,[0 0]);
B_s = subs(B_s,q,[0 0 x 0 0 0]);
B_s = subs(B_s,u,[0 0]);

% 代入实际参数
A_s = subs(A_s,param,param_real);
B_s = subs(B_s,param,param_real);

% 处理变参数
syms L;
var_param   = [L1 L2 I_p];
var_param_f = [L/2 L/2 (0.4+0.6)*2/4*L^2];% I_p = 1/4*m_p*L^2
A_s = subs(A_s,var_param,var_param_f);
B_s = subs(B_s,var_param,var_param_f);

C = eye(6);
D = zeros(6,2);
save('system_matrices.mat','A_s','B_s','C','D','L');