clc;
clear;

load('system_matrices.mat');
sampleTime = 5e-3;
% 腿长L的范围
[l1, l2, l3, l4, l5] = deal(0.15,0.288, 0.288, 0.15, 0.15);
L_min = 0.18;% sqrt(l2^2 - ((l1+l5+l4)/2)^2)
L_max = 0.42;% l1 + sqrt(l2^2 - (l5/2)^2)

% lqr 参数
Q = diag([20000,1,1,0.1,5000,1]); %phi Dphi x Dx pitch Dpitch
R = diag([0.1,1]); % t_w t_h
l_series = L_min:0.01:L_max;
K_series = zeros(2,6,length(l_series));

% 获取不同L的K增益矩阵
for i = 1:length(l_series)
    l = l_series(i);
    A = subs(A_s,L,l);
    B = subs(B_s,L,l);
    A = double(A);
    B = double(B);
    sys = ss(A,B,C,D);
    sysd = c2d(sys,sampleTime);
    [Temp,S,e] = dlqr(sysd.A,sysd.B,Q,R);
    K_series(:,:,i) = Temp;
end

% 对K增益矩阵重构，每一页是K中(i,j)元素随L的变化数组
K_series_reshape = zeros(1,length(l_series),2*6);

for i = 1:2
    for j = 1:6
        k_temp = K_series(i,j,:);
        K_series_reshape(:,:,(i-1)*6+j) = reshape(k_temp,[1,length(l_series)]);
    end
end

% 拟合多项式，获取多项式系数,拟合阶数是3阶
polyfit_n = 3;
K_polyn = zeros(2,6,polyfit_n+1);
%for i = 1:12
%    K_polyn(:,:,i) = polyfit(l_series,K_series_reshape(:,:,i),polyfit_n);
%end

for i = 1:2
    for j = 1:6
        K_polyn(i,j,:) = polyfit(l_series,K_series_reshape(:,:,(i-1)*6+j),polyfit_n);
    end
end

writematrix(K_polyn,"K_polyn.csv");

fprintf("计算完成\n");






