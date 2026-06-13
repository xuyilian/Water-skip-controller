clear; clc; close all;

%% ===============================
% 1. 参数设置
% ===============================

fs = 1000;              % 采样频率 Hz
T  = 10;                % 信号时长 s
t  = (0:1/fs:T)';       % 时间向量
N  = length(t);

f0 = 50;                % 高频正弦频率 Hz
w  = 2*pi*f0;           % 角频率 rad/s

%% ===============================
% 2. 构造真实信号
% y(t) = A(t) + B sin(wt) + C cos(wt)
% ===============================

% 低频分量 A(t)，假设它缓慢变化
A_true = 1.0 ...
       + 0.4*sin(2*pi*0.3*t) ...
       + 0.2*cos(2*pi*0.1*t);

% 高频分量参数
B_true = 0.8;
C_true = -0.5;

% 噪声
noise_std = 0.15;
noise = noise_std * randn(N,1);

% 测量信号
y = A_true ...
  + B_true*sin(w*t) ...
  + C_true*cos(w*t) ...
  + noise;

%% ===============================
% 3. 基于梯度下降的在线参数估计
%
% 模型：
% y_hat(k) = A_hat(k) + B_hat(k) sin(wt_k) + C_hat(k) cos(wt_k)
%
% 参数向量：
% theta = [A; B; C]
%
% 回归向量：
% phi = [1; sin(wt); cos(wt)]
%
% 误差：
% e = y - y_hat
%
% 梯度下降更新：
% theta(k+1) = theta(k) + mu * e * phi
%
% ===============================

theta_hat = zeros(3,N);     % 存储估计值 [A; B; C]
theta = [0; 0; 0];          % 初始估计

mu = 0.01;                  % 学习率，需要根据采样率和信号幅值调整

for k = 1:N
    
    phi = [1;
           sin(w*t(k));
           cos(w*t(k))];
    
    y_hat = theta' * phi;
    
    e = y(k) - y_hat;
    
    % 梯度下降 / LMS 更新
    theta = theta + mu * e * phi;
    
    theta_hat(:,k) = theta;
end

A_est_raw = theta_hat(1,:)';
B_est     = theta_hat(2,:)';
C_est     = theta_hat(3,:)';

%% ===============================
% 4. 对 A 的估计结果进行低通滤波
% ===============================

fc = 2;   % 低通截止频率 Hz，应高于 A(t) 的变化频率，低于高频 f0

[b,a] = butter(4, fc/(fs/2), 'low');

% 使用 filtfilt 做零相位滤波
A_est_filtered = filtfilt(b,a,A_est_raw);

%% ===============================
% 5. 绘图
% ===============================

figure;

subplot(4,1,1);
plot(t, y, 'k');
grid on;
xlabel('Time / s');
ylabel('y(t)');
title('测量信号');

subplot(4,1,2);
plot(t, A_true, 'b', 'LineWidth', 1.5); hold on;
plot(t, A_est_raw, 'r');
grid on;
xlabel('Time / s');
ylabel('A');
legend('真实 A(t)', '梯度下降估计 A');
title('A 的原始估计');

subplot(4,1,3);
plot(t, A_true, 'b', 'LineWidth', 1.5); hold on;
plot(t, A_est_filtered, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time / s');
ylabel('A');
legend('真实 A(t)', '滤波后估计 A');
title('滤波后 A 随时间变化曲线');

subplot(4,1,4);
plot(t, B_est, 'r', 'LineWidth', 1.2); hold on;
plot(t, C_est, 'g', 'LineWidth', 1.2);
yline(B_true, '--r', '真实 B');
yline(C_true, '--g', '真实 C');
grid on;
xlabel('Time / s');
ylabel('B, C');
legend('估计 B', '估计 C', '真实 B', '真实 C');
title('B 和 C 的估计结果');