%%
% ==========================================
% Online LMS separation for mocap_x_raw / mocap_y_raw
% Model:
%   mocap_x_raw = Ax + Bx*sin(phase) + Cx*cos(phase)
%   mocap_y_raw = Ay + By*sin(phase) + Cy*cos(phase)
%
% phase_dot = 2*pi*f0
% f0 = yawrate_deg / 360
%
% Ax, Ay are the estimated low-frequency components
% ==========================================

% Time
t = Abs_time(:);
dt_vec = [0; diff(t)];
N = length(t);

% Raw signals
x_raw = mocap_x_raw(:);
y_raw = mocap_y_raw(:);

% -------------------------------
% 1. Online yaw rate from mocap yaw
% -------------------------------

% unwrap yaw to avoid jump from 180 to -180 deg
yaw_unwrapped_deg = rad2deg(unwrap(deg2rad(mocap_yaw_deg(:))));

% yaw rate in deg/s
yawrate_deg = zeros(N,1);
for k = 2:N
    dt = dt_vec(k);
    if dt > 1e-6
        yawrate_deg(k) = (yaw_unwrapped_deg(k) - yaw_unwrapped_deg(k-1)) / dt;
    else
        yawrate_deg(k) = yawrate_deg(k-1);
    end
end

% optional: lightly filter yawrate to reduce noise
yawrate_alpha = 0.1;
yawrate_deg_filt = zeros(N,1);
yawrate_deg_filt(1) = yawrate_deg(1);

for k = 2:N
    yawrate_deg_filt(k) = yawrate_deg_filt(k-1) ...
        + yawrate_alpha * (yawrate_deg(k) - yawrate_deg_filt(k-1));
end

% instantaneous frequency, Hz
f0 = yawrate_deg_filt / 360;

% -------------------------------
% 2. Integrate frequency to phase
% -------------------------------

phase = zeros(N,1);

for k = 2:N
    dt = dt_vec(k);

    % phase_dot = 2*pi*f0
    phase(k) = phase(k-1) + 2*pi*f0(k)*dt;
end

% -------------------------------
% 3. Online LMS estimation
% -------------------------------

% thetax = [Ax; Bx; Cx]
% thetay = [Ay; By; Cy]
thetax = [0; 0; 0];
thetay = [0; 0; 0];

thetax_hist = zeros(3,N);
thetay_hist = zeros(3,N);

% LMS learning rate
mu = 0.3;

% normalized LMS small value
eps_n = 1e-6;

for k = 1:N

    phi = [1;
           sin(phase(k));
           cos(phase(k))];

    % ---- X estimation ----
    x_hat = thetax' * phi;
    ex = x_raw(k) - x_hat;

    thetax = thetax + mu * ex * phi / (phi' * phi + eps_n);

    % ---- Y estimation ----
    y_hat = thetay' * phi;
    ey = y_raw(k) - y_hat;

    thetay = thetay + mu * ey * phi / (phi' * phi + eps_n);

    thetax_hist(:,k) = thetax;
    thetay_hist(:,k) = thetay;
end

% Low-frequency components
mocap_x_low_raw = thetax_hist(1,:)';
mocap_y_low_raw = thetay_hist(1,:)';

% Estimated high-frequency coefficients
mocap_x_B = thetax_hist(2,:)';
mocap_x_C = thetax_hist(3,:)';

mocap_y_B = thetay_hist(2,:)';
mocap_y_C = thetay_hist(3,:)';

% Reconstructed high-frequency parts
mocap_x_high_est = mocap_x_B .* sin(phase) + mocap_x_C .* cos(phase);
mocap_y_high_est = mocap_y_B .* sin(phase) + mocap_y_C .* cos(phase);

% Optional: lightly smooth A estimate online-style
A_alpha = 0.3;
mocap_x_low = zeros(N,1);
mocap_y_low = zeros(N,1);

mocap_x_low(1) = mocap_x_low_raw(1);
mocap_y_low(1) = mocap_y_low_raw(1);

for k = 2:N
    mocap_x_low(k) = mocap_x_low(k-1) + A_alpha * (mocap_x_low_raw(k) - mocap_x_low(k-1));
    mocap_y_low(k) = mocap_y_low(k-1) + A_alpha * (mocap_y_low_raw(k) - mocap_y_low(k-1));
end

%%
figure('Name','Mocap X Y Low Frequency Estimation','NumberTitle','off');

subplot(2,1,1);
plot(t, x_raw, 'LineWidth', 1.0);
hold on;
plot(t, mocap_x_low_raw, '--', 'LineWidth', 1.2);
plot(t, mocap_x_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('X (m)');
title('Mocap X Low-frequency Estimation');
legend('mocap x raw', 'Ax LMS raw', 'Ax smoothed');

subplot(2,1,2);
plot(t, y_raw, 'LineWidth', 1.0);
hold on;
plot(t, mocap_y_low_raw, '--', 'LineWidth', 1.2);
plot(t, mocap_y_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Y (m)');
title('Mocap Y Low-frequency Estimation');
legend('mocap y raw', 'Ay LMS raw', 'Ay smoothed');

%%
figure('Name','Mocap XY Raw vs Low Frequency Trajectory','NumberTitle','off');

plot(x_raw, y_raw, 'LineWidth', 1.0);
hold on;
plot(mocap_x_low, mocap_y_low, 'LineWidth', 2.0);

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
title('Mocap XY Trajectory');
legend('raw trajectory', 'low-frequency trajectory');

%%
figure('Name','Yaw Rate and Instantaneous Frequency','NumberTitle','off');

subplot(2,1,1);
plot(t, yawrate_deg, 'LineWidth', 1.0);
hold on;
plot(t, yawrate_deg_filt, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
legend('raw', 'filtered');
title('Mocap Yaw Rate');

subplot(2,1,2);
plot(t, f0, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('f0 (Hz)');
title('Instantaneous frequency f0 = yawrate / 360');