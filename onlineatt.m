%%
% ==========================================
% Online LMS separation for R13 / R23
% Model:
%   R13 = A13 + B13*sin(phase) + C13*cos(phase)
%   R23 = A23 + B23*sin(phase) + C23*cos(phase)
%
% phase_dot = 2*pi*f0
% f0 = yawrate_deg / 360
%
% A13, A23 are the estimated low-frequency components
% ==========================================

% Time
t = Abs_time(:);
dt_vec = [0; diff(t)];
N = length(t);

% Raw signals
r13_raw = R13(:);
r23_raw = R23(:);

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

% theta13 = [A13; B13; C13]
% theta23 = [A23; B23; C23]
theta13 = [0; 0; 0];
theta23 = [0; 0; 0];

theta13_hist = zeros(3,N);
theta23_hist = zeros(3,N);

% LMS learning rate
% smaller -> smoother/slower
% larger  -> faster/noisier
mu = 0.3;

% normalized LMS small value
eps_n = 1e-6;

for k = 1:N

    phi = [1;
           sin(phase(k));
           cos(phase(k))];

    % ---- R13 estimation ----
    r13_hat = theta13' * phi;
    e13 = r13_raw(k) - r13_hat;

    % normalized LMS update
    theta13 = theta13 + mu * e13 * phi / (phi' * phi + eps_n);

    % ---- R23 estimation ----
    r23_hat = theta23' * phi;
    e23 = r23_raw(k) - r23_hat;

    theta23 = theta23 + mu * e23 * phi / (phi' * phi + eps_n);

    theta13_hist(:,k) = theta13;
    theta23_hist(:,k) = theta23;
end

% Low-frequency components
R13_low_raw = theta13_hist(1,:)';
R23_low_raw = theta23_hist(1,:)';

% Estimated high-frequency coefficients
R13_B = theta13_hist(2,:)';
R13_C = theta13_hist(3,:)';

R23_B = theta23_hist(2,:)';
R23_C = theta23_hist(3,:)';

% Reconstructed high-frequency parts
R13_high_est = R13_B .* sin(phase) + R13_C .* cos(phase);
R23_high_est = R23_B .* sin(phase) + R23_C .* cos(phase);

% Optional: lightly smooth A estimate online-style
A_alpha = 0.3;
R13_low = zeros(N,1);
R23_low = zeros(N,1);

R13_low(1) = R13_low_raw(1);
R23_low(1) = R23_low_raw(1);

for k = 2:N
    R13_low(k) = R13_low(k-1) + A_alpha * (R13_low_raw(k) - R13_low(k-1));
    R23_low(k) = R23_low(k-1) + A_alpha * (R23_low_raw(k) - R23_low(k-1));
end

%%
figure('Name','R13 R23 Low Frequency Estimation','NumberTitle','off');

subplot(2,1,1);
plot(t, r13_raw, 'LineWidth', 1.0);
hold on;
plot(t, R13_low_raw, '--', 'LineWidth', 1.2);
plot(t, R13_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R13');
title('R13 Low-frequency Estimation');
legend('R13 raw', 'A13 LMS raw', 'A13 smoothed');

subplot(2,1,2);
plot(t, r23_raw, 'LineWidth', 1.0);
hold on;
plot(t, R23_low_raw, '--', 'LineWidth', 1.2);
plot(t, R23_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R23');
title('R23 Low-frequency Estimation');
legend('R23 raw', 'A23 LMS raw', 'A23 smoothed');

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

