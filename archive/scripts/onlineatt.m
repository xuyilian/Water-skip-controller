%%
% ==========================================
% Online LMS separation for R13 / R23
% Model:
%   R13 = A13 + B13*sin(phase) + C13*cos(phase)
%   R23 = A23 + B23*sin(phase) + C23*cos(phase)
%
% phase_dot = 2*pi*f0
% f0 is fixed from fixed_yawrate_abs_deg instead of estimated yawrate.
%
% A13, A23 are the estimated low-frequency components
% ==========================================

% Time
trim_start_time = 0;
fixed_yawrate_abs_deg = 2100;
lms_start_yawrate_deg = 500;
raw_t = Abs_time(:);
trim_idx = raw_t >= trim_start_time;

if nnz(trim_idx) < 3
    error('Not enough samples remain after trim_start_time = %.2f s.', trim_start_time);
end

t = raw_t(trim_idx);
dt_vec = [0; diff(t)];
N = length(t);

% Raw signals
r13_all = R13(:);
r23_all = R23(:);
mocap_yaw_deg_all = mocap_yaw_deg(:);

r13_raw = r13_all(trim_idx);
r23_raw = r23_all(trim_idx);
mocap_yaw_deg_trim = mocap_yaw_deg_all(trim_idx);

% -------------------------------
% 1. No prefiltering before LMS
% -------------------------------

r13_lms_input = r13_raw;
r23_lms_input = r23_raw;

% -------------------------------
% 2. Online yaw rate from mocap yaw
% -------------------------------

% unwrap yaw to avoid jump from 180 to -180 deg
yaw_unwrapped_deg = rad2deg(unwrap(deg2rad(mocap_yaw_deg_trim)));

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

% Keep measured yawrate only for diagnosis/start detection. Do not use it
% for the LMS phase frequency.
yawrate_deg_clean = yawrate_deg;

if exist('mocap_yawrate_deg_filt', 'var')
    yawrate_for_sign_all = double(mocap_yawrate_deg_filt(:));
    yawrate_for_sign = yawrate_for_sign_all(trim_idx);
else
    yawrate_for_sign = yawrate_deg;
end

spin_idx = abs(yawrate_for_sign) >= lms_start_yawrate_deg;
if any(spin_idx)
    spin_sign = sign(median(yawrate_for_sign(spin_idx), 'omitnan'));
else
    spin_sign = sign(median(yawrate_for_sign, 'omitnan'));
end
if ~isfinite(spin_sign) || spin_sign == 0
    spin_sign = 1;
end

fixed_yawrate_deg = spin_sign * fixed_yawrate_abs_deg;
yawrate_deg_filt = fixed_yawrate_deg * ones(N,1);

% fixed instantaneous frequency, Hz
f0 = yawrate_deg_filt / 360;

% Start LMS only after the measured spin rate is high enough. The measured
% yawrate is not used for phase after this point.
lms_start_idx = find(abs(yawrate_for_sign) >= lms_start_yawrate_deg, 1, 'first');

if isempty(lms_start_idx)
    warning('LMS did not start: |measured yawrate| never reached %.0f deg/s.', ...
        lms_start_yawrate_deg);
else
    fprintf('LMS starts at t = %.3f s, measured yawrate = %.1f deg/s.\n', ...
        t(lms_start_idx), yawrate_for_sign(lms_start_idx));
    fprintf('Fixed LMS yawrate = %.1f deg/s, f0 = %.3f Hz.\n', ...
        fixed_yawrate_deg, fixed_yawrate_deg / 360);
end

% -------------------------------
% 3. Integrate frequency to phase
% -------------------------------

phase = zeros(N,1);

if ~isempty(lms_start_idx)
    for k = lms_start_idx+1:N
        dt = dt_vec(k);

        % phase_dot = 2*pi*f0
        phase(k) = phase(k-1) + 2*pi*f0(k)*dt;
    end
end

% -------------------------------
% 4. Online LMS estimation
% -------------------------------

% theta13 = [A13; B13; C13]
% theta23 = [A23; B23; C23]
theta13 = [0; 0; 0];
theta23 = [0; 0; 0];

theta13_hist = nan(3,N);
theta23_hist = nan(3,N);

% LMS learning rate
% smaller -> smoother/slower
% larger  -> faster/noisier
mu = 0.02;

% normalized LMS small value
eps_n = 1e-6;

if ~isempty(lms_start_idx)
    theta13 = [r13_lms_input(lms_start_idx); 0; 0];
    theta23 = [r23_lms_input(lms_start_idx); 0; 0];

    for k = lms_start_idx:N

        phi = [1;
               sin(phase(k));
               cos(phase(k))];

        % ---- R13 estimation ----
        r13_hat = theta13' * phi;
        e13 = r13_lms_input(k) - r13_hat;

        % normalized LMS update
        theta13 = theta13 + mu * e13 * phi / (phi' * phi + eps_n);

        % ---- R23 estimation ----
        r23_hat = theta23' * phi;
        e23 = r23_lms_input(k) - r23_hat;

        theta23 = theta23 + mu * e23 * phi / (phi' * phi + eps_n);

        theta13_hist(:,k) = theta13;
        theta23_hist(:,k) = theta23;
    end
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

% Use the LMS A terms directly as the output, without post-smoothing.
R13_low = R13_low_raw;
R23_low = R23_low_raw;

%%
fig_low = figure('Name','R13 R23 Low Frequency Estimation','NumberTitle','off');

subplot(2,1,1);
plot(t, r13_raw, 'LineWidth', 1.0);
hold on;
plot(t, R13_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R13');
title(sprintf('R13 Low-frequency Estimation, fixed yawrate %.0f deg/s', fixed_yawrate_deg));
legend('R13 raw', 'A13 LMS output');

subplot(2,1,2);
plot(t, r23_raw, 'LineWidth', 1.0);
hold on;
plot(t, R23_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R23');
title('R23 Low-frequency Estimation');
legend('R23 raw', 'A23 LMS output');

%%
fig_yawrate = figure('Name','Yaw Rate and Fixed Frequency','NumberTitle','off');

subplot(2,1,1);
plot(t, yawrate_deg, 'LineWidth', 1.0);
hold on;
plot(t, yawrate_for_sign, 'LineWidth', 1.2);
plot(t, yawrate_deg_filt, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
legend('raw from yaw', 'logged/diagnostic', 'fixed for LMS');
title('Measured Yaw Rate and Fixed LMS Yaw Rate');

subplot(2,1,2);
plot(t, f0, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('f0 (Hz)');
title('Fixed frequency f0 = fixed yawrate / 360');

%%
fig_quality = figure('Name','Fixed-yawrate LMS quality','NumberTitle','off');

subplot(2,2,1);
plot(t, r13_raw, 'LineWidth', 0.9); hold on;
plot(t, R13_low, 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('R13');
legend('raw', 'low');

subplot(2,2,2);
plot(t, r23_raw, 'LineWidth', 0.9); hold on;
plot(t, R23_low, 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('R23');
legend('raw', 'low');

subplot(2,2,3);
plot(t, r13_raw - R13_low, 'LineWidth', 0.9); hold on;
plot(t, R13_high_est, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('R13 high');
legend('raw-low', 'estimated high');

subplot(2,2,4);
plot(t, r23_raw - R23_low, 'LineWidth', 0.9); hold on;
plot(t, R23_high_est, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('R23 high');
legend('raw-low', 'estimated high');

output_dir = fullfile(pwd, 'matlab_figures_onlineatt_tuning');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

saveas(fig_low, fullfile(output_dir, 'onlineatt_fixed_2100_low.png'));
saveas(fig_yawrate, fullfile(output_dir, 'onlineatt_fixed_2100_yawrate.png'));
saveas(fig_quality, fullfile(output_dir, 'onlineatt_fixed_2100_quality.png'));
