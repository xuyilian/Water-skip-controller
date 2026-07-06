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
trim_start_time = 0;
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
% 1. Online low-pass filtering for R13 / R23 before LMS
% -------------------------------

% Tuned with DataExchange/20260704_161923.mat after removing t < 1.2 s.
% Larger alpha -> less smoothing and faster response.
% Smaller alpha -> stronger smoothing and more delay.
att_alpha = 0.42;
r13_lms_input = zeros(N,1);
r23_lms_input = zeros(N,1);

r13_lms_input(1) = r13_raw(1);
r23_lms_input(1) = r23_raw(1);

for k = 2:N
    r13_lms_input(k) = r13_lms_input(k-1) ...
        + att_alpha * (r13_raw(k) - r13_lms_input(k-1));
    r23_lms_input(k) = r23_lms_input(k-1) ...
        + att_alpha * (r23_raw(k) - r23_lms_input(k-1));
end

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

% Remove isolated yaw-rate spikes before the low-pass filter.
% This is causal: only current and previous samples are used.
yawrate_spike_window = 11;
yawrate_jump_limit = 2500;  % deg/s, relative to recent clean median
yawrate_abs_limit = 5000;   % deg/s, hard physical sanity bound
yawrate_deg_clean = yawrate_deg;

for k = 2:N
    i0 = max(1, k - yawrate_spike_window);
    recent_median = median(yawrate_deg_clean(i0:k-1));

    is_abs_spike = abs(yawrate_deg(k)) > yawrate_abs_limit;
    is_jump_spike = abs(yawrate_deg(k) - recent_median) > yawrate_jump_limit;

    if is_abs_spike || is_jump_spike
        yawrate_deg_clean(k) = recent_median;
    else
        yawrate_deg_clean(k) = yawrate_deg(k);
    end
end

% optional: lightly filter yawrate to reduce noise
yawrate_alpha = 0.05;
yawrate_deg_filt = zeros(N,1);
yawrate_deg_filt(1) = yawrate_deg_clean(1);

for k = 2:N
    yawrate_deg_filt(k) = yawrate_deg_filt(k-1) ...
        + yawrate_alpha * (yawrate_deg_clean(k) - yawrate_deg_filt(k-1));
end

% instantaneous frequency, Hz
f0 = yawrate_deg_filt / 360;

% Start LMS only after the filtered spin rate is high enough.
% Use absolute yaw rate because the spin direction may be negative.
lms_start_yawrate_deg = 500;
lms_start_idx = find(abs(yawrate_deg_filt) >= lms_start_yawrate_deg, 1, 'first');

if isempty(lms_start_idx)
    warning('LMS did not start: |filtered yawrate| never reached %.0f deg/s.', ...
        lms_start_yawrate_deg);
else
    fprintf('LMS starts at t = %.3f s, filtered yawrate = %.1f deg/s.\n', ...
        t(lms_start_idx), yawrate_deg_filt(lms_start_idx));
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
figure('Name','R13 R23 Low Frequency Estimation','NumberTitle','off');

subplot(2,1,1);
plot(t, r13_raw, 'LineWidth', 1.0);
hold on;
plot(t, r13_lms_input, ':', 'LineWidth', 1.2);
plot(t, R13_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R13');
title('R13 Low-frequency Estimation');
legend('R13 raw', 'R13 filtered input', 'A13 LMS output');

subplot(2,1,2);
plot(t, r23_raw, 'LineWidth', 1.0);
hold on;
plot(t, r23_lms_input, ':', 'LineWidth', 1.2);
plot(t, R23_low, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R23');
title('R23 Low-frequency Estimation');
legend('R23 raw', 'R23 filtered input', 'A23 LMS output');

%%
figure('Name','Yaw Rate and Instantaneous Frequency','NumberTitle','off');

subplot(2,1,1);
plot(t, yawrate_deg, 'LineWidth', 1.0);
hold on;
plot(t, yawrate_deg_clean, 'LineWidth', 1.2);
plot(t, yawrate_deg_filt, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
legend('raw', 'despiked', 'filtered');
title('Mocap Yaw Rate');

subplot(2,1,2);
plot(t, f0, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('f0 (Hz)');
title('Instantaneous frequency f0 = yawrate / 360');
