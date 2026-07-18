%% Water-skip controller flight-log analysis
% Plots the signals saved by revolvinglian.py (DataExchange/*.mat).
%
% Usage:
%   1) Load a log into the workspace, e.g.:
%        load('DataExchange/20260713_140141.mat');
%   2) Run the whole script (Run / F5), OR run any single section on its own --
%      each section bootstraps itself via analyse_prep if needed.
%
% Legends are kept consistent: "(raw)" = as logged, "(filtered)" = onboard/
% estimator filter, "(smoothed)" = extra filtering applied here in MATLAB.

%% ---------- Setup (run this first, or just run the whole script) ----------
% Builds t / dt / Fs, crops to the analysis window, and fills any missing
% signals with NaN. Every plotting section below starts with the same guard
%   if ~exist('t','var'); analyse_prep; end
% so you can also run a single section on its own and it will set itself up.
analyse_prep;

%% ---------- Loop timing (dt) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','Loop timing (dt)','NumberTitle','off');
plot(dt, 'LineWidth', 1.2); grid on;
xlabel('Sample k'); ylabel('dt [s]');
title('Control-loop time step');

%% ---------- Position: mocap raw / filtered / desired ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','Position: mocap vs desired','NumberTitle','off');
tiledlayout(3,1);

% X/Y dashed target = the REAL controller target: the AUTOBI (pool-centre)
% setpoint when logged, falling back to the static desired_x/y for older logs.
if any(~isnan(autobi_desired_x))
    x_target = autobi_desired_x;  y_target = autobi_desired_y;
    xy_target_name = 'x (target: pool/AUTOBI)';
    yy_target_name = 'y (target: pool/AUTOBI)';
else
    x_target = desired_x;  y_target = desired_y;
    xy_target_name = 'x (desired)';
    yy_target_name = 'y (desired)';
end

nexttile;
plot(t, mocap_x_raw,  'LineWidth', 1.0); hold on;
plot(t, mocap_x_filt, 'LineWidth', 1.6);
plot(t, x_target, '--', 'LineWidth', 1.4);
grid on; ylabel('x [m]');
legend('x (raw)','x (filtered)', xy_target_name);
title('Position vs desired');

nexttile;
plot(t, mocap_y_raw,  'LineWidth', 1.0); hold on;
plot(t, mocap_y_filt, 'LineWidth', 1.6);
plot(t, y_target, '--', 'LineWidth', 1.4);
grid on; ylabel('y [m]');
legend('y (raw)','y (filtered)', yy_target_name);

nexttile;
plot(t, mocap_z_raw,  'LineWidth', 1.0); hold on;
plot(t, mocap_z_filt, 'LineWidth', 1.6);
plot(t, desired_z, '--', 'LineWidth', 1.4);
grid on; ylabel('z [m]'); xlabel('Time [s]');
legend('z (raw)','z (filtered)','z (desired)');

%% ---------- Velocity (filtered mocap) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','Mocap velocity (filtered)','NumberTitle','off');
plot(t, mocap_vx_filt, 'LineWidth', 1.4); hold on;
plot(t, mocap_vy_filt, 'LineWidth', 1.4);
plot(t, mocap_vz_filt, 'LineWidth', 1.4);
grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('vx (filtered)','vy (filtered)','vz (filtered)');
title('Filtered mocap velocity');

%% ---------- Battery voltage ----------
% Only present in logs recorded after battery logging was enabled.
% Watch for sag below the 3.2 V low-battery threshold under motor load.
if ~exist('t','var'); analyse_prep; end
if exist('pm_vbat','var')
    figure('Name','Battery voltage','NumberTitle','off');
    plot(t, pm_vbat, 'LineWidth', 1.4); hold on;
    yline(3.2, '--', 'low 3.2 V',      'Color',[0.85 0.4 0.0], 'HandleVisibility','off');
    yline(3.0, ':',  'critical 3.0 V', 'Color',[0.80 0.0 0.0], 'HandleVisibility','off');
    grid on; xlabel('Time [s]'); ylabel('V_{bat} [V]');
    title('Battery voltage (watch for sag under load)');
    legend('v_{bat}','Location','best');
else
    fprintf('Battery plot skipped: this log has no pm_vbat field.\n');
end

%% ---------- Attitude angles (mocap) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','Mocap attitude (roll/pitch/yaw)','NumberTitle','off');
tiledlayout(3,1);

nexttile;
plot(t, mocap_roll_deg, 'LineWidth', 1.4); grid on;
ylabel('Roll [deg]'); title('Mocap Euler angles');

nexttile;
plot(t, mocap_pitch_deg, 'LineWidth', 1.4); grid on;
ylabel('Pitch [deg]');

nexttile;
plot(t, mocap_yaw_deg, 'LineWidth', 1.4); grid on;
ylabel('Yaw [deg]'); xlabel('Time [s]');

%% ---------- Yaw rate: raw / logged filter / MATLAB smoothed ----------
% Extra smoothing applied here so the plotted yaw rate is readable.
% Moving-average window in samples (toolbox-free); ~0.1 s by default.
if ~exist('t','var'); analyse_prep; end
yawrate_smooth_window = max(3, round(0.1 * Fs));
mocap_yawrate_deg_smooth = movmean(mocap_yawrate_deg, yawrate_smooth_window);

figure('Name','Mocap yaw and yaw rate','NumberTitle','off');

yyaxis left;
plot(t, mocap_yaw_deg, 'LineWidth', 1.4);
ylabel('Yaw [deg]');

yyaxis right;
plot(t, mocap_yawrate_deg, 'LineWidth', 1.0); hold on;
plot(t, mocap_yawrate_deg_filt,   'LineWidth', 1.6);
plot(t, mocap_yawrate_deg_smooth, 'LineWidth', 2.0);
ylabel('Yaw rate [deg/s]');

grid on; xlabel('Time [s]');
title('Mocap yaw and yaw rate');
legend('yaw', ...
       'yaw rate (raw)', ...
       'yaw rate (logged filter)', ...
       sprintf('yaw rate (smoothed, %d-pt)', yawrate_smooth_window));

%% ---------- Onboard (sent) yaw vs mocap yaw ----------
if ~exist('t','var'); analyse_prep; end
if ~exist('mocap_yawrate_deg_smooth','var')
    mocap_yawrate_deg_smooth = movmean(mocap_yawrate_deg, max(3, round(0.1 * Fs)));
end
figure('Name','Onboard vs mocap yaw / yaw rate','NumberTitle','off');
tiledlayout(2,1);

nexttile;
plot(t, mocap_yaw_deg, 'LineWidth', 1.5); hold on;
plot(t, yaw_deg, '--',  'LineWidth', 1.5);
grid on; ylabel('Yaw [deg]');
legend('yaw (mocap)','yaw (onboard)');
title('Yaw angle comparison');

nexttile;
plot(t, mocap_yawrate_deg_filt,   'LineWidth', 1.6); hold on;
plot(t, mocap_yawrate_deg_smooth, 'LineWidth', 1.4);
grid on; xlabel('Time [s]'); ylabel('Yaw rate [deg/s]');
legend('yaw rate (logged filter)','yaw rate (smoothed)');
title('Yaw rate (filtered vs smoothed)');

%% ---------- Setpoint commands ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','Commands: roll/pitch/yaw/thrust','NumberTitle','off');
tiledlayout(4,1);

nexttile; plot(t, double(cmd_roll),   'LineWidth', 1.4); grid on; ylabel('roll');
title('Commands sent to Crazyflie');
nexttile; plot(t, double(cmd_pitch),  'LineWidth', 1.4); grid on; ylabel('pitch');
nexttile; plot(t, double(cmd_yaw),    'LineWidth', 1.4); grid on; ylabel('yaw rate');
nexttile; plot(t, double(cmd_thrust), 'LineWidth', 1.4); grid on; ylabel('thrust');
xlabel('Time [s]');

%% ---------- R13 / R23 tracking (raw vs filtered) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','R13/R23 tracking','NumberTitle','off');
tiledlayout(2,1);

nexttile;
plot(t, R13,      'LineWidth', 1.0); hold on;
plot(t, R13_filt, 'LineWidth', 1.8);
grid on; ylabel('R13'); title('R13 tracking');
legend('R13 (raw)','R13 (filtered)');

nexttile;
plot(t, R23,      'LineWidth', 1.0); hold on;
plot(t, R23_filt, 'LineWidth', 1.8);
grid on; ylabel('R23'); xlabel('Time [s]'); title('R23 tracking');
legend('R23 (raw)','R23 (filtered)');

%% ---------- R13 / R23 derivatives (raw vs filtered) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','R13/R23 derivatives','NumberTitle','off');
tiledlayout(2,1);

nexttile;
plot(t, R13_d,      'LineWidth', 1.0); hold on;
plot(t, R13_d_filt, 'LineWidth', 1.8);
grid on; ylabel('dR13/dt');
legend('R13 rate (raw)','R13 rate (filtered)');
title('R13 / R23 time derivatives');

nexttile;
plot(t, R23_d,      'LineWidth', 1.0); hold on;
plot(t, R23_d_filt, 'LineWidth', 1.8);
grid on; ylabel('dR23/dt'); xlabel('Time [s]');
legend('R23 rate (raw)','R23 rate (filtered)');

%% ---------- 3D trajectory (mocap) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','3D trajectory (mocap)','NumberTitle','off');
plot3(mocap_x_raw, mocap_y_raw, mocap_z_raw, 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Mocap 3D trajectory');

%% ---------- 2D trajectory (mocap) ----------
if ~exist('t','var'); analyse_prep; end
figure('Name','2D trajectory (mocap)','NumberTitle','off');
plot(mocap_x_raw, mocap_y_raw, 'LineWidth', 1.5); hold on;
plot(mocap_x_raw(1),   mocap_y_raw(1),   'o', 'MarkerSize', 8,  'LineWidth', 1.5);
plot(mocap_x_raw(end), mocap_y_raw(end), 'x', 'MarkerSize', 10, 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Mocap 2D trajectory');
legend('trajectory','start','end');

%% ==========================================================
%  Segment analysis around the water-skip event
%  Segment = first sample where the filtered yaw rate drops below a
%  threshold, extended by a fixed duration. Raw/filtered signals and their
%  1st/2nd derivatives are plotted for X, Y, R13 and R23.
%  ==========================================================
if ~exist('t','var'); analyse_prep; end
t_all = Abs_time(:);

% Segment definition.
yawrate_threshold        = -2800;   % deg/s
duration_after_threshold = 3;       % seconds

idx_start = find(mocap_yawrate_deg_filt(:) < yawrate_threshold, 1, 'first');
if isempty(idx_start)
    % No water-skip event in this log (e.g. short / aborted run). Skip the
    % segment plots but leave all earlier figures intact.
    warning(['Segment analysis skipped: filtered yaw rate never drops below ' ...
        '%.0f deg/s in this log.'], yawrate_threshold);
    return
end

seg_t_start = t_all(idx_start);
seg_t_end   = seg_t_start + duration_after_threshold;
idx_seg     = (t_all >= seg_t_start) & (t_all <= seg_t_end);

if nnz(idx_seg) < 5
    warning(['Segment analysis skipped: only %d sample(s) in the %.1f s window ' ...
        '(event too close to the end of the log).'], nnz(idx_seg), duration_after_threshold);
    return
end

% Segment time, reset to start at 0.
ts = t_all(idx_seg) - seg_t_start;
fprintf('Segment: %.3f s to %.3f s, duration %.3f s\n', ...
    seg_t_start, seg_t_end, ts(end));

% Cut signals (force column vectors).
x_raw  = mocap_x_raw(idx_seg).';
x_filt = mocap_x_filt(idx_seg).';
y_raw  = mocap_y_raw(idx_seg).';
y_filt = mocap_y_filt(idx_seg).';

r13_raw  = R13(idx_seg).';
r13_filt = R13_filt(idx_seg).';
r23_raw  = R23(idx_seg).';
r23_filt = R23_filt(idx_seg).';

vx_filt_seg = mocap_vx_filt(idx_seg).';
vy_filt_seg = mocap_vy_filt(idx_seg).';

r13_d_filt_seg = R13_d_filt(idx_seg).';
r23_d_filt_seg = R23_d_filt(idx_seg).';

yawrate_filt_seg = mocap_yawrate_deg_filt(idx_seg).';

% Smoothing window for derivatives computed here.
deriv_smooth_window = 11;

% First derivatives.
x_filt_d = movmean(gradient(x_filt, ts), deriv_smooth_window);
y_filt_d = movmean(gradient(y_filt, ts), deriv_smooth_window);

% Second derivatives.
x_filt_dd        = gradient(x_filt_d, ts);
x_filt_dd_smooth = movmean(x_filt_dd, deriv_smooth_window);
y_filt_dd        = gradient(y_filt_d, ts);
y_filt_dd_smooth = movmean(y_filt_dd, deriv_smooth_window);

r13_d_filt_smooth = movmean(r13_d_filt_seg, deriv_smooth_window);
r23_d_filt_smooth = movmean(r23_d_filt_seg, deriv_smooth_window);
r13_dd_smooth     = movmean(gradient(r13_d_filt_smooth, ts), deriv_smooth_window);
r23_dd_smooth     = movmean(gradient(r23_d_filt_smooth, ts), deriv_smooth_window);

% ---------- Segment yaw rate ----------
figure('Name','Segment: yaw rate','NumberTitle','off');
plot(ts, yawrate_filt_seg, 'LineWidth', 1.5); grid on;
xlabel('Time after threshold [s]'); ylabel('Yaw rate (filtered) [deg/s]');
title(sprintf('Segment: %.2f s to %.2f s', seg_t_start, seg_t_end));

% ---------- Segment X ----------
figure('Name','Segment: X and derivatives','NumberTitle','off');
tiledlayout(3,1);

nexttile;
plot(ts, x_raw,  'LineWidth', 1.0); hold on;
plot(ts, x_filt, 'LineWidth', 1.6);
grid on; ylabel('X [m]'); title('Mocap X');
legend('x (raw)','x (filtered)');

nexttile;
plot(ts, x_filt_d,    'LineWidth', 1.6); hold on;
plot(ts, vx_filt_seg, 'LineWidth', 1.2);
grid on; ylabel('dX/dt [m/s]');
legend('x rate (from filtered X)','vx (logged filter)');

nexttile;
plot(ts, x_filt_dd,        'LineWidth', 1.0); hold on;
plot(ts, x_filt_dd_smooth, 'LineWidth', 1.8);
grid on; ylabel('d^2X/dt^2 [m/s^2]'); xlabel('Time after threshold [s]');
legend('x accel','x accel (smoothed)');

% ---------- Segment Y ----------
figure('Name','Segment: Y and derivatives','NumberTitle','off');
tiledlayout(3,1);

nexttile;
plot(ts, y_raw,  'LineWidth', 1.0); hold on;
plot(ts, y_filt, 'LineWidth', 1.6);
grid on; ylabel('Y [m]'); title('Mocap Y');
legend('y (raw)','y (filtered)');

nexttile;
plot(ts, y_filt_d,    'LineWidth', 1.6); hold on;
plot(ts, vy_filt_seg, 'LineWidth', 1.2);
grid on; ylabel('dY/dt [m/s]');
legend('y rate (from filtered Y)','vy (logged filter)');

nexttile;
plot(ts, y_filt_dd,        'LineWidth', 1.0); hold on;
plot(ts, y_filt_dd_smooth, 'LineWidth', 1.8);
grid on; ylabel('d^2Y/dt^2 [m/s^2]'); xlabel('Time after threshold [s]');
legend('y accel','y accel (smoothed)');

% ---------- Segment R13 ----------
figure('Name','Segment: R13 and derivatives','NumberTitle','off');
tiledlayout(3,1);

nexttile;
plot(ts, r13_raw,  'LineWidth', 1.0); hold on;
plot(ts, r13_filt, 'LineWidth', 1.6);
grid on; ylabel('R13'); title('R13');
legend('R13 (raw)','R13 (filtered)');

nexttile;
plot(ts, r13_d_filt_seg,    'LineWidth', 1.0); hold on;
plot(ts, r13_d_filt_smooth, 'LineWidth', 1.6);
grid on; ylabel('dR13/dt');
legend('R13 rate (logged filter)','R13 rate (smoothed)');

nexttile;
plot(ts, r13_dd_smooth, 'LineWidth', 1.6);
grid on; ylabel('d^2R13/dt^2'); xlabel('Time after threshold [s]');
legend('R13 accel (smoothed)');

% ---------- Segment R23 ----------
figure('Name','Segment: R23 and derivatives','NumberTitle','off');
tiledlayout(3,1);

nexttile;
plot(ts, r23_raw,  'LineWidth', 1.0); hold on;
plot(ts, r23_filt, 'LineWidth', 1.6);
grid on; ylabel('R23'); title('R23');
legend('R23 (raw)','R23 (filtered)');

nexttile;
plot(ts, r23_d_filt_seg,    'LineWidth', 1.0); hold on;
plot(ts, r23_d_filt_smooth, 'LineWidth', 1.6);
grid on; ylabel('dR23/dt');
legend('R23 rate (logged filter)','R23 rate (smoothed)');

nexttile;
plot(ts, r23_dd_smooth, 'LineWidth', 1.6);
grid on; ylabel('d^2R23/dt^2'); xlabel('Time after threshold [s]');
legend('R23 accel (smoothed)');
