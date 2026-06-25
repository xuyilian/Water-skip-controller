%% ==== 画图（适配 STQ_leg_meas_demo_stance_time.py 当前保存字段） ====

t = Abs_time(:);

% 选时间窗口
t_start = 0;
t_end   = inf;
idx = (t >= t_start) & (t <= t_end);

% 统一裁剪：对 workspace 里所有“长度等于 t 的向量”裁剪
vars = whos;
for k = 1:numel(vars)
    name = vars(k).name;
    v = eval(name);
    if isnumeric(v) && isvector(v) && numel(v) == numel(t)
        eval([name ' = ' name '(idx);']);
    end
end
t = t(idx);

% dt 分布
dt = diff(t);
figure; plot(dt); grid on; title('dt 分布'); xlabel('k'); ylabel('dt [s]');

%% ---------- Plot each axis separately ----------
figure('Name', 'Acceleration components', 'NumberTitle', 'off');
subplot(3,1,1);
plot(Abs_time, acc_x, 'LineWidth', 1.2);
grid on;
ylabel('acc.x');
title('Acceleration X');
subplot(3,1,2)
plot(Abs_time, acc_y, 'LineWidth', 1.2);
grid on;
ylabel('acc.y');
title('Acceleration Y');
subplot(3,1,3);
plot(Abs_time, acc_z, 'LineWidth', 1.2);
grid on;
ylabel('acc.z');
xlabel('Time [s]');
title('Acceleration Z');

%% ==== 1) 电池电压（如果有） ====
    figure('Name','Battery voltage','NumberTitle','off');
    plot(t, double(pm_vbat)); grid on;
    xlabel('Time [s]'); ylabel('Vbat');
    title('pm.vbat');

%% ==== 2) 位置：mocap vs desired ====

    figure('Name','Mocap position vs Desired','NumberTitle','off');
    tiledlayout(3,1);

    nexttile;
    plot(t, mocap_x_raw, 'LineWidth',1.2); hold on;
    %plot(t, desired_x, '--', 'LineWidth',1.2);
    ylabel('x [m]'); grid on; legend('mocap\_x','desired\_x');

    nexttile;
    plot(t, mocap_y_raw, 'LineWidth',1.2); hold on;
    %plot(t, desired_y, '--', 'LineWidth',1.2);
    ylabel('y [m]'); grid on; legend('mocap\_y','desired\_y');

    nexttile;
    plot(t, mocap_z_raw, 'LineWidth',1.2); hold on;
    %plot(t, desired_z, '--', 'LineWidth',1.2);
    ylabel('z [m]'); xlabel('Time [s]'); grid on; legend('mocap\_z','desired\_z');


%% ==== 3) 动捕速度（注意：你当前代码 mocap_vz 保存的是 0.0） ====

    figure('Name','Mocap velocity','NumberTitle','off');
    plot(Abs_time, mocap_vx, 'LineWidth',1.2); hold on;
    plot(Abs_time, mocap_vy, 'LineWidth',1.2);
    plot(Abs_time, mocap_vz, 'LineWidth',1.2);
    grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]');
    legend('mocap\_vx','mocap\_vy','mocap\_vz');
    title('Note: current logger sets mocap\_vz to 0.0 in script');


%% ==== 4) 发送给 Crazyflie 的姿态/推力指令 ====

    figure('Name','Setpoint: roll/pitch/yaw/thrust','NumberTitle','off');
    tiledlayout(4,1);

    nexttile; plot(t, double(cmd_roll), 'LineWidth',1.2); grid on; ylabel('roll'); 
    nexttile; plot(t, double(cmd_pitch),'LineWidth',1.2); grid on; ylabel('pitch');
    nexttile; plot(t, double(cmd_yaw),  'LineWidth',1.2); grid on; ylabel('yawrate'); 
    nexttile; plot(t, double(cmd_thrust),'LineWidth',1.2); grid on; ylabel('thrust'); xlabel('Time [s]');

    %% ==== 4) 发送给 Crazyflie 的姿态/推力指令 ====

    figure('Name','Setpoint: roll/pitch/yaw/thrust','NumberTitle','off');
    tiledlayout(3,1);

% roll: cmd_roll vs bi_roll_flight
nexttile;
plot(t, double(cmd_roll), 'LineWidth', 1.2); hold on;
plot(t, double(bi_roll_flight), '--', 'LineWidth', 1.2);
grid on; ylabel('roll');
legend('cmd\_roll','bi\_roll\_flight');

% pitch: 仍然单独画 bi_pitch_flight（如你需要也可叠 cmd_pitch）
nexttile;
plot(t, double(cmd_pitch),'LineWidth',1.2); hold on;
plot(t, double(bi_pitch_flight), 'LineWidth', 1.2);
grid on; ylabel('pitch');
legend('cmd\_pitch','bi\_pitch\_flight');

% thrust
nexttile;
plot(t, double(cmd_thrust), 'LineWidth', 1.2);
grid on; ylabel('thrust'); xlabel('Time [s]');
legend('cmd\_thrust');
%% ==== 5) GeoController 输出 U_X/U_Y/U_Z + U_yaw ====

    figure('Name','Controller outputs: U_X/U_Y/U_Z/U_yaw','NumberTitle','off');
    tiledlayout(4,1);

    nexttile; plot(t, double(U_X), 'LineWidth',1.2); grid on; ylabel('U\_X');
    nexttile; plot(t, double(U_Y), 'LineWidth',1.2); grid on; ylabel('U\_Y');
    nexttile; plot(t, double(U_Z), 'LineWidth',1.2); grid on; ylabel('U\_Z');
    nexttile; plot(t, double(U_yaw),'LineWidth',1.2); grid on; ylabel('U\_yaw'); xlabel('Time [s]');


%% ==== 6) R13/R23 及导数（来自 RealTimeProcessor） ====

    figure('Name','R13/R23 and derivatives','NumberTitle','off');
    tiledlayout(2,1);

    nexttile;
    plot(t, double(R13), 'LineWidth',1.2); hold on;
    plot(t, double(R23), 'LineWidth',1.2);
    grid on; ylabel('R'); legend('R13','R23');

    nexttile;
    plot(t, double(R13_d), 'LineWidth',1.2); hold on;
    plot(t, double(R23_d), 'LineWidth',1.2);
    grid on; ylabel('dR/dt'); xlabel('Time [s]'); legend('R13\_d','R23\_d');



%% ==== 8) 3D 轨迹 ====

    figure('Name','XYZ 3D Trajectory (mocap)','NumberTitle','off');
    plot3(mocap_x_raw, mocap_y_raw, mocap_z_raw, 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Mocap 3D Trajectory');

%%
%%
figure('Name','XY 2D Trajectory (mocap)','NumberTitle','off');

plot(mocap_x_raw, mocap_y_raw, 'LineWidth', 1.5);
hold on;

target_time = 10.07;

[~, idx_1007] = min(abs(Abs_time - target_time));

plot(mocap_x_raw(idx_1007), mocap_y_raw(idx_1007), 'ro', 'MarkerSize', 8, 'LineWidth', 2);    
plot(mocap_x_raw(1), mocap_y_raw(1), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(mocap_x_raw(end), mocap_y_raw(end), 'x', 'MarkerSize', 10, 'LineWidth', 1.5);

grid on;
axis equal;

xlabel('X (m)');
ylabel('Y (m)');
title('Mocap 2D Trajectory');
legend('Trajectory', 'enable','Start', 'End');
%% ==== Plot cmd vs IMU vs Mocap as subplots (Roll/Pitch) ====

% crop + cast
cmd_roll = double(cmd_roll(idx));
cmd_pitch = double(cmd_pitch(idx));

imu_roll_deg = double(imu_roll_deg(idx));
imu_pitch_deg = double(imu_pitch_deg(idx));

mocap_roll_deg = double(mocap_roll_deg(idx));
mocap_pitch_deg = double(mocap_pitch_deg(idx));

figure('Name','Cmd vs IMU vs Mocap (Roll/Pitch)','NumberTitle','off');
tiledlayout(2,1);

% ---- Roll ----
nexttile;
plot(t, cmd_roll, 'LineWidth', 1.2); hold on;
plot(t, imu_roll_deg, '--', 'LineWidth', 1.2);
plot(t, mocap_roll_deg, ':', 'LineWidth', 1.6);
grid on; ylabel('Roll [deg]');
legend('cmd\_roll','imu\_roll','mocap\_roll');

% ---- Pitch ----
nexttile;
plot(t, cmd_pitch, 'LineWidth', 1.2); hold on;
plot(t, imu_pitch_deg, '--', 'LineWidth', 1.2);
plot(t, mocap_pitch_deg, ':', 'LineWidth', 1.6);
grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
legend('cmd\_pitch','imu\_pitch','mocap\_pitch');
%% 
figure('Name','Battery voltage','NumberTitle','off');
    plot(t, mocap_yawrate_dps); grid on;
    xlabel('Time [s]'); ylabel('Vbat');
    title('pm.vbat');
%% 
figure('Name','Firmware control (ctr_roll/pitch/yaw)','NumberTitle','off');
tiledlayout(3,1);

% --- roll ---
nexttile;
plot(t, controller_ctr_roll, 'LineWidth',1.2); hold on;
plot(t, double(cmd_roll)*100, '--', 'LineWidth',1.2);
grid on; ylabel('roll (int16)');
legend('controller.ctr\_roll','cmd\_roll');

% --- pitch ---
nexttile;
plot(t, controller_ctr_pitch, 'LineWidth',1.2); hold on;
plot(t, double(cmd_pitch)*100, '--', 'LineWidth',1.2);
grid on; ylabel('pitch (int16)');
legend('controller.ctr\_pitch','cmd\_pitch');

% --- yaw ---
nexttile;
plot(t, controller_ctr_yaw, 'LineWidth',1.2); hold on;
plot(t, double(cmd_yaw)*100, '--', 'LineWidth',1.2);
grid on; xlabel('Time [s]'); ylabel('yaw (int16)');
legend('controller.ctr\_yaw','cmd\_yaw');

%%
%%
figure('Name','Mocap Roll Pitch Yaw','NumberTitle','off');

subplot(3,1,1);
plot(Abs_time, mocap_roll_deg, 'LineWidth', 1.5);
grid on;
ylabel('Roll (deg)');
title('Mocap Euler Angles');

subplot(3,1,2);
plot(Abs_time, mocap_pitch_deg, 'LineWidth', 1.5);
grid on;
ylabel('Pitch (deg)');

subplot(3,1,3);

yyaxis left;
plot(Abs_time, mocap_yaw_deg, 'LineWidth', 1.5);
ylabel('Yaw (deg)');

yyaxis right;
plot(Abs_time, mocap_yawrate_deg, 'LineWidth', 1.2);
hold on;
plot(Abs_time, mocap_yawrate_deg_filt, 'LineWidth', 2);
ylabel('Yaw rate (deg/s)');

grid on;
xlabel('Time (s)');
title('Mocap Yaw and Logged Yaw Rate');
legend('Yaw', 'Yaw rate raw', 'Yaw rate filtered');

%%
%%
% small smoothing window
filter_window = 8;   % 越小滤波越轻，建议 3~10

%R13_filt = movmean(R13, filter_window);
%R23_filt = movmean(R23, filter_window);

figure('Name','R13 R23 Tracking','NumberTitle','off');

subplot(2,1,1);
plot(Abs_time, R13*100, 'LineWidth', 1.0);
hold on;
plot(Abs_time, R13_filt*1000, 'LineWidth', 1.8);
plot(Abs_time, mocap_x_filt*1000, '--', 'LineWidth', 1.5);

plot(Abs_time, cmd_roll, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('R13 / desired_x');
title('R13 Tracking');
legend('R13 raw', 'R13 filtered', 'mocap\_x','cmd_x');

subplot(2,1,2);
plot(Abs_time, R23, 'LineWidth', 1.0);
hold on;
plot(Abs_time, R23_filt*1000, 'LineWidth', 1.8);
plot(Abs_time, mocap_y_filt*1000, '--', 'LineWidth', 1.5);
plot(Abs_time, cmd_pitch, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('R23 / desired_y');
title('R23 Tracking');
legend('R23 raw', 'R23 filtered', 'mocap\_y','cmd_y');

%%
figure('Name','Command Roll Pitch Yaw Thrust','NumberTitle','off');

subplot(4,1,1);
plot(Abs_time, cmd_roll, 'LineWidth', 1.5);
grid on;
ylabel('cmd roll');
title('Command Signals');

subplot(4,1,2);
plot(Abs_time, cmd_pitch, 'LineWidth', 1.5);
grid on;
ylabel('cmd pitch');

subplot(4,1,3);
plot(Abs_time, cmd_yaw, 'LineWidth', 1.5);
grid on;
ylabel('cmd yaw');

subplot(4,1,4);
plot(Abs_time, cmd_thrust, 'LineWidth', 1.5);
grid on;
ylabel('cmd thrust');
xlabel('Time (s)');

%% 

figure('Name','R13 R23 Tracking','NumberTitle','off');

subplot(3,1,1);
plot(Abs_time, mocap_x_raw, 'LineWidth', 1.0);
hold on;
plot(Abs_time, mocap_x_filt, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R13 / desired_x');
title('R13 Tracking');
legend('R13 raw', 'R13 filtered');

subplot(3,1,2);
plot(Abs_time, mocap_y_raw, 'LineWidth', 1.0);
hold on;
plot(Abs_time, mocap_y_filt, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('R23 / desired_y');
title('R23 Tracking');
legend('R23 raw', 'R23 filtered');

subplot(3,1,3);
plot(Abs_time, mocap_z_raw, 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('R23 / desired_y');
title('R23 Tracking');
legend('R23 raw', 'R23 filtered');

%%
%%
% ==========================================
% Raw / Filt signals and 1st / 2nd derivatives
% Cut data from first mocap_yawrate_deg_filt > 4000 to +2 seconds
%
% Signals:
%   mocap_x_raw / mocap_x_filt
%   mocap_y_raw / mocap_y_filt
%   R13 / R13_filt
%   R23 / R23_filt
% ==========================================

t_all = Abs_time(:);

% ---------- Find segment ----------
yawrate_threshold = -2800;
duration_after_threshold = 3;   % seconds

yawrate_signal = mocap_yawrate_deg_filt(:);

idx_start = find(yawrate_signal < yawrate_threshold, 1, 'first');

if isempty(idx_start)
    error('No data found where mocap_yawrate_deg_filt < %.1f', yawrate_threshold);
end

t_start = t_all(idx_start);
t_end = t_start + duration_after_threshold;

idx_seg = (t_all >= t_start) & (t_all <= t_end);

% Cut time and reset to start from 0
t = t_all(idx_seg) - t_start;

fprintf('Selected segment: %.3f s to %.3f s, duration %.3f s\n', ...
    t_start, t_end, t(end));

% ---------- Cut signals ----------
x_raw = mocap_x_raw(idx_seg);
x_filt = mocap_x_filt(idx_seg);

mocap_vx_filt = mocap_vx_filt(idx_seg);
mocap_vy_filt = mocap_vy_filt(idx_seg);

R13_d_filt = R13_d_filt(idx_seg);
R23_d_filt = R23_d_filt(idx_seg);

y_raw = mocap_y_raw(idx_seg);
y_filt = mocap_y_filt(idx_seg);

r13_raw = R13(idx_seg);
r13_filt = R13_filt(idx_seg);

r23_raw = R23(idx_seg);
r23_filt = R23_filt(idx_seg);

yawrate_filt_cut = mocap_yawrate_deg_filt(idx_seg);

% Make sure all data are column vectors
x_raw = x_raw(:);
x_filt = x_filt(:);

y_raw = y_raw(:);
y_filt = y_filt(:);

r13_raw = r13_raw(:);
r13_filt = r13_filt(:);

r23_raw = r23_raw(:);
r23_filt = r23_filt(:);

yawrate_filt_cut = yawrate_filt_cut(:);

dd_filter_window = 21; 

% ---------- First derivatives ----------
x_raw_d = gradient(x_raw, t);
x_filt_d = gradient(x_filt, t);

y_raw_d = gradient(y_raw, t);
y_filt_d = gradient(y_filt, t);

x_filt_d = movmean(x_filt_d, dd_filter_window);
y_filt_d = movmean(y_filt_d, dd_filter_window);

r13_raw_d = gradient(r13_raw, t);
r13_filt_d = gradient(r13_filt, t);

r23_raw_d = gradient(r23_raw, t);
r23_filt_d = gradient(r23_filt, t);

% ---------- Second derivatives ----------
x_raw_dd = gradient(x_raw_d, t);
x_filt_dd = gradient(x_filt_d, t);


y_raw_dd = gradient(y_raw_d, t);
y_filt_dd = gradient(y_filt_d, t);



% ---------- Extra filter for second derivatives ----------
dd_filter_window = 11;   % 越大越平滑，延迟/失真越大

x_filt_dd_filt = movmean(x_filt_dd, dd_filter_window);
y_filt_dd_filt = movmean(y_filt_dd, dd_filter_window);

r13_raw_dd = gradient(r13_raw_d, t);

r23_raw_dd = gradient(r23_raw_d, t);

r13_filt_d_filt = movmean(R13_d_filt, dd_filter_window);
r23_filt_d_filt = movmean(R23_d_filt, dd_filter_window);

r13_filt_dd = gradient(r13_filt_d_filt, t);
r23_filt_dd = gradient(r23_filt_d_filt, t);

r13_filt_dd_filt = movmean(r13_filt_dd, dd_filter_window);
r23_filt_dd_filt = movmean(r23_filt_dd, dd_filter_window);


% ---------- Plot selected yawrate ----------
figure('Name','Selected Segment Yawrate','NumberTitle','off');

plot(t, yawrate_filt_cut, 'LineWidth', 1.5);
grid on;
xlabel('Time after yawrate threshold (s)');
ylabel('mocap yawrate filtered (deg/s)');
title(sprintf('Selected Segment: %.2f s to %.2f s', t_start, t_end));

% ---------- Plot X ----------
figure('Name','Mocap X raw/filt and derivatives','NumberTitle','off');

subplot(3,1,1);
plot(t, x_raw, 'LineWidth', 1.0);
hold on;
plot(t, x_filt, 'LineWidth', 1.5);
grid on;
ylabel('X (m)');
title('Mocap X');
legend('x raw', 'x filt');

subplot(3,1,2);
plot(t, x_raw_d, 'LineWidth', 1.0);
hold on;
%plot(t, x_filt_d, 'LineWidth', 1.5);
plot(t, mocap_vx_filt, 'LineWidth', 1.5);
grid on;
ylabel('dX/dt (m/s)');
legend('x raw dot', 'x filt dot');

subplot(3,1,3);
plot(t, r13_filt*10, 'LineWidth', 1.0);
hold on;
plot(t, x_filt_dd, 'LineWidth', 1.5);
plot(t, x_filt_dd_filt, 'LineWidth', 1.8);
grid on;
ylabel('d2X/dt2 (m/s^2)');
xlabel('Time after yawrate threshold (s)');
legend('x raw ddot', 'x filt ddot');

% ---------- Plot Y ----------
figure('Name','Mocap Y raw/filt and derivatives','NumberTitle','off');

subplot(3,1,1);
plot(t, y_raw, 'LineWidth', 1.0);
hold on;
plot(t, y_filt, 'LineWidth', 1.5);
grid on;
ylabel('Y (m)');
title('Mocap Y');
legend('y raw', 'y filt');

subplot(3,1,2);
plot(t, y_raw_d, 'LineWidth', 1.0);
hold on;
%plot(t, y_filt_d, 'LineWidth', 1.5);
plot(t, mocap_vy_filt, 'LineWidth', 1.5);
grid on;
ylabel('dY/dt (m/s)');
legend('y raw dot', 'y filt dot');

subplot(3,1,3);
plot(t, r23_filt*10, 'LineWidth', 1.0);
hold on;
plot(t, y_filt_dd, 'LineWidth', 1.5);
plot(t, y_filt_dd_filt, 'LineWidth', 1.8);
grid on;
ylabel('d2Y/dt2 (m/s^2)');
xlabel('Time after yawrate threshold (s)');
legend('y raw ddot', 'y filt ddot');

% ---------- Plot R13 ----------
figure('Name','R13 raw/filt and derivatives','NumberTitle','off');

subplot(3,1,1);
plot(t, r13_raw, 'LineWidth', 1.0);
hold on;
plot(t, r13_filt, 'LineWidth', 1.5);
grid on;
ylabel('R13');
title('R13');
legend('R13 raw', 'R13 filt');

subplot(3,1,2);
plot(t, r13_filt_d_filt, 'LineWidth', 1.0);
hold on;
%plot(t, r13_filt_d, 'LineWidth', 1.5);
plot(t, R13_d_filt, 'LineWidth', 1.5);
grid on;
ylabel('dR13/dt');
legend('r13 raw dot', 'R13 filt dot');

subplot(3,1,3);
plot(t, r13_filt_dd_filt, 'LineWidth', 1.0);
hold on;
plot(t, r13_filt_dd, 'LineWidth', 1.5);
grid on;
ylabel('d2R13/dt2');
xlabel('Time after yawrate threshold (s)');
legend('R13 filt ddot filt', 'R13 filt ddot');

% ---------- Plot R23 ----------
figure('Name','R23 raw/filt and derivatives','NumberTitle','off');

subplot(3,1,1);
plot(t, r23_raw, 'LineWidth', 1.0);
hold on;
plot(t, r23_filt, 'LineWidth', 1.5);
grid on;
ylabel('R23');
title('R23');
legend('R23 raw', 'R23 filt');

subplot(3,1,2);
plot(t, r23_filt_d_filt, 'LineWidth', 1.0);
hold on;
%plot(t, r23_filt_d, 'LineWidth', 1.5);
plot(t, R23_d_filt, 'LineWidth', 1.5);
grid on;
ylabel('dR23/dt');
legend('R23 filt dot filt', 'R23 filt dot');

subplot(3,1,3);
plot(t, r23_filt_dd_filt, 'LineWidth', 1.0);
hold on;
plot(t, r23_filt_dd, 'LineWidth', 1.5);
grid on;
ylabel('d2R23/dt2');
xlabel('Time after yawrate threshold (s)');
legend('R23 filt ddot filt', 'R23 filt ddot');