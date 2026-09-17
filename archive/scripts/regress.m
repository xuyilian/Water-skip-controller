%% ==========================================
% Multi-MAT regression using same data processing as plotting script
%
% For each MAT file:
%   Cut every non-overlapping segment from each downward threshold crossing
%   of mocap_yawrate_deg_filt to +duration.
%
% Model:
%       Xdot = A X + B U
%
% State:
%       X = [x, y, vx, vy, r13, r23, r13_dot, r23_dot]^T
%
% Input:
%       U = [tau_x, tau_y]^T
%         = [cmd_pitch, cmd_roll]^T
%
% For attitude rows:
%       r13_ddot uses only [vx, vy, r13_dot, r23_dot, tau_y]
%       r23_ddot uses only [vx, vy, r13_dot, r23_dot, tau_x]
% ==========================================

clear; clc;

%% ---------- Select MAT files ----------
[file_names, folder_path] = uigetfile('*.mat', ...
    'Select MAT files for regression', ...
    'MultiSelect', 'on');

if isequal(file_names, 0)
    error('No MAT files selected.');
end

if ischar(file_names)
    file_names = {file_names};
end

num_files = numel(file_names);

fprintf('Selected %d MAT files.\n', num_files);

%% ---------- User settings ----------
yawrate_threshold = -2000;
duration_after_threshold = 2.0;

vel_filter_window = 11;
dd_filter_window  = 11;

%% ---------- Storage for all files ----------
X_all = [];
U_all = [];
Xdot_all = [];

file_info = struct([]);

diag_sample_all = [];
diag_r13_d_raw_all = [];
diag_r13_d_filt_all = [];
diag_r23_d_raw_all = [];
diag_r23_d_filt_all = [];
diag_r13_raw_all = [];
diag_r13_filt_all = [];
diag_r23_raw_all = [];
diag_r23_filt_all = [];
diag_ax_all = [];
diag_ay_all = [];
diag_sample_offset = 0;

%% ==========================================
% Process each MAT file
% ==========================================
for k = 1:num_files
    file_path = fullfile(folder_path, file_names{k});
    fprintf('\nLoading file %d/%d: %s\n', k, num_files, file_names{k});

    S = load(file_path);

    %% ---------- Check required variables ----------
    required_vars = {
        'Abs_time', ...
        'mocap_x_raw', 'mocap_x_filt', ...
        'mocap_y_raw', 'mocap_y_filt', ...
        'R13', 'R13_filt', ...
        'R23', 'R23_filt', ...
        'R13_d_filt', 'R23_d_filt', ...
        'mocap_vx_filt', 'mocap_vy_filt', ...
        'mocap_yawrate_deg_filt', ...
        'cmd_pitch', 'cmd_roll'
    };

    missing_vars = {};
    for ii = 1:numel(required_vars)
        if ~isfield(S, required_vars{ii})
            missing_vars{end+1} = required_vars{ii}; %#ok<SAGROW>
        end
    end

    if ~isempty(missing_vars)
        warning('Skipping %s because missing variables: %s', ...
            file_names{k}, strjoin(missing_vars, ', '));
        continue;
    end

    %% ---------- Full time ----------
    t_all = S.Abs_time(:);

    %% ---------- Find all non-overlapping segments ----------
    yawrate_signal = S.mocap_yawrate_deg_filt(:);

    below_threshold = yawrate_signal < yawrate_threshold;
    idx_cross = find(below_threshold & [true; ~below_threshold(1:end-1)]);

    if isempty(idx_cross)
        warning('Skipping %s: no data where yawrate < %.1f', ...
            file_names{k}, yawrate_threshold);
        continue;
    end

    idx_start_list = [];
    last_segment_end_time = -inf;

    for jj = 1:numel(idx_cross)
        candidate_idx = idx_cross(jj);
        candidate_t = t_all(candidate_idx);

        if candidate_t <= last_segment_end_time
            continue;
        end

        idx_start_list(end+1) = candidate_idx; %#ok<SAGROW>
        last_segment_end_time = candidate_t + duration_after_threshold;
    end

    fprintf('Found %d threshold crossings, using %d non-overlapping segments.\n', ...
        numel(idx_cross), numel(idx_start_list));

    file_segment_count = 0;

    for seg_i = 1:numel(idx_start_list)
        idx_start = idx_start_list(seg_i);
        t_start = t_all(idx_start);
        t_end = t_start + duration_after_threshold;

        idx_seg = (t_all >= t_start) & (t_all <= t_end);

        if nnz(idx_seg) < 20
            warning('Skipping %s segment %d: selected segment too short.', ...
                file_names{k}, seg_i);
            continue;
        end

        t = t_all(idx_seg) - t_start;

        fprintf('Selected segment %d/%d: %.3f s to %.3f s, duration %.3f s, N=%d\n', ...
            seg_i, numel(idx_start_list), t_start, t_end, t(end), numel(t));

        %% ---------- Cut signals ----------
        x_raw  = S.mocap_x_raw(idx_seg);
        x_filt = S.mocap_x_filt(idx_seg);

        y_raw  = S.mocap_y_raw(idx_seg);
        y_filt = S.mocap_y_filt(idx_seg);

        r13_raw  = S.R13(idx_seg);
        r13_filt = S.R13_filt(idx_seg);

        r23_raw  = S.R23(idx_seg);
        r23_filt = S.R23_filt(idx_seg);

        vx_filt_cut = S.mocap_vx_filt(idx_seg);
        vy_filt_cut = S.mocap_vy_filt(idx_seg);

        r13_dot_cut = S.R13_d_filt(idx_seg);
        r23_dot_cut = S.R23_d_filt(idx_seg);
        if isfield(S, 'R13_d') && isfield(S, 'R23_d')
            r13_dot_raw_cut = S.R13_d(idx_seg);
            r23_dot_raw_cut = S.R23_d(idx_seg);
        else
            r13_dot_raw_cut = [];
            r23_dot_raw_cut = [];
        end

        tau_x = S.cmd_pitch(idx_seg);
        tau_y = S.cmd_roll(idx_seg);

        %% ---------- Column vectors ----------
        x_raw  = x_raw(:);
        x_filt = x_filt(:);

        y_raw  = y_raw(:);
        y_filt = y_filt(:);

        r13_raw  = r13_raw(:);
        r13_filt = r13_filt(:);

        r23_raw  = r23_raw(:);
        r23_filt = r23_filt(:);

        vx_filt_cut = vx_filt_cut(:);
        vy_filt_cut = vy_filt_cut(:);

        r13_dot_cut = r13_dot_cut(:);
        r23_dot_cut = r23_dot_cut(:);
        if isempty(r13_dot_raw_cut)
            r13_dot_raw = gradient(r13_filt, t);
            r23_dot_raw = gradient(r23_filt, t);
        else
            r13_dot_raw = r13_dot_raw_cut(:);
            r23_dot_raw = r23_dot_raw_cut(:);
        end

        tau_x = tau_x(:);
        tau_y = tau_y(:);

        %% ---------- Position derivatives for plotting / consistency ----------
        x_raw_d  = gradient(x_raw, t);
        x_filt_d = gradient(x_filt, t);

        y_raw_d  = gradient(y_raw, t);
        y_filt_d = gradient(y_filt, t);

        %% ---------- Use logged filtered mocap velocities for regression ----------
        vx = movmean(vx_filt_cut, vel_filter_window);
        vy = movmean(vy_filt_cut, vel_filter_window);

        %% ---------- Acceleration from logged filtered velocities ----------
        ax = gradient(vx, t);
        ay = gradient(vy, t);

        ax = movmean(ax, dd_filter_window);
        ay = movmean(ay, dd_filter_window);

        %% ---------- R13/R23 derivatives ----------
        r13_raw_d = gradient(r13_raw, t);
        r23_raw_d = gradient(r23_raw, t);

        r13_dot = movmean(r13_dot_cut, dd_filter_window);
        r23_dot = movmean(r23_dot_cut, dd_filter_window);

        r13_ddot = gradient(r13_dot, t);
        r23_ddot = gradient(r23_dot, t);

        r13_ddot = movmean(r13_ddot, dd_filter_window);
        r23_ddot = movmean(r23_ddot, dd_filter_window);

        %% ---------- Build regression data for this segment ----------
        x   = x_filt;
        y   = y_filt;
        r13 = r13_filt;
        r23 = r23_filt;

        X = [
            x, ...
            y, ...
            vx, ...
            vy, ...
            r13, ...
            r23, ...
            r13_dot, ...
            r23_dot ...
        ];

        U = [
            tau_x, ...
            tau_y ...
        ];

        Xdot = [
            vx, ...
            vy, ...
            ax, ...
            ay, ...
            r13_dot, ...
            r23_dot, ...
            r13_ddot, ...
            r23_ddot ...
        ];

        %% ---------- Remove invalid samples ----------
        data_all = [X, U, Xdot];

        valid_idx = all(isfinite(data_all), 2);

        diag_segment = [
            r13_dot_raw, ...
            r13_dot_cut, ...
            r23_dot_raw, ...
            r23_dot_cut, ...
            r13_raw, ...
            r13_filt, ...
            r23_raw, ...
            r23_filt, ...
            ax, ...
            ay ...
        ];

        X = X(valid_idx, :);
        U = U(valid_idx, :);
        Xdot = Xdot(valid_idx, :);
        diag_segment = diag_segment(valid_idx, :);

        %% ---------- Trim edge samples ----------
        trimN = max(5, ceil(max(vel_filter_window, dd_filter_window)/2));

        if size(X,1) > 2*trimN
            X = X(trimN+1:end-trimN, :);
            U = U(trimN+1:end-trimN, :);
            Xdot = Xdot(trimN+1:end-trimN, :);
            diag_segment = diag_segment(trimN+1:end-trimN, :);
        else
            warning('Skipping %s segment %d after trim: too few samples.', ...
                file_names{k}, seg_i);
            continue;
        end

        %% ---------- Store diagnostic signals for plotting ----------
        diag_sample = diag_sample_offset + (1:size(diag_segment, 1)).';
        diag_sample_all = [diag_sample_all; diag_sample; NaN]; %#ok<AGROW>
        diag_r13_d_raw_all = [diag_r13_d_raw_all; diag_segment(:,1); NaN]; %#ok<AGROW>
        diag_r13_d_filt_all = [diag_r13_d_filt_all; diag_segment(:,2); NaN]; %#ok<AGROW>
        diag_r23_d_raw_all = [diag_r23_d_raw_all; diag_segment(:,3); NaN]; %#ok<AGROW>
        diag_r23_d_filt_all = [diag_r23_d_filt_all; diag_segment(:,4); NaN]; %#ok<AGROW>
        diag_r13_raw_all = [diag_r13_raw_all; diag_segment(:,5); NaN]; %#ok<AGROW>
        diag_r13_filt_all = [diag_r13_filt_all; diag_segment(:,6); NaN]; %#ok<AGROW>
        diag_r23_raw_all = [diag_r23_raw_all; diag_segment(:,7); NaN]; %#ok<AGROW>
        diag_r23_filt_all = [diag_r23_filt_all; diag_segment(:,8); NaN]; %#ok<AGROW>
        diag_ax_all = [diag_ax_all; diag_segment(:,9); NaN]; %#ok<AGROW>
        diag_ay_all = [diag_ay_all; diag_segment(:,10); NaN]; %#ok<AGROW>
        diag_sample_offset = diag_sample_offset + size(diag_segment, 1);

        %% ---------- Append to all files ----------
        X_all = [X_all; X]; %#ok<AGROW>
        U_all = [U_all; U]; %#ok<AGROW>
        Xdot_all = [Xdot_all; Xdot]; %#ok<AGROW>

        file_segment_count = file_segment_count + 1;
        file_info(end+1).file_name = file_names{k}; %#ok<SAGROW>
        file_info(end).segment_index = seg_i;
        file_info(end).idx_start = idx_start;
        file_info(end).t_start = t_start;
        file_info(end).t_end = t_end;
        file_info(end).num_samples = size(X,1);
    end

    if file_segment_count == 0
        warning('Skipping %s: no valid segments after processing.', file_names{k});
    end
end

%% ---------- Check total data ----------
if isempty(X_all)
    error('No valid data collected from selected MAT files.');
end

fprintf('\nTotal samples used for regression: %d\n', size(X_all,1));

%% ==========================================
% Regression on all files together
% ==========================================

A_hat = zeros(8,8);
B_hat = zeros(8,2);

%% ---------- Rows 1 to 6: normal full regression ----------
Phi_full = [X_all, U_all];

M_T_1to6 = Phi_full \ Xdot_all(:,1:6);
M_1to6   = M_T_1to6.';

A_hat(1:6,:) = M_1to6(:,1:8);
B_hat(1:6,:) = M_1to6(:,9:10);

%% ==========================================
% Physically constrained attitude regression
%
% Fixed structure:
%
%   r13_ddot = -kv*vy - d*r13_dot + gc*r23_dot + b*tau_y
%   r23_ddot =  kv*vx - gc*r13_dot - d*r23_dot + b*tau_x
%
% Unknown parameters:
%
%   theta = [kv, d, gc, b]^T
%
% This enforces:
%
%   A(7,4) = -A(8,3)
%   A(7,7) =  A(8,8)
%   A(7,8) = -A(8,7)
%   B(7,2) =  B(8,1)
%   B(7,1) = 0
%   B(8,2) = 0
% ==========================================

vx      = X_all(:,3);
vy      = X_all(:,4);
r13_dot = X_all(:,7);
r23_dot = X_all(:,8);

tau_x = U_all(:,1);   % cmd_pitch
tau_y = U_all(:,2);   % cmd_roll

r13_ddot_meas = Xdot_all(:,7);
r23_ddot_meas = Xdot_all(:,8);

% Stack two equations into one least-squares problem:
%
% r13_ddot = -kv*vy - d*r13_dot + gc*r23_dot + b*tau_y
% r23_ddot =  kv*vx - gc*r13_dot - d*r23_dot + b*tau_x
%
% theta = [kv, d, gc, b]^T

Y_att = [
    r13_ddot_meas;
    r23_ddot_meas
];

Phi_att = [
    -vy,       -r13_dot,    r23_dot,    tau_y;   % r13_ddot
     vx,       -r23_dot,   -r13_dot,    tau_x    % r23_ddot
];

theta_att = Phi_att \ Y_att;

kv = theta_att(1);
d  = theta_att(2);
gc = theta_att(3);
b  = theta_att(4);

% For compatibility with old variable names
by = b;
bx = b;

% Fill constrained A/B rows
A_hat(7,:) = 0;
A_hat(8,:) = 0;
B_hat(7,:) = 0;
B_hat(8,:) = 0;

% r13_ddot = -kv*vy - d*r13_dot + gc*r23_dot + b*tau_y
A_hat(7,4) = -kv;
A_hat(7,7) = -d;
A_hat(7,8) = gc;
B_hat(7,2) = b;

% r23_ddot = kv*vx - gc*r13_dot - d*r23_dot + b*tau_x
A_hat(8,3) = kv;
A_hat(8,7) = -gc;
A_hat(8,8) = -d;
B_hat(8,1) = b;
%% ---------- Prediction ----------
Xdot_pred_all = X_all * A_hat.' + U_all * B_hat.';

err_all = Xdot_all - Xdot_pred_all;
rmse_all = sqrt(mean(err_all.^2, 1));

%% ---------- Display ----------
disp('A matrix from multi-MAT regression:');
disp(A_hat);

disp('B matrix from multi-MAT regression:');
disp(B_hat);

disp('RMSE of each Xdot component:');
disp(rmse_all);

disp('State order:');
disp('[x, y, vx, vy, r13, r23, r13_dot, r23_dot]');

disp('Input order:');
disp('[tau_x, tau_y] = [cmd_pitch, cmd_roll]');

fprintf('\nPhysically constrained attitude model from all MAT files:\n');

fprintf('kv = %.6f\n', kv);
fprintf('d  = %.6f\n', d);
fprintf('gc = %.6f\n', gc);
fprintf('by = %.6f\n', by);
fprintf('bx = %.6f\n', bx);

fprintf('\nr13_ddot = %.6f*vy + %.6f*r13_dot + %.6f*r23_dot + %.6f*tau_y\n', ...
    A_hat(7,4), A_hat(7,7), A_hat(7,8), B_hat(7,2));

fprintf('r23_ddot = %.6f*vx + %.6f*r13_dot + %.6f*r23_dot + %.6f*tau_x\n', ...
    A_hat(8,3), A_hat(8,7), A_hat(8,8), B_hat(8,1));

%% ---------- Plot regression result ----------
state_dot_names = {
    'x dot', ...
    'y dot', ...
    'vx dot / ax', ...
    'vy dot / ay', ...
    'r13 dot', ...
    'r23 dot', ...
    'r13 ddot', ...
    'r23 ddot' ...
};

figure('Name','Multi-MAT regression: Xdot vs predicted Xdot','NumberTitle','off');

for i = 1:8
    subplot(4,2,i);
    plot(Xdot_all(:,i), 'LineWidth', 1.0);
    hold on;
    plot(Xdot_pred_all(:,i), '--', 'LineWidth', 1.0);
    grid on;
    ylabel(state_dot_names{i});
    legend('Measured', 'Predicted');
end

xlabel('Stacked sample index');

%% ---------- Plot prediction error ----------
figure('Name','Multi-MAT regression: prediction error','NumberTitle','off');

for i = 1:8
    subplot(4,2,i);
    plot(err_all(:,i), 'LineWidth', 1.0);
    grid on;
    ylabel(['err ', state_dot_names{i}]);
end

xlabel('Stacked sample index');

%% ---------- Plot inputs ----------
figure('Name','Multi-MAT regression: inputs','NumberTitle','off');

subplot(2,1,1);
plot(U_all(:,1), 'LineWidth', 1.0);
grid on;
ylabel('\tau_x');
title('\tau_x = cmd\_pitch');

subplot(2,1,2);
plot(U_all(:,2), 'LineWidth', 1.0);
grid on;
ylabel('\tau_y');
xlabel('Stacked sample index');
title('\tau_y = cmd\_roll');

%% ---------- Plot diagnostic signals ----------
if ~isempty(diag_sample_all)
    figure('Name','Multi-MAT regression: diagnostics','NumberTitle','off');

    subplot(5,1,1);
    plot(diag_sample_all, diag_r13_d_raw_all, 'LineWidth', 1.0);
    hold on;
    plot(diag_sample_all, diag_r13_d_filt_all, '--', 'LineWidth', 1.0);
    grid on;
    ylabel('R13 dot');
    legend('R13\_d raw', 'R13\_d filt');

    subplot(5,1,2);
    plot(diag_sample_all, diag_r23_d_raw_all, 'LineWidth', 1.0);
    hold on;
    plot(diag_sample_all, diag_r23_d_filt_all, '--', 'LineWidth', 1.0);
    grid on;
    ylabel('R23 dot');
    legend('R23\_d raw', 'R23\_d filt');

    subplot(5,1,3);
    plot(diag_sample_all, diag_r13_raw_all, 'LineWidth', 1.0);
    hold on;
    plot(diag_sample_all, diag_r13_filt_all, '--', 'LineWidth', 1.0);
    grid on;
    ylabel('R13');
    legend('R13 raw', 'R13 filt');

    subplot(5,1,4);
    plot(diag_sample_all, diag_r23_raw_all, 'LineWidth', 1.0);
    hold on;
    plot(diag_sample_all, diag_r23_filt_all, '--', 'LineWidth', 1.0);
    grid on;
    ylabel('R23');
    legend('R23 raw', 'R23 filt');

    subplot(5,1,5);
    plot(diag_sample_all, diag_ax_all, 'LineWidth', 1.0);
    hold on;
    plot(diag_sample_all, diag_ay_all, '--', 'LineWidth', 1.0);
    grid on;
    ylabel('horizontal acc');
    xlabel('Stacked segment sample index');
    legend('a_x', 'a_y');
end

%% ---------- Save result ----------
identified_model_multi_mat_corresponding_tau.A = A_hat;
identified_model_multi_mat_corresponding_tau.B = B_hat;
identified_model_multi_mat_corresponding_tau.rmse_all = rmse_all;

identified_model_multi_mat_corresponding_tau.file_info = file_info;
identified_model_multi_mat_corresponding_tau.num_segments_used = numel(file_info);
identified_model_multi_mat_corresponding_tau.yawrate_threshold = yawrate_threshold;
identified_model_multi_mat_corresponding_tau.duration_after_threshold = duration_after_threshold;
identified_model_multi_mat_corresponding_tau.vel_filter_window = vel_filter_window;
identified_model_multi_mat_corresponding_tau.dd_filter_window = dd_filter_window;

identified_model_multi_mat_corresponding_tau.constraint = ...
    'r13_ddot fitted with vx, vy, r13_dot, r23_dot, tau_y only; r23_ddot fitted with vx, vy, r13_dot, r23_dot, tau_x only';

identified_model_multi_mat_corresponding_tau.state_name = {
    'x', ...
    'y', ...
    'vx', ...
    'vy', ...
    'r13', ...
    'r23', ...
    'r13_dot', ...
    'r23_dot'
};

identified_model_multi_mat_corresponding_tau.input_name = {
    'tau_x_cmd_pitch', ...
    'tau_y_cmd_roll'
};

save('identified_AB_model_multi_mat_corresponding_tau.mat', ...
    'identified_model_multi_mat_corresponding_tau');

disp('Saved to identified_AB_model_multi_mat_corresponding_tau.mat');
