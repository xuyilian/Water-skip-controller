%% Tune yawrate filter and yaw jump rejection from the latest flight log
% Offline model matched to revolvinglian.py:
%   1) raw mocap yaw -> yawrate raw by wrapped finite difference
%   2) yawrate raw -> yawrate clean by median spike rejection
%   3) yawrate clean -> yawrate filt by first-order low-pass
%   4) sent yaw is kept as an internally continuous angle, then wrapped to
%      [-180, 180] only at the final output. The yawrate model still uses
%      raw mocap yaw, matching revolvinglian.py. Prediction is used only
%      for non-boundary yaw jumps.

clear; clc;

data_dir = fullfile(pwd, 'DataExchange');
files = dir(fullfile(data_dir, '*.mat'));
if isempty(files)
    error('No .mat files found in %s', data_dir);
end

[~, newest_idx] = max([files.datenum]);
mat_path = fullfile(files(newest_idx).folder, files(newest_idx).name);
S = load(mat_path);

required_vars = {'Abs_time', 'mocap_yaw_deg'};
for i = 1:numel(required_vars)
    if ~isfield(S, required_vars{i})
        error('Latest log %s does not contain %s', mat_path, required_vars{i});
    end
end

t = double(S.Abs_time(:));
yaw_raw = wrap_deg_180(double(S.mocap_yaw_deg(:)));
N = numel(t);

dt_vec = [0; diff(t)];
dt_nominal = median(dt_vec(dt_vec > 1e-6));
dt_vec(dt_vec <= 1e-6) = dt_nominal;

% Match fixed Python constants unless explicitly tuned below.
yawrate_start_deg = 2000.0;
yawrate_spike_window = 11;
yawrate_abs_limit = 9000.0;
yaw_raw_jump_reject_deg = 90.0;
yaw_raw_jump_rate_margin_deg = 45.0;
yaw_reacquire_frames = 3;
yaw_wrap_boundary_deg = 150.0;
preferred_max_yawrate_alpha = 0.20;

% Grid search candidates. Include high alpha values so the yawrate model can
% follow fast rotation without accumulating phase drift.
yawrate_alpha_candidates = [0.06:0.02:0.30, 0.35:0.05:1.00];
yawrate_jump_limit_candidates = 800:200:6000;
yaw_jump_reject_candidates = 4:2:90;

% A physical "jump" is evaluated as disagreement between the output step and
% its yawrate-filter prediction. Wrapped ±180 display jumps are not counted.
allowed_prediction_step_error_deg = 30.0;

n_alpha = numel(yawrate_alpha_candidates);
n_jump_limit = numel(yawrate_jump_limit_candidates);
n_reject = numel(yaw_jump_reject_candidates);
n_total = n_alpha * n_jump_limit * n_reject;

fit_rms = zeros(n_total, 1);
jump_count = zeros(n_total, 1);
jump_max = zeros(n_total, 1);
nonboundary_output_jump_count = zeros(n_total, 1);
reject_count = zeros(n_total, 1);
yawrate_roughness = zeros(n_total, 1);
unwrapped_rms = zeros(n_total, 1);
score = zeros(n_total, 1);

alpha_hist = zeros(n_total, 1);
jump_limit_hist = zeros(n_total, 1);
reject_deg_hist = zeros(n_total, 1);

idx = 0;
yaw_raw_unwrapped_ref = rad2deg(unwrap(deg2rad(yaw_raw)));
for ia = 1:n_alpha
    yawrate_alpha = yawrate_alpha_candidates(ia);
    for ij = 1:n_jump_limit
        yawrate_jump_limit = yawrate_jump_limit_candidates(ij);

        for ir = 1:n_reject
            yaw_jump_reject_deg = yaw_jump_reject_candidates(ir);
            idx = idx + 1;

            [yaw_out, yaw_out_unwrapped, reject_flag, ...
                    yawrate_raw, yawrate_clean, yawrate_filt] = ...
                simulate_yaw_output( ...
                    yaw_raw, dt_vec, ...
                    yawrate_alpha, yawrate_spike_window, ...
                    yawrate_jump_limit, yawrate_abs_limit, ...
                    yawrate_start_deg, yaw_jump_reject_deg, ...
                    yaw_raw_jump_reject_deg, yaw_raw_jump_rate_margin_deg, ...
                    yaw_reacquire_frames, yaw_wrap_boundary_deg);

            fit_error = wrap_deg_180(yaw_out - yaw_raw);
            fit_rms(idx) = rms(fit_error);

            unwrapped_rms(idx) = rms(yaw_out_unwrapped - yaw_raw_unwrapped_ref);

            predicted_step = yawrate_filt(2:end) .* dt_vec(2:end);
            actual_step = diff(yaw_out_unwrapped);
            step_error = wrap_deg_180(actual_step - predicted_step);
            high_speed = abs(yawrate_filt(2:end)) >= yawrate_start_deg;
            jump_flag = high_speed & abs(step_error) > allowed_prediction_step_error_deg;
            output_display_step = diff(yaw_out);
            output_display_jump = abs(output_display_step) > 180.0;
            output_boundary_crossing = crosses_wrap_boundary(yaw_out_unwrapped);
            nonboundary_output_jump_count(idx) = ...
                sum(output_display_jump & ~output_boundary_crossing);

            jump_count(idx) = sum(jump_flag);
            if any(high_speed)
                jump_max(idx) = max(abs(step_error(high_speed)));
                yawrate_step = diff(yawrate_filt);
                yawrate_rough = rms(yawrate_step(high_speed));
            else
                jump_max(idx) = 0.0;
                yawrate_rough = rms(diff(yawrate_filt));
            end
            reject_count(idx) = sum(reject_flag);
            yawrate_roughness(idx) = yawrate_rough;

            alpha_hist(idx) = yawrate_alpha;
            jump_limit_hist(idx) = yawrate_jump_limit;
            reject_deg_hist(idx) = yaw_jump_reject_deg;

            % Priority:
            %   no physical output jumps,
            %   no non-boundary display jumps after wrapping,
            %   close to raw mocap yaw,
            %   no long-term unwrapped phase drift,
            %   fewer rejected samples,
            %   smoother yawrate model.
            score(idx) = fit_rms(idx) ...
                + 0.03 * unwrapped_rms(idx) ...
                + 2000.0 * jump_count(idx) ...
                + 3000.0 * nonboundary_output_jump_count(idx) ...
                + 30.0 * max(0.0, jump_max(idx) - allowed_prediction_step_error_deg) ...
                + 0.005 * reject_count(idx) ...
                + 0.0005 * yawrate_roughness(idx);
        end
    end
end

smooth_idx = find( ...
    nonboundary_output_jump_count == 0 ...
    & alpha_hist <= preferred_max_yawrate_alpha);
no_jump_idx = find(jump_count == 0 & nonboundary_output_jump_count == 0);
if ~isempty(smooth_idx)
    smooth_score = fit_rms(smooth_idx) ...
        + 0.03 * unwrapped_rms(smooth_idx) ...
        + 100.0 * jump_count(smooth_idx) ...
        + 0.005 * reject_count(smooth_idx) ...
        + 0.001 * yawrate_roughness(smooth_idx);
    [~, local_best] = min(smooth_score);
    best_idx = smooth_idx(local_best);
elseif ~isempty(no_jump_idx)
    sub_score = fit_rms(no_jump_idx) ...
        + 0.03 * unwrapped_rms(no_jump_idx) ...
        + 0.005 * reject_count(no_jump_idx) ...
        + 0.0005 * yawrate_roughness(no_jump_idx);
    [~, local_best] = min(sub_score);
    best_idx = no_jump_idx(local_best);
else
    [~, best_idx] = min(score);
end

best_yawrate_alpha = alpha_hist(best_idx);
best_yawrate_jump_limit = jump_limit_hist(best_idx);
best_yaw_jump_reject_deg = reject_deg_hist(best_idx);

[yaw_best, yaw_best_unwrapped, reject_best, ...
        yawrate_raw_best, yawrate_clean_best, yawrate_filt_best] = ...
    simulate_yaw_output( ...
        yaw_raw, dt_vec, ...
        best_yawrate_alpha, yawrate_spike_window, ...
        best_yawrate_jump_limit, yawrate_abs_limit, ...
        yawrate_start_deg, best_yaw_jump_reject_deg, ...
        yaw_raw_jump_reject_deg, yaw_raw_jump_rate_margin_deg, ...
        yaw_reacquire_frames, yaw_wrap_boundary_deg);

yaw_raw_unwrapped = rad2deg(unwrap(deg2rad(yaw_raw)));
unwrapped_error = yaw_best_unwrapped - yaw_raw_unwrapped;

fprintf('\nLatest log: %s\n', mat_path);
fprintf('Recommended R_LMS_YAW_RATE_ALPHA     = %.3f\n', best_yawrate_alpha);
fprintf('Recommended R_LMS_YAWRATE_JUMP_LIMIT = %.1f deg/s\n', best_yawrate_jump_limit);
fprintf('Recommended YAW_JUMP_REJECT_DEG      = %.1f deg\n', best_yaw_jump_reject_deg);
fprintf('Fit RMS circular error               = %.3f deg\n', fit_rms(best_idx));
fprintf('Unwrapped RMS error                  = %.3f deg\n', rms(unwrapped_error));
fprintf('High-speed jump count                = %d\n', jump_count(best_idx));
fprintf('Non-boundary wrapped output jumps    = %d\n', ...
    nonboundary_output_jump_count(best_idx));
fprintf('Max high-speed step error            = %.3f deg\n', jump_max(best_idx));
fprintf('Rejected samples                     = %d / %d\n', reject_count(best_idx), N);

fig_dir = fullfile(pwd, 'matlab_figures_yaw_tuning');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

fig = figure('Name', 'Yaw Jump Reject Tuning', 'Color', 'w', ...
    'Visible', 'off', 'Position', [60 60 1350 940]);
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, yaw_raw, 'LineWidth', 1.0); hold on;
[t_plot, yaw_plot] = make_wrapped_plot_series(t, yaw_best_unwrapped);
plot(t_plot, yaw_plot, '--', 'LineWidth', 1.2);
if any(reject_best)
    scatter(t(reject_best), yaw_best(reject_best), 18, 'filled');
end
yline(180, ':', 'LineWidth', 0.8);
yline(-180, ':', 'LineWidth', 0.8);
grid on;
ylabel('wrapped yaw (deg)');
legend('mocap yaw raw', 'simulated sent yaw', 'rejected mocap frame', ...
    'Location', 'best');
title(sprintf(['Best: yawrate alpha %.2f, yawrate jump %.0f deg/s, ', ...
    'yaw reject %.1f deg'], ...
    best_yawrate_alpha, best_yawrate_jump_limit, best_yaw_jump_reject_deg));

nexttile;
plot(t, yaw_raw_unwrapped, 'LineWidth', 1.0); hold on;
plot(t, yaw_best_unwrapped, '--', 'LineWidth', 1.2);
grid on;
ylabel('unwrapped yaw (deg)');
legend('mocap yaw unwrapped', 'sent yaw unwrapped', 'Location', 'best');

nexttile;
plot(t, yawrate_raw_best, ':', 'LineWidth', 0.9); hold on;
plot(t, yawrate_clean_best, '--', 'LineWidth', 1.0);
plot(t, yawrate_filt_best, 'LineWidth', 1.3);
yline(yawrate_start_deg, ':', 'LineWidth', 1.0);
yline(-yawrate_start_deg, ':', 'LineWidth', 1.0);
grid on;
ylabel('yawrate (deg/s)');
legend('raw', 'clean', 'filtered', 'start threshold', 'Location', 'best');

nexttile;
scatter3(alpha_hist, jump_limit_hist, reject_deg_hist, 22, fit_rms, 'filled');
hold on;
scatter3(best_yawrate_alpha, best_yawrate_jump_limit, best_yaw_jump_reject_deg, ...
    90, 'r', 'filled');
grid on;
xlabel('yawrate alpha');
ylabel('yawrate jump limit (deg/s)');
zlabel('yaw reject (deg)');
title('Parameter scan colored by circular fit RMS');
cb = colorbar;
cb.Label.String = 'fit RMS (deg)';
view(35, 24);

png_path = fullfile(fig_dir, 'yaw_joint_tuning_latest.png');
fig_path = fullfile(fig_dir, 'yaw_joint_tuning_latest.fig');
exportgraphics(fig, png_path, 'Resolution', 200);
savefig(fig, fig_path);

result_path = fullfile(fig_dir, 'yaw_joint_tuning_latest.mat');
save(result_path, ...
    'mat_path', ...
    'best_yawrate_alpha', 'best_yawrate_jump_limit', ...
    'best_yaw_jump_reject_deg', ...
    'yawrate_alpha_candidates', 'yawrate_jump_limit_candidates', ...
    'yaw_jump_reject_candidates', ...
    'fit_rms', 'jump_count', 'jump_max', ...
    'nonboundary_output_jump_count', 'reject_count', ...
    'yawrate_roughness', 'unwrapped_rms', 'score', ...
    'alpha_hist', 'jump_limit_hist', 'reject_deg_hist', ...
    'yaw_raw', 'yaw_best', 'yaw_raw_unwrapped', 'yaw_best_unwrapped', ...
    'yawrate_raw_best', 'yawrate_clean_best', 'yawrate_filt_best', ...
    'reject_best', 't', 'yawrate_start_deg', ...
    'yaw_raw_jump_reject_deg', 'yaw_raw_jump_rate_margin_deg', ...
    'yaw_reacquire_frames', 'yaw_wrap_boundary_deg', ...
    'preferred_max_yawrate_alpha', ...
    'allowed_prediction_step_error_deg');

fprintf('Saved figure: %s\n', png_path);
fprintf('Saved result: %s\n\n', result_path);

function [yaw_out, yaw_unwrapped_out, reject_flag, ...
        yawrate_raw, yawrate_clean, yawrate_filt] = ...
        simulate_yaw_output( ...
        yaw_raw, dt_vec, yawrate_alpha, spike_window, jump_limit, abs_limit, ...
        yawrate_start_deg, jump_reject_deg, raw_jump_reject_deg, ...
        raw_jump_rate_margin_deg, reacquire_frames, wrap_boundary_deg)

    N = numel(yaw_raw);
    yaw_out = zeros(N, 1);
    yaw_unwrapped_out = zeros(N, 1);
    reject_flag = false(N, 1);
    yawrate_raw = zeros(N, 1);
    yawrate_clean = zeros(N, 1);
    yawrate_filt = zeros(N, 1);

    yaw_prev_wrapped = wrap_deg_180(yaw_raw(1));
    yaw_prev_unwrapped = yaw_prev_wrapped;
    yaw_raw_prev_wrapped = yaw_prev_wrapped;
    reject_count_run = 0;
    yaw_out(1) = yaw_prev_wrapped;
    yaw_unwrapped_out(1) = yaw_prev_unwrapped;

    for k = 2:N
        % Use the previous filtered yawrate to predict this sample. The
        % yawrate model itself is updated from raw mocap yaw, matching the
        % online controller.
        yaw_pred_unwrapped = yaw_prev_unwrapped + yawrate_filt(k-1) * dt_vec(k);
        yaw_raw_wrapped = wrap_deg_180(yaw_raw(k));
        yaw_raw_step = yaw_raw_wrapped - yaw_prev_wrapped;
        raw_jump_limit_deg = max( ...
            raw_jump_reject_deg, ...
            abs(yawrate_filt(k-1)) * dt_vec(k) + raw_jump_rate_margin_deg);
        is_large_raw_jump = abs(yaw_raw_step) > raw_jump_limit_deg;
        is_wrap_boundary_jump = is_large_raw_jump ...
            && yaw_raw_wrapped * yaw_prev_wrapped < 0.0 ...
            && abs(yaw_raw_wrapped) >= wrap_boundary_deg ...
            && abs(yaw_prev_wrapped) >= wrap_boundary_deg;
        is_nonboundary_raw_jump = is_large_raw_jump && ~is_wrap_boundary_jump;
        yaw_raw_candidate_unwrapped = yaw_prev_unwrapped ...
            + wrap_deg_180(yaw_raw_wrapped - yaw_prev_wrapped);

        if is_nonboundary_raw_jump
            reject_count_run = reject_count_run + 1;
            if reject_count_run >= reacquire_frames
                yaw_unwrapped_out(k) = yaw_raw_candidate_unwrapped;
                reject_count_run = 0;
            else
                yaw_unwrapped_out(k) = yaw_pred_unwrapped;
                reject_flag(k) = true;
            end
        else
            yaw_unwrapped_out(k) = yaw_raw_candidate_unwrapped;
            reject_count_run = 0;
        end

        yaw_out(k) = wrap_deg_180(yaw_unwrapped_out(k));
        yaw_delta = wrap_deg_180(yaw_raw_wrapped - yaw_raw_prev_wrapped);
        yaw_raw_prev_wrapped = yaw_raw_wrapped;
        yaw_prev_unwrapped = yaw_unwrapped_out(k);
        yaw_prev_wrapped = yaw_out(k);

        yawrate_raw(k) = yaw_delta / dt_vec(k);
        i0 = max(1, k - spike_window);
        recent_median = median(yawrate_clean(i0:k-1));
        is_abs_spike = abs(yawrate_raw(k)) > abs_limit;
        is_jump_spike = abs(yawrate_raw(k) - recent_median) > jump_limit;

        if is_abs_spike || is_jump_spike
            yawrate_clean(k) = recent_median;
        else
            yawrate_clean(k) = yawrate_raw(k);
        end

        yawrate_filt(k) = yawrate_filt(k-1) ...
            + yawrate_alpha * (yawrate_clean(k) - yawrate_filt(k-1));
    end
end

function crossed = crosses_wrap_boundary(yaw_unwrapped)
    wrap_index = floor((yaw_unwrapped + 180.0) / 360.0);
    crossed = diff(wrap_index) ~= 0;
end

function [t_plot, yaw_plot] = make_wrapped_plot_series(t, yaw_unwrapped)
    N = numel(t);
    t_plot = t(1);
    yaw_plot = wrap_deg_180(yaw_unwrapped(1));

    for k = 1:N-1
        y0 = yaw_unwrapped(k);
        y1 = yaw_unwrapped(k+1);
        w1 = wrap_deg_180(y1);
        display_step = w1 - yaw_plot(end);

        if display_step < -180.0
            boundary = (floor((y0 + 180.0) / 360.0) + 1.0) * 360.0 - 180.0;
            tc = t(k) + (boundary - y0) / (y1 - y0) * (t(k+1) - t(k));
            t_plot = [t_plot; tc; NaN; tc; t(k+1)]; %#ok<AGROW>
            yaw_plot = [yaw_plot; 180.0; NaN; -180.0; w1]; %#ok<AGROW>
        elseif display_step > 180.0
            boundary = floor((y0 + 180.0) / 360.0) * 360.0 - 180.0;
            tc = t(k) + (boundary - y0) / (y1 - y0) * (t(k+1) - t(k));
            t_plot = [t_plot; tc; NaN; tc; t(k+1)]; %#ok<AGROW>
            yaw_plot = [yaw_plot; -180.0; NaN; 180.0; w1]; %#ok<AGROW>
        else
            t_plot = [t_plot; t(k+1)]; %#ok<AGROW>
            yaw_plot = [yaw_plot; w1]; %#ok<AGROW>
        end
    end
end

function y = wrap_deg_180(x)
    y = mod(x + 180.0, 360.0) - 180.0;
end
