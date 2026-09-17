% yaw_offset_sim.m
% ------------------------------------------------------------------
% Simulate the revolving yaw pipeline to see how the world-frame force
% direction behaves as the body spins, for different yaw_offset_deg.
%
% Pipeline (matches revolvinglian.py + firmware snippet):
%   1. Body spins in world frame:  mocap_yaw(t) = wrap(w*t)
%   2. Python:   yaw_send = wrap(mocap_yaw + offset)
%                then int16 quantize (yawdeg*100 rounded) as the radio does
%   3. Firmware: y = yaw_send [rad]
%                f_pitch =  U_X*cos(y) + U_Y*sin(y)
%                f_roll  = -U_X*sin(y) + U_Y*cos(y)     -> body_cmd = R(-y)*U
%   4. Physical body at world yaw = mocap_yaw turns body force to world:
%                world_force = R(mocap_yaw)*body_cmd
% ------------------------------------------------------------------
clear; clc; close all;

% ---- config ----
yawrate_dps = -5000.0;          % cf2 fixed yawrate (deg/s)
U  = [1.0, 0.0];                % commanded world force: +X (angle 0 deg)
desired_dir = atan2d(U(2), U(1));
t  = linspace(0, 0.1, 2000);    % 0.1 s ~ 1.4 rev at 5000 dps

offsets = [0, 180, 210];
cols = [0.13 0.47 0.71;         % blue
        0.17 0.63 0.17;         % green
        0.84 0.15 0.16];        % red

figure('Position', [100 100 900 780], 'Color', 'w');

% --- panel 1: mocap yaw (same for all offsets) ---
subplot(3,1,1); hold on; grid on;
mocap_yaw = wrap180(yawrate_dps * t);
plot(t*1000, mocap_yaw, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
ylabel('mocap\_yaw [deg]');
title(sprintf(['Body spinning at %.0f deg/s   |   ' ...
    'commanded world force dir = %.0f deg (+X)'], yawrate_dps, desired_dir));

% --- panels 2 and 3 ---
subplot(3,1,2); hold on; grid on;
subplot(3,1,3); hold on; grid on;

leg2 = {}; leg3 = {};
for k = 1:numel(offsets)
    off = offsets(k);
    [~, ~, fx, fy, direction] = simulate(off, yawrate_dps, U, t);

    subplot(3,1,2);
    plot(t*1000, direction, 'Color', cols(k,:), 'LineWidth', 1.8);
    leg2{end+1} = sprintf('offset=%d  (mean dir=%.1f deg)', off, mean(direction)); %#ok<SAGROW>

    subplot(3,1,3);
    plot(t*1000, fx, 'Color', cols(k,:), 'LineWidth', 1.5);
    plot(t*1000, fy, '--', 'Color', cols(k,:), 'LineWidth', 1.0);
    leg3{end+1} = sprintf('Fx off=%d', off); %#ok<SAGROW>
    leg3{end+1} = sprintf('Fy off=%d', off); %#ok<SAGROW>
end

subplot(3,1,2);
yline(desired_dir, 'k:', 'LineWidth', 1.0);
ylabel('world FORCE direction [deg]');
legend(leg2, 'Location', 'best', 'FontSize', 8);

subplot(3,1,3);
ylabel('world force  Fx / Fy');
xlabel('time [ms]');
legend(leg3, 'Location', 'best', 'FontSize', 7, 'NumColumns', 3);

% ---- numeric summary ----
fprintf('\n offset |  mean dir  |   std    | expected\n');
fprintf('--------+------------+----------+---------\n');
for k = 1:numel(offsets)
    off = offsets(k);
    [~, ~, ~, ~, direction] = simulate(off, yawrate_dps, U, t);
    expected = wrap180(desired_dir - off);
    fprintf(' %6.1f | %10.3f | %8.3f | %8.3f\n', ...
        off, mean(direction), std(direction), expected);
end

saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'yaw_offset_sim.png'));


% ================== helper functions ==================
function v = wrap180(v_deg)
    v = mod(v_deg + 180, 360) - 180;
end

function q = quantize_i16(yawdeg, scale)
    if nargin < 2, scale = 100.0; end
    q = round(yawdeg * scale);
    q = min(max(q, -32768), 32767);
    q = q / scale;
end

function [mocap_yaw, yaw_send, fx, fy, direction] = simulate(offset_deg, yawrate_dps, U, t)
    mocap_yaw = wrap180(yawrate_dps * t);            % step 1
    yaw_send  = wrap180(mocap_yaw + offset_deg);     % step 2
    yaw_send  = quantize_i16(yaw_send);              % int16 packing

    y  = deg2rad(yaw_send);
    Ux = U(1); Uy = U(2);
    f_pitch =  Ux*cos(y) + Uy*sin(y);                % step 3: R(-y)*U
    f_roll  = -Ux*sin(y) + Uy*cos(y);

    th = deg2rad(mocap_yaw);                          % step 4: R(theta)*body
    fx = cos(th).*f_pitch - sin(th).*f_roll;
    fy = sin(th).*f_pitch + cos(th).*f_roll;

    direction = atan2d(fy, fx);
end
