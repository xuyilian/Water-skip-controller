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

%% ---------- Onboard vs mocap vertical acceleration ----------
% Checks whether the vehicle's ~8000 deg/s spin contaminates the onboard
% accelerometer's Z axis. Physics: pure centripetal acceleration from
% rotation about Z only loads the accelerometer's X/Y axes, not Z -- in the
% ideal case Z should read true vertical dynamics regardless of spin rate.
% Real hardware (prop/hydrofoil imbalance, any wobble/precession) can leak
% some of that into Z as an oscillation at the spin frequency. mocap_z is
% physically incapable of being affected by spin the same way (it's an
% external optical measurement), so it's the reference to compare against.
if ~exist('t','var'); analyse_prep; end
if exist('acc_z','var')
    g = 9.81;
    % Double-differentiate filtered mocap Z to get a KINEMATIC vertical
    % acceleration (m/s^2), then convert to the accelerometer's own
    % convention -- SPECIFIC FORCE, reading +1 g at rest, not 0 -- so the
    % two traces sit on the same axis and are directly comparable.
    accel_smooth_window = max(3, round(0.1 * Fs));
    vz_from_filt = movmean(gradient(mocap_z_filt, t), accel_smooth_window);
    az_mocap_ms2 = movmean(gradient(vz_from_filt, t), accel_smooth_window);
    az_mocap_g   = az_mocap_ms2 / g + 1.0;

    figure('Name','Onboard vs mocap vertical acceleration','NumberTitle','off');
    plot(t, acc_z, 'LineWidth', 1.2); hold on;
    plot(t, az_mocap_g, 'LineWidth', 1.6);
    yline(1.0, ':', '1 g (at rest)', 'HandleVisibility','off');
    grid on; xlabel('Time [s]'); ylabel('Vertical acceleration [g]');
    title('Onboard accelerometer (acc.z) vs mocap-derived vertical acceleration');
    legend('acc.z (onboard, raw)', 'z acceleration (from mocap, double-differentiated)', ...
        'Location','best');
else
    fprintf('Onboard-vs-mocap acceleration plot skipped: this log has no acc_z field.\n');
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

% Distinct colors (not line style) for the right-axis traces: at high
% oscillation frequency, dashed/dotted styles visually crush together and
% become unreadable, but solid lines in different colors stay distinguishable.
color_yaw          = [0.00 0.45 0.74];  % blue
color_yawrate_raw  = [0.65 0.65 0.65];  % gray  -- noisiest, recedes visually
color_yawrate_filt = [0.85 0.10 0.10];  % red
color_yawrate_smooth = [0.00 0.60 0.20]; % green

yyaxis left;
plot(t, mocap_yaw_deg, '-', 'Color', color_yaw, 'LineWidth', 1.4);
ylabel('Yaw [deg]');

yyaxis right;
plot(t, mocap_yawrate_deg, '-', 'Color', color_yawrate_raw, 'LineWidth', 1.0); hold on;
plot(t, mocap_yawrate_deg_filt,   '-', 'Color', color_yawrate_filt,   'LineWidth', 1.6);
plot(t, mocap_yawrate_deg_smooth, '-', 'Color', color_yawrate_smooth, 'LineWidth', 1.8);
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
%  Segment analysis around water-skip hop events (multiple hops per log)
%  Hop = detected directly from the mocap PHYSICS (a real fall then a
%  rebound in vertical velocity), using the same fall/rise thresholds as
%  the live DROP auto-hop-detect in revolvingdario.py. Deliberately NOT
%  keyed off DROP mode's command signature: in practice hops get tested
%  both via DROP mode AND by simply disabling the controller (Circle) and
%  letting the vehicle fall, and only the physics-based detection catches
%  both. The whole log is scanned, so every hop in a multi-hop log gets its
%  own set of plots, each tagged "(hop k/N)".
%
%  IMPORTANT: mocap tracking commonly drops out right at splashdown (markers
%  submerged / occluded by splash) -- e.g. mocap_z_raw can snap to a stale
%  value while mocap_vz_filt spikes to an unphysical speed. Every plot below
%  shades that dropout window (gray patch) instead of silently plotting it
%  as real data. Battery voltage is the one signal here that is NOT
%  mocap-derived, so it stays trustworthy straight through the dropout --
%  it's the best available cross-check for "was it actually in the water"
%  when mocap itself is worthless.
%  ==========================================================
if ~exist('t','var'); analyse_prep; end
t_all = Abs_time(:);

% ---- Hop detection knobs (same physics/values as DROP_HOP_* in
% revolvingdario.py's live auto-hop-detect) ----
hop_fall_vz          = -0.3;   % m/s -- must fall at least this fast to arm
hop_rise_vz          = 0.15;   % m/s -- rising above this (once armed) is a hop
hop_max_plausible_vz = 3.0;    % m/s -- above this, treat vz as a mocap dropout glitch
hop_max_plausible_z  = 1.5;    % m -- above this, treat z as a mocap dropout sentinel
lead_in              = 1.0;    % seconds shown before each hop onset
max_duration         = 4.0;    % seconds shown after each hop onset (or end of log)
vz_dropout_threshold = 5.0;    % m/s -- |vz| above this flags a mocap-dropout sample

% ---- Scan the whole log for hop onsets: arm on a real fall, trigger on
% the rebound, then only re-arm after falling again (so one long fall can't
% register as multiple hops). ----
vz_all = mocap_vz_filt(:);
z_all  = mocap_z_raw(:);
armed  = false;
hop_onsets = [];
for k = 1:numel(t_all)
    if vz_all(k) < hop_fall_vz
        armed = true;
    end
    if armed && vz_all(k) > hop_rise_vz && vz_all(k) < hop_max_plausible_vz ...
            && z_all(k) < hop_max_plausible_z
        hop_onsets(end+1) = k; %#ok<AGROW>
        armed = false;
    end
end

if isempty(hop_onsets)
    warning('Segment analysis skipped: no water-skip hop detected in this log.');
    return
end

fprintf('Segment analysis: found %d hop(s) in this log, at t = %s s\n', ...
    numel(hop_onsets), mat2str(round(t_all(hop_onsets).', 2)));

for h = 1:numel(hop_onsets)
    idx_hop = hop_onsets(h);

    seg_t_start = max(t_all(1),   t_all(idx_hop) - lead_in);
    seg_t_end   = min(t_all(end), t_all(idx_hop) + max_duration);
    idx_seg     = (t_all >= seg_t_start) & (t_all <= seg_t_end);

    if nnz(idx_seg) < 5
        warning('Hop %d/%d skipped: only %d sample(s) in the segment window.', ...
            h, numel(hop_onsets), nnz(idx_seg));
        continue
    end

    % Segment time, reset to start at 0 at the hop onset.
    ts = t_all(idx_seg) - t_all(idx_hop);
    fprintf('  Hop %d/%d: onset at t = %.3f s, window %.3f s to %.3f s\n', ...
        h, numel(hop_onsets), t_all(idx_hop), ts(1), ts(end));

    % Cut signals (force column vectors, matching ts).
    z_raw       = mocap_z_raw(idx_seg).';
    z_filt      = mocap_z_filt(idx_seg).';
    vz_seg      = mocap_vz_filt(idx_seg).';
    x_raw       = mocap_x_raw(idx_seg).';
    y_raw       = mocap_y_raw(idx_seg).';
    r13_seg     = R13_filt(idx_seg).';
    r23_seg     = R23_filt(idx_seg).';
    yawrate_seg = mocap_yawrate_deg_filt(idx_seg).';
    vbat_seg    = pm_vbat(idx_seg).';

    % Dropout mask: samples where mocap is not to be trusted. Two distinct
    % signatures, since a dropout doesn't always look the same:
    %   1) huge spurious vertical speed (sentinel-jump dropout)
    %   2) z_raw exactly repeating for several samples in a row (frozen
    %      dropout -- e.g. RealTimeProcessor keeps last-good values on a
    %      missing packet). Real optical tracking always has sub-mm jitter,
    %      so an exact repeat run is itself a stale-data signature. This one
    %      is NOT caught by the vz check -- a frozen value settles vz toward
    %      0, not a spike. Matches DROP_HOP_FROZEN_SAMPLES in revolvingdario.py.
    frozen_run_length = 15;  % consecutive exact-repeat samples (~150 ms @ 100 Hz)
    is_repeat = [false; diff(z_raw(:)) == 0];
    frozen = movsum(is_repeat, [frozen_run_length - 1, 0]) >= frozen_run_length;

    dropout = (abs(vz_seg) > vz_dropout_threshold) | frozen;
    if any(dropout)
        fprintf('  Hop %d/%d: %d/%d samples flagged as mocap dropout (|vz| > %.1f m/s, or frozen z)\n', ...
            h, numel(hop_onsets), nnz(dropout), numel(dropout), vz_dropout_threshold);
    end

    plot_hop_segment(h, numel(hop_onsets), ts, z_raw, z_filt, vz_seg, ...
        x_raw, y_raw, r13_seg, r23_seg, yawrate_seg, vbat_seg, dropout);
end


% ======================================================================
%  Local helper functions (valid at the end of a MATLAB script, R2016b+)
% ======================================================================
function plot_hop_segment(hop_num, n_hops, ts, z_raw, z_filt, vz_seg, ...
        x_raw, y_raw, r13_seg, r23_seg, yawrate_seg, vbat_seg, dropout)
% Produces the standard 5-figure water-skip segment view for ONE detected
% hop, tagged "(hop hop_num/n_hops)" so multiple hops in one log don't get
% confused with each other. t = 0 is the hop onset in every plot.
tag = sprintf(' (hop %d/%d)', hop_num, n_hops);

figure('Name', ['Segment: altitude' tag], 'NumberTitle','off');
plot(ts, z_raw, 'LineWidth', 1.0); hold on;
plot(ts, z_filt, 'LineWidth', 1.6);
xline(0, '--', 'hop', 'LineWidth', 1.2);
grid on; xlabel('Time from hop onset [s]'); ylabel('Altitude [m]');
title(['Segment: altitude through the water-skip event' tag]);
legend('z (raw)','z (filtered)', 'Location','best');
shade_dropout(gca, ts, dropout);

figure('Name', ['Segment: vertical velocity' tag], 'NumberTitle','off');
plot(ts, vz_seg, 'LineWidth', 1.6); hold on;
yline(0, ':');
xline(0, '--', 'hop', 'LineWidth', 1.2);
grid on; xlabel('Time from hop onset [s]'); ylabel('v_z (filtered) [m/s]');
title(['Segment: vertical velocity -- impact speed vs. rebound speed' tag]);
shade_dropout(gca, ts, dropout);

figure('Name', ['Segment: battery voltage' tag], 'NumberTitle','off');
plot(ts, vbat_seg, 'LineWidth', 1.6); hold on;
xline(0, '--', 'hop', 'LineWidth', 1.2);
grid on; xlabel('Time from hop onset [s]'); ylabel('V_{bat} [V]');
title(['Segment: battery voltage (onboard telemetry -- unaffected by mocap dropout)' tag]);
shade_dropout(gca, ts, dropout);

figure('Name', ['Segment: yaw rate' tag], 'NumberTitle','off');
plot(ts, yawrate_seg, 'LineWidth', 1.6); hold on;
xline(0, '--', 'hop', 'LineWidth', 1.2);
grid on; xlabel('Time from hop onset [s]'); ylabel('Yaw rate (filtered) [deg/s]');
title(['Segment: spin rate through submersion' tag]);
shade_dropout(gca, ts, dropout);

figure('Name', ['Segment: horizontal drift and tilt' tag], 'NumberTitle','off');
tiledlayout(2,1);

nexttile;
plot(ts, x_raw, 'LineWidth', 1.4); hold on;
plot(ts, y_raw, 'LineWidth', 1.4);
xline(0, '--', 'hop', 'LineWidth', 1.2);
grid on; ylabel('Position [m]');
legend('x (raw)','y (raw)', 'Location','best');
title(['Segment: horizontal drift' tag]);
shade_dropout(gca, ts, dropout);

nexttile;
plot(ts, r13_seg, 'LineWidth', 1.4); hold on;
plot(ts, r23_seg, 'LineWidth', 1.4);
xline(0, '--', 'hop', 'LineWidth', 1.2);
grid on; ylabel('Tilt'); xlabel('Time from hop onset [s]');
legend('R13 (filtered)','R23 (filtered)', 'Location','best');
title(['Segment: attitude tilt' tag]);
shade_dropout(gca, ts, dropout);
end

function shade_dropout(ax, ts, dropout)
% Shades contiguous TRUE runs of `dropout` (aligned with ts) as translucent
% gray patches behind the plotted data, flagging stretches where mocap
% tracking was lost (e.g. markers submerged/occluded at splashdown) so they
% are never mistaken for real position/velocity data. No-op if nothing is
% flagged. Patches are excluded from the legend (HandleVisibility off).
if ~any(dropout)
    return
end
yl = ylim(ax);
starts = find(diff([false; dropout(:); false]) == 1);
ends   = find(diff([false; dropout(:); false]) == -1) - 1;
for k = 1:numel(starts)
    xs = ts([starts(k), ends(k)]);
    p = patch(ax, [xs(1) xs(2) xs(2) xs(1)], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.5 0.5 0.5], 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
    uistack(p, 'bottom');
end
ylim(ax, yl);
end
