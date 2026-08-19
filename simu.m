clear; clc; close all;

% =========================================================================
%  Water-skipping hop study
%  Predicts the spin (omega) and vertical velocity (zdot) of the vehicle
%  through one water contact (entry -> submerged -> exit), and sweeps the
%  three engineering variables we can change:
%       1) rtip    (hydrofoil tip radius, measured from the SPIN AXIS. The
%                   foil does not reach the axis: it runs from the motor
%                   mount at rinner out to rtip, so its span is rtip-rinner)
%       2) beta    (hydrofoil angle of attack / inclination)
%       3) zdot0   (vertical speed into the water at contact)
%  The submerged chord grows with penetration depth, c_sub = z/sin(beta),
%  but is capped at the physical chord cf once the plate is fully wetted
%  (depth > cf*sin(beta)).  All other parameters are held constant.
% =========================================================================

RUN_FIXED_RTIP_SCATTER_ONLY = true;   % true: only generate the fixed-rtip beta-zdot figure

%% ===== Fixed parameters (held constant) =====
% AS-BUILT VEHICLE (updated 2026-08-17). Every value here describes the
% vehicle that actually flew the hops reported in Ch.5.
N    = 4;            % number of hydrofoils
cf   = 15e-3;        % hydrofoil physical chord [m]; caps the wetted length
m    = 90e-3;        % vehicle mass [kg], all-up incl. mocap markers
rho  = 1000;         % water density [kg/m^3]
g    = 9.81;         % gravitational acceleration [m/s^2]

% ---- HYDROFOIL RADIAL GEOMETRY (the two radius knobs) -------------------
% The foil does NOT reach the axis of rotation. Each leg drops from its motor
% mount at rinner and kinks outward into the foil, which runs from there to
% rtip. Both radii are measured from the SPIN AXIS, and the blade-element
% integral is taken over rinner -> rtip (see compute_thrust_and_torque):
%
%        axis .......... rinner ============ rtip
%        (no foil here)         ^ wetted span ^
%
% As built: rinner = 75 mm (150 mm core diameter), rtip = 145 mm, so the
% foil span is 70 mm. Vary either independently; rtip must exceed rinner.
rinner = 75e-3;      % foil ROOT radius [m] -- where the foil starts
% rtip is set per case; its nominal value is nom.rtip below.

% Moment of inertia about the spin axis: uniform 2-D disk of the vehicle mass
%   I = m * R_disk^2.
% R_disk is taken as the full tip radius, 145 mm: the legs and foils reach
% that far and carry real mass, so the vehicle's inertia is set by its whole
% span rather than by the motor-mount circle alone.
% TODO: still a PLACEHOLDER -- the MoI has never been measured. omega_exit
% depends on it directly, so measure it before quoting a predicted spin loss.
R_disk = 145e-3;     % disk radius [m] = foil tip radius
I      = m * R_disk^2;

% Spin: entry value and the controller minimum (both given in deg/s).
% Read from the logged mocap yaw rate. NOTE the signal oscillates strongly
% at the spin frequency -- in hover the raw rate swings ~7900-12000 deg/s
% and the 9-point filtered rate ~7500-9200 -- so these are read off the
% traces, not sampled at a point. (Rates are genuine, not aliased: the raw
% yaw steps ~88 deg/sample at 95 Hz against a 17,000 deg/s limit.)
%
%     hover, lift rotors ON    ~8800 deg/s
%     freefall, rotors OFF     ~8000 deg/s   <- omega0
%     after the water contact  ~3200 deg/s   <- ~40% retained; a validation
%                                               target for the omega_exit
%                                               this model predicts
%
% The ~800 deg/s drop between hover and freefall is consistent in both the
% raw and filtered traces and across both hops of the stable-2-hop case. It
% is the lift rotors' reaction-torque contribution to yaw disappearing when
% they are commanded off, leaving the tangential rotors driving the spin
% alone. Use the freefall value: the foils never meet the water in hover.
omega0    = deg2rad(8000);   % spin at water contact [rad/s] (8000 deg/s)
omega_min = deg2rad(1000);   % controller minimum spin [rad/s] (1000 deg/s)

%% ===== Integration settings =====
z0    = 0;           % water surface position [m]
dt    = 1e-5;        % time step [s]
t_end = 0.3;         % max simulation time [s]
t     = 0:dt:t_end;

%% ===== Nominal values of the three engineering variables =====
% The as-built operating point: foil tip 145 mm from the spin axis
% (= rinner 75 mm + 70 mm span), beta 30 deg (the only angle fabricated and
% flown), entry speed the measured -1.55 to -1.87 m/s.
nom.rinner = rinner;          % m (75 mm) -- foil root, sweepable like the rest
nom.rtip  = 0.145;            % m (145 mm from the spin axis)
nom.beta  = deg2rad(30);      % rad (30 deg, as built)
nom.zdot0 = -1.7;             % m/s (downward), mid-range of the measured entries

%% ===== Sweep ranges for each variable =====
% rtip is swept about the as-built 145 mm by varying the foil SPAN from
% 20 to 120 mm beyond the fixed 75 mm root; beta spans the design study,
% with 30 deg the value actually built.
rtip_list   = rinner + [0.020 0.045 0.070 0.095 0.120];   % m -> 95..195 mm
beta_list   = deg2rad([5 10 15 20 25 30]);                % rad
zdot0_list  = [-1.0 -1.5 -1.7 -2.0 -3.0];                 % m/s
% Root radius swept with the TIP held at its nominal 145 mm, so this varies
% the span from the inside: a larger rinner means a shorter foil sitting
% further out, where it sweeps faster.
rinner_list = [0.025 0.050 0.075 0.100 0.125];            % m, must stay < rtip

%% ===== Pack constants into a struct for the helpers =====
params = struct('N',N,'cf',cf,'m',m,'I',I,'rho',rho,'g',g,'rinner',rinner, ...
                'omega0',omega0,'omega_min',omega_min,'z0',z0,'t',t,'dt',dt);

%% ===== Fast path: fixed-rtip beta-zdot plane colored by exit speed =====
if RUN_FIXED_RTIP_SCATTER_ONLY
    plot_fixed_rtip_exit_speed_scatter([], params);
    return;
end

%% ===== Report nominal-case result =====
[~,~,~,we_nom,ze_nom,te_nom] = run_case(nom.rtip,nom.beta,nom.zdot0,params,nom.rinner);
fprintf('\nNominal case (r_tip=%.0f mm, beta=%.0f deg, zdot0=%.1f m/s):\n', ...
    nom.rtip*1e3, rad2deg(nom.beta), nom.zdot0);
fprintf('  omega_exit = %.0f deg/s (min %.0f),  zdot_exit = %.3f m/s,  t_contact = %.2f ms\n', ...
    rad2deg(we_nom), rad2deg(omega_min), ze_nom, te_nom*1e3);

%% ===== Specific entry-to-exit case requested for reporting =====
% rtip is measured from the SPIN AXIS and must exceed rinner, so this is the
% as-built 145 mm, not the old 40 mm (which predated the annular geometry
% and now fails the rtip > rinner check).
plot_entry_exit_case(nom.rtip, nom.beta, nom.zdot0, params);

%% ===== Three one-at-a-time sweeps (time-series of omega and zdot) =====
sweep_and_plot('rtip',   rtip_list,   rtip_list*1e3,     'r_{tip} = %.0f mm',  nom, params);
sweep_and_plot('rinner', rinner_list, rinner_list*1e3,   'r_{in} = %.0f mm',   nom, params);
sweep_and_plot('beta',   beta_list,   rad2deg(beta_list),'\\beta = %.0f deg',  nom, params);
sweep_and_plot('zdot0',  zdot0_list,  zdot0_list,        'z'' = %.1f m/s',     nom, params);

%% ===== Optimization: grids for the 2-D heatmaps and the 3-D search =====
% rtip is swept from rinner+20 mm to rinner+120 mm, i.e. 95-195 mm from the
% spin axis, bracketing the as-built 145 mm.
rtip_lo = rinner + 0.020;   rtip_hi = rinner + 0.120;

% Smooth grids for the 2-D heatmaps (any two variables -> xy-plane)
gs.rtip  = linspace(rtip_lo, rtip_hi, 45);
gs.beta  = deg2rad(linspace(5, 50, 45));
gs.zdot0 = linspace(-0.5, -5.0, 45);

% Coarser grid for the exhaustive 3-D optimum search
g3.rtip  = linspace(rtip_lo, rtip_hi, 10);
g3.beta  = deg2rad(linspace(5, 50, 10));
g3.zdot0 = linspace(-0.5, -5.0, 10);

% Grid for iso-color surfaces in the full 3-D design space
giso.rtip  = linspace(rtip_lo, rtip_hi, 18);
giso.beta  = deg2rad(linspace(5, 50, 18));
giso.zdot0 = linspace(-0.5, -5.0, 18);

%% ===== 2-D heatmaps: choose any two variables for the xy-plane =====
% Each call plots the hop (zdot_exit) and the retained spin (omega_exit) over
% a variable pair, with the third held at nominal.  The controller floor is
% drawn as a contour and the best feasible point (max hop with
% omega_exit >= floor) is marked.  Swap the first two arguments to view any
% other pair.
heatmap_pair('beta','zdot0', gs, nom, params);    % strongest energy levers
heatmap_pair('rtip','beta',  gs, nom, params);    % geometry pair

%% ===== 4-D iso-color surface points: all three design variables at once =====
% x/y/z are the three design variables; each point lies on a hop isosurface.
isocolor_surface_points(giso, params);

%% ===== Global optimum across all three variables =====
optimize_hop(g3, params);

% =========================================================================
%  Helper functions
% =========================================================================

%% Run one case and return time series + exit values
function [time, omega_t, zdot_t, omega_exit, zdot_exit, t_exit] = run_case(rtip, beta, zdot0, p, rinner)
    % rinner is optional: omit it to use the vehicle default in p.rinner,
    % or pass a value to sweep the foil root radius like any other variable.
    if nargin < 5 || isempty(rinner)
        rinner = p.rinner;
    end
    [time, ~, zdot_t, omega_t, ~, zdot_exit, omega_exit, t_exit] = simulate_water_skipping( ...
        rinner, rtip, beta, p.N, p.cf, p.m, p.I, p.rho, p.g, p.omega0, p.z0, zdot0, p.t, p.dt);
end

%% Fixed-rtip beta-zdot plane with trajectories colored by exit speed
function plot_fixed_rtip_exit_speed_scatter(rtip_fixed, params)
    beta_min = 10;   beta_max = 45;  % deg -- brackets the as-built 30 deg
    zin_min  = -3.0; zin_max  = -1.0; % m/s -- brackets the measured entries
    n_beta = 3;
    n_zin = 3;

    [B, Zin] = meshgrid(linspace(beta_min, beta_max, n_beta), ...
                        linspace(zin_min, zin_max, n_zin));
    beta_deg = B(:);
    zdot_in = Zin(:);
    n_cases = numel(beta_deg);

    if isempty(rtip_fixed) || isnan(rtip_fixed)
        % Default to the as-built tip radius. (select_balanced_fixed_rtip
        % below used to pick a value giving an even split of ejected and
        % failed contacts; at as-built conditions every case ejects, so
        % there is no balance to find and the as-built geometry is the
        % honest choice for a figure describing this vehicle.)
        rtip_fixed = params.rinner + 0.070;      % 145 mm from the spin axis
    end

    data = struct('valid',{}, 'exit',{}, 'beta_deg',{}, 'zdot_in',{}, ...
                  't_ms',{}, 'zdot',{}, 'omega_deg',{}, 'zdot_exit',{});
    zexit = nan(n_cases, 1);
    color_metric = nan(n_cases, 1);
    success = false(n_cases, 1);
    x_end = 0;

    fprintf('\nFixed-rtip beta-zdot plane trajectories (r_tip = %.0f mm):\n', rtip_fixed*1e3);
    for k = 1:n_cases
        [time, omega_t, zdot_t, omega_exit, zdot_exit, ~] = ...
            run_case(rtip_fixed, deg2rad(beta_deg(k)), zdot_in(k), params);

        data(k).valid = numel(omega_t) == numel(time);
        data(k).exit = data(k).valid && ~isnan(omega_exit);
        data(k).beta_deg = beta_deg(k);
        data(k).zdot_in = zdot_in(k);

        if data(k).valid
            data(k).t_ms = time(:) * 1e3;
            data(k).zdot = zdot_t(:);
            data(k).omega_deg = rad2deg(omega_t(:));
            color_metric(k) = max(data(k).zdot);
            x_end = max(x_end, max(data(k).t_ms));
        end

        if data(k).exit
            data(k).zdot_exit = zdot_exit;
            zexit(k) = zdot_exit;
            color_metric(k) = zdot_exit;
            success(k) = true;
        end
    end

    fprintf('  successful exits = %d / %d\n', nnz(success), n_cases);
    fprintf('  no exit          = %d / %d\n', nnz(~success), n_cases);
    if ~any(success)
        warning('No successful exits were found for the selected fixed-rtip plane samples.');
        return;
    end

    zmin = min(zexit(success));
    zmax = max(zexit(success));
    fprintf('  z''_exit range   = %.2f to %.2f m/s\n', zmin, zmax);

    base_cmap = turbo(256);
    case_colors = base_cmap(round(linspace(34, 224, n_cases)), :);

    valid_flags = [data.valid].';
    exit_flags = [data.exit].';
    success_idx = find(valid_flags & exit_flags);
    fail_idx = find(valid_flags & ~exit_flags);
    x_end_success = 0;
    x_end_fail = 0;
    for k = success_idx(:).'
        x_end_success = max(x_end_success, max(data(k).t_ms));
    end
    for k = fail_idx(:).'
        x_end_fail = max(x_end_fail, max(data(k).t_ms));
    end

    % Single panel: at as-built conditions every case in this plane ejects,
    % so there is no failed-contact set to separate out. All trajectories are
    % drawn together -- solid for vertical velocity (left axis), dashed for
    % spin (right axis) -- with a filled marker at water exit and a cross if
    % a case ever fails to exit.
    all_idx = find(valid_flags);
    x_end_all = 0;
    for k = all_idx(:).'
        x_end_all = max(x_end_all, max(data(k).t_ms));
    end

    % Figure is sized to the plot alone: the trajectories carry a legend
    % inside the axes rather than a separate label panel beside them, so
    % there is no empty margin to crop.
    fig = figure('Name','fixed_rtip_beta_zdot_trajectories', ...
        'Color','w', 'Units','centimeters', 'Position',[2 2 11.5 8.0], ...
        'PaperUnits','centimeters', 'PaperSize',[11.5 8.0], ...
        'PaperPosition',[0 0 11.5 8.0], 'PaperPositionMode','manual');

    ax = axes(fig, 'Position',[0.105 0.135 0.775 0.845]);
    hold(ax,'on'); grid(ax,'on');
    format_journal_axis(ax);

    h_leg = gobjects(0);   % one handle per case, for the legend
    leg   = {};

    yyaxis(ax,'left');
    for k = all_idx(:).'
        clr = case_colors(k,:);
        hl = plot(ax, data(k).t_ms, data(k).zdot, '-', 'Color',clr, 'LineWidth',1.15);
        h_leg(end+1) = hl; %#ok<AGROW>
        leg{end+1} = sprintf('\beta=%.0f^\circ, z''_{in}=%.0f', ...
            data(k).beta_deg, data(k).zdot_in); %#ok<AGROW>
        if data(k).exit
            plot(ax, data(k).t_ms(end), data(k).zdot(end), 'o', 'Color',clr, ...
                'MarkerFaceColor',clr, 'MarkerSize',4.0, 'HandleVisibility','off');
        else
            plot(ax, data(k).t_ms(end), data(k).zdot(end), 'x', 'Color',clr, ...
                'MarkerSize',5.5, 'LineWidth',1.0, 'HandleVisibility','off');
        end
    end
    yline(ax, 0, ':', 'Color',[0.45 0.45 0.45], 'LineWidth',0.75, ...
        'HandleVisibility','off');
    ylabel(ax, 'z''(t) [m s^{-1}]');

    yyaxis(ax,'right');
    for k = all_idx(:).'
        clr = case_colors(k,:);
        plot(ax, data(k).t_ms, data(k).omega_deg, '--', 'Color',clr, ...
            'LineWidth',1.00, 'HandleVisibility','off');
        if data(k).exit
            plot(ax, data(k).t_ms(end), data(k).omega_deg(end), 'o', 'Color',clr, ...
                'MarkerFaceColor',clr, 'MarkerSize',4.0, 'HandleVisibility','off');
        else
            plot(ax, data(k).t_ms(end), data(k).omega_deg(end), 'x', 'Color',clr, ...
                'MarkerSize',5.5, 'LineWidth',1.0, 'HandleVisibility','off');
        end
    end
    yline(ax, rad2deg(params.omega_min), '--', 'Color',[0.45 0.45 0.45], ...
        'LineWidth',0.75, 'HandleVisibility','off');
    ylabel(ax, '\omega(t) [deg s^{-1}]');

    xlabel(ax, 'time from water entry, t [ms]');
    xlim(ax, [0 max(x_end_all * 1.08, x_end_all + 0.5)]);
    ax.YAxis(1).Color = [0.18 0.18 0.18];
    ax.YAxis(2).Color = [0.18 0.18 0.18];

    yyaxis(ax,'left');
    lg = legend(ax, h_leg, leg, 'Location','southeast', 'NumColumns',3, ...
        'FontName','Arial', 'FontSize',6.2, 'Box','off');
    lg.ItemTokenSize = [10 4];


    drawnow;
    add_top_border(fig, ax);

    out_dir = 'matlab_figures_fixed_span_plane';
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    out_name = sprintf('fixed_span%.0f_velocity_omega', rtip_fixed*1e3);
    exportgraphics(fig, fullfile(out_dir, [out_name '.png']), 'Resolution', 300);
    savefig(fig, fullfile(out_dir, [out_name '.fig']));
    fprintf('  saved figure     = %s\n', fullfile(out_dir, [out_name '.png']));
end

function rtip_best = select_balanced_fixed_rtip(candidate_rtips, beta_deg, zdot_in, params)
    n_cases = numel(beta_deg);
    target_success = n_cases / 2;
    best_score = inf;
    rtip_best = candidate_rtips(1);
    best_success = 0;

    fprintf('\nSelecting fixed r_tip/span for balanced exit/no-exit samples:\n');
    for i = 1:numel(candidate_rtips)
        rtip = candidate_rtips(i);
        success_count = 0;
        valid_count = 0;

        for k = 1:n_cases
            [time, omega_t, ~, omega_exit, ~, ~] = ...
                run_case(rtip, deg2rad(beta_deg(k)), zdot_in(k), params);
            valid = numel(omega_t) == numel(time);
            valid_count = valid_count + valid;
            success_count = success_count + (valid && ~isnan(omega_exit));
        end

        fail_count = valid_count - success_count;
        balance_score = abs(success_count - target_success);
        invalid_penalty = n_cases - valid_count;
        score = balance_score + invalid_penalty;

        fprintf('  r_tip/span = %.0f mm: exit = %d, no exit = %d, invalid = %d\n', ...
            rtip*1e3, success_count, fail_count, invalid_penalty);

        if score < best_score || (score == best_score && success_count > best_success)
            best_score = score;
            rtip_best = rtip;
            best_success = success_count;
        end
    end

    fprintf('  selected r_tip/span = %.0f mm (exit = %d, no exit = %d)\n', ...
        rtip_best*1e3, best_success, n_cases - best_success);
end

function format_journal_axis(ax)
    set(ax, 'FontName','Arial', 'FontSize',8.2, 'LineWidth',0.75, ...
        'Box','off', ...
        'TickDir','out', 'Layer','top', 'XMinorTick','on', 'YMinorTick','on', ...
        'GridColor',[0.82 0.82 0.82], 'GridAlpha',0.42, ...
        'MinorGridColor',[0.92 0.92 0.92], 'MinorGridAlpha',0.16);
end

function add_top_border(fig, ax)
    old_units = ax.Units;
    ax.Units = 'normalized';
    pos = ax.Position;
    ax.Units = old_units;
    annotation(fig, 'line', [pos(1) pos(1)+pos(3)], ...
        [pos(2)+pos(4) pos(2)+pos(4)], ...
        'Color',[0.18 0.18 0.18], 'LineWidth',0.75);
end

function add_parameter_label_panel(fig, data, case_colors, valid_flags)
    % Sits to the right of the single trajectory panel. The x limits are set
    % to 1 so that the swatch and text positions below are read directly as
    % fractions of the panel width -- with a wider range the labels bunch
    % into the left edge and leave the panel looking empty.
    ax_label = axes(fig, 'Position',[0.655 0.17 0.335 0.75]);
    hold(ax_label,'on');
    axis(ax_label, 'off');
    xlim(ax_label, [0 1]);
    ylim(ax_label, [0 1]);

    idx = find(valid_flags);
    y_pos = linspace(0.94, 0.06, numel(idx));
    for ii = 1:numel(idx)
        k = idx(ii);
        clr = case_colors(k,:);
        plot(ax_label, [0.02 0.16], [y_pos(ii) y_pos(ii)], '-', ...
            'Color',clr, 'LineWidth',2.4);
        label = sprintf('\\beta = %.0f^\\circ,  z''_{in} = %.0f m s^{-1}', ...
            data(k).beta_deg, data(k).zdot_in);
        text(ax_label, 0.21, y_pos(ii), label, 'Interpreter','tex', ...
            'FontName','Arial', 'FontSize',8.0, 'Color',[0.18 0.18 0.18], ...
            'HorizontalAlignment','left', 'VerticalAlignment','middle');
    end
end

%% Plot requested entry-to-exit case with comparison cases, including failures
function plot_entry_exit_case(rtip, beta, zdot0, params)
    % Comparison cases around the as-built point. rtip values are measured
    % from the spin axis, so they read rinner + span.
    ri = params.rinner;
    cases = struct( ...
        'rtip',  {rtip, ri+0.070, ri+0.070, ri+0.045, ri+0.020}, ...
        'beta',  {beta, deg2rad(15), deg2rad(45), deg2rad(45), deg2rad(45)}, ...
        'zdot0', {zdot0, -1.5, -2.5, -3.0, -3.0}, ...
        'label', {'S1 as-built', 'S2 145/15/-1.5', 'S3 145/45/-2.5', ...
                  'F1 120/45/-3', 'F2 95/45/-3'});

    gray = [0.42 0.42 0.42];
    black = [0.18 0.18 0.18];
    case_colors = [ ...
        0.00 0.28 0.80;  % S1: blue
        0.86 0.18 0.12;  % S2: red-orange
        0.00 0.55 0.25;  % S3: green
        0.55 0.20 0.75;  % F1: purple
        0.90 0.58 0.00]; % F2: amber
    omega_style = '-';
    zdot_style  = '--';
    line_widths = [1.65 1.45 1.45 1.45 1.45];

    data = struct('valid',{}, 'exit',{}, 't_ms',{}, 'omega_deg',{}, ...
                  'zdot',{}, 'omega_exit',{}, 'zdot_exit',{}, 't_exit',{});
    all_omega = rad2deg(params.omega_min);
    all_zdot = 0;
    x_end = 0;

    fprintf('\nEntry-to-exit comparison cases:\n');
    for i = 1:numel(cases)
        [time, omega_t, zdot_t, omega_exit, zdot_exit, t_exit] = ...
            run_case(cases(i).rtip, cases(i).beta, cases(i).zdot0, params);

        data(i).valid = numel(omega_t) == numel(time);
        if ~data(i).valid
            % Keep the slot populated so later indexing (e.g. data(1)) does
            % not fail when the leading case is the one that is invalid.
            data(i).exit = false;
            data(i).t_ms = [];  data(i).omega_deg = [];  data(i).zdot = [];
            data(i).omega_exit = NaN;  data(i).zdot_exit = NaN;
            data(i).t_exit = NaN;
            fprintf('  %-15s : invalid geometry (needs rtip > rinner = %.0f mm)\n', ...
                cases(i).label, params.rinner*1e3);
            continue;
        end

        data(i).exit = ~isnan(omega_exit);
        data(i).t_ms = time(:) * 1e3;
        data(i).omega_deg = rad2deg(omega_t(:));
        data(i).zdot = zdot_t(:);
        data(i).omega_exit = omega_exit;
        data(i).zdot_exit = zdot_exit;
        data(i).t_exit = t_exit;

        all_omega = [all_omega; data(i).omega_deg]; %#ok<AGROW>
        all_zdot = [all_zdot; data(i).zdot]; %#ok<AGROW>
        x_end = max(x_end, max(data(i).t_ms));

        if data(i).exit
            fprintf('  %-15s : EXIT, omega_exit = %.0f deg/s, z''_exit = %+.2f m/s, t = %.1f ms\n', ...
                cases(i).label, rad2deg(omega_exit), zdot_exit, t_exit*1e3);
        else
            fprintf('  %-15s : NO EXIT, final omega = %.0f deg/s, final z'' = %+.2f m/s, t = %.1f ms\n', ...
                cases(i).label, rad2deg(omega_t(end)), zdot_t(end), time(end)*1e3);
        end
    end

    if ~any([data.valid])
        warning('plot_entry_exit_case: no valid cases to plot.');
        return;
    end

    fig = figure('Name','entry_to_exit_cases', ...
        'Color','w', 'Units','centimeters', 'Position',[2 2 18.0 8.0], ...
        'PaperUnits','centimeters', 'PaperSize',[18.0 8.0], ...
        'PaperPosition',[0 0 18.0 8.0], 'PaperPositionMode','manual');

    axis_pos = [0.105 0.30 0.765 0.60];
    ax = axes(fig, 'Position',axis_pos);
    hold(ax,'on'); grid(ax,'on'); box(ax,'off');
    set(ax, 'FontName','Arial', 'FontSize',8.5, 'LineWidth',0.75, ...
        'TickDir','out', 'Layer','top', 'XMinorTick','on', 'YMinorTick','on', ...
        'GridColor',[0.80 0.80 0.80], 'GridAlpha',0.45, ...
        'MinorGridColor',[0.90 0.90 0.90], 'MinorGridAlpha',0.18);

    x_label = x_end * 1.025;
    xlim(ax, [0 max(x_end * 1.10, x_end + 0.5)]);

    omega_min_deg = rad2deg(params.omega_min);
    omega_span = max(all_omega) - min([all_omega; omega_min_deg]);
    omega_pad = max(120, 0.10 * omega_span);
    omega_ylim = [max(0, floor((min([all_omega; omega_min_deg]) - omega_pad) / 100) * 100), ...
                  ceil((max(all_omega) + omega_pad) / 100) * 100];

    zdot_span = max([all_zdot; 0]) - min([all_zdot; 0]);
    zdot_pad = max(0.25, 0.10 * zdot_span);
    zdot_ylim = [floor((min([all_zdot; 0]) - zdot_pad) * 2) / 2, ...
                 ceil((max([all_zdot; 0]) + zdot_pad) * 2) / 2];

    yyaxis(ax,'left');
    zdot_to_omega_axis = @(v) omega_ylim(1) + ...
        (v - zdot_ylim(1)) ./ diff(zdot_ylim) .* diff(omega_ylim);
    if data(1).valid
        hshade = fill(ax, [data(1).t_ms; flipud(data(1).t_ms)], ...
            [zdot_to_omega_axis(data(1).zdot); ...
             flipud(zdot_to_omega_axis(zeros(size(data(1).zdot))))], ...
            [0.88 0.88 0.88], 'EdgeColor','none', 'FaceAlpha',0.42, ...
            'HandleVisibility','off');
        uistack(hshade, 'bottom');
    end

    for i = 1:numel(cases)
        if ~data(i).valid
            continue;
        end
        case_color = case_colors(i,:);
        plot(ax, data(i).t_ms, data(i).omega_deg, omega_style, ...
            'LineWidth',line_widths(i), 'Color',case_color, 'HandleVisibility','off');
        if data(i).exit
            plot(ax, data(i).t_ms(end), data(i).omega_deg(end), 'o', ...
                'MarkerSize',4.2, 'MarkerEdgeColor',case_color, ...
                'MarkerFaceColor',case_color, 'HandleVisibility','off');
        else
            plot(ax, data(i).t_ms(end), data(i).omega_deg(end), 'x', ...
                'MarkerSize',6.0, 'LineWidth',1.1, 'Color',case_color, ...
                'HandleVisibility','off');
        end
    end
    ylim(ax, omega_ylim);
    ylabel(ax, '\omega [deg s^{-1}]', 'Color',black);
    yline(ax, omega_min_deg, '--', 'Color',gray, 'LineWidth',0.75, 'HandleVisibility','off');
    text(ax, x_label, data(1).omega_deg(end), '\omega', 'Color',black, ...
        'FontName','Arial', 'FontSize',9, 'FontWeight','bold', ...
        'HorizontalAlignment','left', 'VerticalAlignment','middle');
    text(ax, x_end * 0.98, omega_min_deg + 0.025 * diff(omega_ylim), ...
        '\omega_{min}', 'Color',gray, 'FontName','Arial', 'FontSize',7.5, ...
        'HorizontalAlignment','right', 'VerticalAlignment','bottom', ...
        'BackgroundColor','w', 'Margin',1);

    yyaxis(ax,'right');
    for i = 1:numel(cases)
        if ~data(i).valid
            continue;
        end
        case_color = case_colors(i,:);
        plot(ax, data(i).t_ms, data(i).zdot, zdot_style, ...
            'LineWidth',line_widths(i), 'Color',case_color, 'HandleVisibility','off');
        if data(i).exit
            plot(ax, data(i).t_ms(end), data(i).zdot(end), 's', ...
                'MarkerSize',4.2, 'MarkerEdgeColor',case_color, ...
                'MarkerFaceColor','w', 'HandleVisibility','off');
        else
            plot(ax, data(i).t_ms(end), data(i).zdot(end), 'x', ...
                'MarkerSize',6.0, 'LineWidth',1.1, 'Color',case_color, ...
                'HandleVisibility','off');
        end
    end
    ylim(ax, zdot_ylim);
    ylabel(ax, 'z'' [m s^{-1}]', 'Color',black);
    yline(ax, 0, ':', 'Color',gray, 'LineWidth',0.85, 'HandleVisibility','off');
    text(ax, x_label, data(1).zdot(end), 'z''', 'Color',black, ...
        'FontName','Arial', 'FontSize',9, 'FontWeight','bold', ...
        'HorizontalAlignment','left', 'VerticalAlignment','middle');
    text(ax, x_end * 0.82, 0 + 0.055 * diff(zdot_ylim), ...
        'z'' = 0', 'Color',gray, 'FontName','Arial', 'FontSize',7.5, ...
        'HorizontalAlignment','right', 'VerticalAlignment','bottom');

    yyaxis(ax,'left');
    xlabel(ax, 'time from water entry, t [ms]');
    ax.YAxis(1).Color = black;
    ax.YAxis(2).Color = black;

    if data(1).exit
        exit_ms = data(1).t_exit * 1e3;
        xline(ax, exit_ms, '--', 'Color',[0.10 0.10 0.10], ...
            'LineWidth',0.85, 'HandleVisibility','off');
    end

    legend_handles = gobjects(1, numel(cases) + 2);
    legend_labels = cell(1, numel(cases) + 2);
    legend_handles(1) = plot(ax, NaN, NaN, omega_style, ...
        'Color',black, 'LineWidth',1.45);
    legend_labels{1} = '\omega';
    legend_handles(2) = plot(ax, NaN, NaN, zdot_style, ...
        'Color',black, 'LineWidth',1.45);
    legend_labels{2} = 'z''';
    for i = 1:numel(cases)
        legend_handles(i+2) = plot(ax, NaN, NaN, '-', ...
            'Color',case_colors(i,:), 'LineWidth',line_widths(i));
        if data(i).valid && data(i).exit
            legend_labels{i+2} = [cases(i).label ' exit'];
        else
            legend_labels{i+2} = [cases(i).label ' no exit'];
        end
    end
    lgd = legend(ax, legend_handles, legend_labels, 'Location','southoutside', ...
        'NumColumns',4, 'Box','off', 'FontSize',6.8);
    lgd.ItemTokenSize = [16 8];
    lgd.Units = 'normalized';
    lgd.Position = [0.165 0.040 0.70 0.115];
    ax.Position = axis_pos;

    annotation(fig, 'rectangle', axis_pos, ...
        'Color',[0.15 0.15 0.15], 'LineWidth',0.75);
end

%% Sweep one variable, plot omega(t) and zdot(t) families, return exit metrics
function [exit_omega_deg, exit_zdot] = sweep_and_plot(field, vals, dispvals, fmt, nom, params)
    n = numel(vals);
    exit_omega_deg = nan(1,n);
    exit_zdot      = nan(1,n);
    cmap = lines(n);

    figure('Name', sprintf('Sweep: %s', field));
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    ax1 = nexttile; hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
    ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');

    h_lines = gobjects(0);   % handles of plotted curves, for the legend
    leg     = {};

    fprintf('\nSweep over %s:\n', field);
    for i = 1:n
        cse = nom;
        cse.(field) = vals(i);
        if cse.rtip <= cse.rinner
            fprintf('  %-14s : skipped, r_tip (%.0f mm) <= r_in (%.0f mm)\n', ...
                sprintf(fmt, dispvals(i)), cse.rtip*1e3, cse.rinner*1e3);
            continue;
        end
        [time, omega_t, zdot_t, omega_exit, zdot_exit, t_exit] = ...
            run_case(cse.rtip, cse.beta, cse.zdot0, params, cse.rinner);

        % Skip only truly invalid runs (e.g. bad geometry -> length mismatch)
        if numel(omega_t) ~= numel(time)
            fprintf('  %-14s : invalid geometry\n', sprintf(fmt, dispvals(i)));
            continue;
        end

        % Always plot the trajectory, exit or not
        hl = plot(ax1, time*1e3, rad2deg(omega_t), 'Color', cmap(i,:), 'LineWidth', 1.3);
        plot(ax2, time*1e3, zdot_t, 'Color', cmap(i,:), 'LineWidth', 1.3);
        h_lines(end+1) = hl; %#ok<AGROW>

        if isnan(omega_exit)
            leg{end+1} = sprintf([fmt '  (no exit)'], dispvals(i)); %#ok<AGROW>
            fprintf('  %-14s : NO EXIT (stays submerged)\n', sprintf(fmt, dispvals(i)));
        else
            exit_omega_deg(i) = rad2deg(omega_exit);
            exit_zdot(i)      = zdot_exit;
            plot(ax1, t_exit*1e3, rad2deg(omega_exit), 'o', ...
                'Color', cmap(i,:), 'MarkerFaceColor', cmap(i,:));
            plot(ax2, t_exit*1e3, zdot_exit, 'o', ...
                'Color', cmap(i,:), 'MarkerFaceColor', cmap(i,:));
            leg{end+1} = sprintf([fmt '  (exit @ %.0f ms)'], dispvals(i), t_exit*1e3); %#ok<AGROW>
            fprintf('  %-14s : omega_exit = %5.0f deg/s,  zdot_exit = %+.2f m/s,  t = %.1f ms\n', ...
                sprintf(fmt, dispvals(i)), rad2deg(omega_exit), zdot_exit, t_exit*1e3);
        end
    end

    yline(ax1, rad2deg(params.omega_min), 'k--', 'controller min');
    yline(ax2, 0, 'k:');
    xlabel(ax1,'time [ms]'); ylabel(ax1,'\omega [deg/s]');
    xlabel(ax2,'time [ms]'); ylabel(ax2,'z'' [m/s]');
    title(ax1, sprintf('\\omega(t), sweeping %s', field));
    title(ax2, sprintf('z''(t), sweeping %s', field));
    if ~isempty(h_lines)
        legend(ax1, h_lines, leg, 'Location','eastoutside');
    end
end

%% Display scaling and axis label for a given variable
function [d, lab] = var_disp(field, v)
    switch field
        case 'rtip';  d = v*1e3;      lab = 'r_{tip} [mm]';
        case 'beta';  d = rad2deg(v); lab = '\beta [deg]';
        case 'zdot0'; d = v;          lab = 'z'' into water [m/s]';
        otherwise;    d = v;          lab = field;
    end
end

%% Sort vectors and matrix rows/columns so imagesc displays axes normally
function [x_img, y_img, Z_img] = sort_for_heatmap(x, y, Z)
    [x_img, ix] = sort(x(:).');
    [y_img, iy] = sort(y(:));
    Z_img = Z(iy, ix);
end

%% 2-D heatmaps of hop and retained spin over a pair of variables
function heatmap_pair(fieldX, fieldY, gridv, nom, params)
    vx = gridv.(fieldX);  nX = numel(vx);
    vy = gridv.(fieldY);  nY = numel(vy);
    Zhop  = nan(nY, nX);   % zdot_exit  [m/s]
    Zspin = nan(nY, nX);   % omega_exit [deg/s]

    for r = 1:nY
        for c = 1:nX
            cse = nom;
            cse.(fieldX) = vx(c);
            cse.(fieldY) = vy(r);
            [~,~,~, we, ze, ~] = run_case(cse.rtip, cse.beta, cse.zdot0, params, cse.rinner);
            if ~isnan(we)
                Zhop(r,c)  = ze;
                Zspin(r,c) = rad2deg(we);
            end
        end
    end

    [dx, labx] = var_disp(fieldX, vx);
    [dy, laby] = var_disp(fieldY, vy);
    [DX, DY]   = meshgrid(dx, dy);
    omin_deg   = rad2deg(params.omega_min);

    % Best feasible design in this slice (max hop with omega_exit >= floor)
    Zhop_feas = Zhop;
    Zhop_feas(isnan(Zspin) | Zspin < omin_deg) = NaN;
    feasible_vals = Zhop_feas(:);
    feasible_idx  = find(~isnan(feasible_vals));
    if isempty(feasible_idx)
        best = NaN;
        idx  = 1;
    else
        [best, rel_idx] = max(feasible_vals(feasible_idx));
        idx = feasible_idx(rel_idx);
    end

    % Fill non-hopping points with 0 so each heatmap spans the whole quadrant
    % (out to all four borders) instead of leaving holes.
    Zhop_plot  = Zhop;   Zhop_plot(isnan(Zhop_plot))   = 0;
    Zspin_plot = Zspin;  Zspin_plot(isnan(Zspin_plot)) = 0;

    [dx_img, dy_img, Zhop_img]  = sort_for_heatmap(dx, dy, Zhop_plot);
    [~,      ~,      Zspin_img] = sort_for_heatmap(dx, dy, Zspin_plot);

    figure('Name', sprintf('Heatmaps: %s vs %s', fieldX, fieldY));
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

    % --- Hop heatmap ---
    ax1 = nexttile;
    imagesc(ax1, dx_img, dy_img, Zhop_img); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
    axis(ax1,'xy'); axis(ax1,'tight');
    if ~isnan(best)
        plot(ax1, DX(idx), DY(idx), 'rp', 'MarkerSize',14, 'MarkerFaceColor','r');
    end
    xlabel(ax1, labx); ylabel(ax1, laby);
    title(ax1, 'Hop: exit vertical velocity  (0 = no hop)');
    cb1 = colorbar(ax1); ylabel(cb1, 'z''_{exit} [m/s]');

    % --- Retained-spin heatmap with controller floor contour ---
    ax2 = nexttile;
    imagesc(ax2, dx_img, dy_img, Zspin_img); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
    axis(ax2,'xy'); axis(ax2,'tight');
    if any(Zspin_plot(:) <= omin_deg) && any(Zspin_plot(:) >= omin_deg)
        contour(ax2, DX, DY, Zspin_plot, [omin_deg omin_deg], 'k--', 'LineWidth',1.5);
    end
    xlabel(ax2, labx); ylabel(ax2, laby);
    title(ax2, sprintf('Retained spin (black contour = %.0f deg/s;  0 = no hop)', omin_deg));
    cb2 = colorbar(ax2); ylabel(cb2, '\omega_{exit} [deg/s]');

    if ~isnan(best)
        fprintf('[heatmap %s vs %s] best feasible hop z''_exit = %.2f m/s at %s = %.3g, %s = %.3g\n', ...
            fieldX, fieldY, best, strtrim(labx), DX(idx), strtrim(laby), DY(idx));
    else
        fprintf('[heatmap %s vs %s] no feasible point in this slice\n', fieldX, fieldY);
    end
end

%% Points sampled from iso-color surfaces in the full design space
function isocolor_surface_points(g, params)
    omin_deg = rad2deg(params.omega_min);
    nS = numel(g.rtip);
    nB = numel(g.beta);
    nZ = numel(g.zdot0);
    ncases = nS*nB*nZ;

    rtip_grid = g.rtip*1e3;
    beta_grid = rad2deg(g.beta);
    zdot_grid = g.zdot0;
    HopFeasVol = zeros(nB,nS,nZ);  % feasible hop, with omega_exit >= floor
    SpinVol = zeros(nB,nS,nZ);     % all valid exits, 0 = no exit

    fprintf('\n4-D iso-color surface evaluation over %d designs...\n', ncases);
    for is = 1:nS
      for ib = 1:nB
        for iz = 1:nZ
          [~,~,~, we, ze, ~] = run_case(g.rtip(is), g.beta(ib), g.zdot0(iz), params);
          if ~isnan(we)
              spin_deg = rad2deg(we);
              SpinVol(ib,is,iz) = spin_deg;
              if spin_deg >= omin_deg
                  HopFeasVol(ib,is,iz) = max(ze, 0);
              end
          end
        end
      end
    end

    fig = figure('Name','4D Iso-color Surface Panels: hop and omega', ...
        'Color','w', 'Units','centimeters', 'Position',[2 2 22.0 10.4]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing','compact', 'Padding','compact');

    ax1 = nexttile(tl, 1);
    plot_isosurface_points(ax1, rtip_grid, beta_grid, zdot_grid, HopFeasVol, SpinVol, ...
        'z''_{exit} [m/s]', '[hop isosurfaces]', '%.2f', 'm/s');

    ax2 = nexttile(tl, 2);
    plot_isosurface_points(ax2, rtip_grid, beta_grid, zdot_grid, SpinVol, SpinVol, ...
        '\omega_{exit} [deg/s]', '[omega isosurfaces]', '%.0f', 'deg/s');
end

%% Draw iso-value surfaces as colored point clouds
function plot_isosurface_points(ax, rtip_grid, beta_grid, zdot_grid, ValueVol, SpinVol, cb_label, prefix, level_fmt, units)
    valid_values = ValueVol(ValueVol > 0);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    set(ax, 'Color','w', 'GridAlpha',0.18, 'LineWidth',0.55, ...
        'FontSize',8, 'FontName','Arial', 'TickDir','out');
    colormap(ax, parula);

    if isempty(valid_values)
        text(ax, 0.5, 0.5, 0.5, 'No iso-surfaces found');
        colorbar(ax);
        fprintf('%s no positive values in this grid\n', prefix);
    else
        vmin = min(valid_values);
        vmax = max(valid_values);
        [RTIP, BETA, ZDOT] = meshgrid(rtip_grid, beta_grid, zdot_grid);

        sorted_values = sort(valid_values(:));
        q = [0.18 0.35 0.52 0.69 0.86 0.96];
        level_idx = max(1, min(numel(sorted_values), round(q*numel(sorted_values))));
        levels = unique(sorted_values(level_idx));

        clim(ax, [vmin vmax]);
        cb = colorbar(ax);
        cb.FontSize = 8;
        cb.FontName = 'Arial';
        cb.LineWidth = 0.55;
        ylabel(cb, cb_label, 'FontSize',9, 'FontName','Arial');

        cmap = colormap(ax);
        max_points_per_level = 900;
        for il = 1:numel(levels)
            lev = levels(il);
            fv = isosurface(RTIP, BETA, ZDOT, ValueVol, lev);
            if isempty(fv.faces) || isempty(fv.vertices)
                continue;
            end

            cidx = 1 + round((lev - vmin) / (vmax - vmin) * (size(cmap,1)-1));
            cidx = min(max(cidx, 1), size(cmap,1));
            level_color = cmap(cidx,:);
            patch(ax, 'Faces',fv.faces, 'Vertices',fv.vertices, ...
                'FaceColor',level_color, 'FaceAlpha',0.035, 'EdgeColor','none');

            faces = fv.faces;
            verts = fv.vertices;
            pts = (verts(faces(:,1),:) + verts(faces(:,2),:) + verts(faces(:,3),:)) / 3;
            if size(pts,1) > max_points_per_level
                keep = round(linspace(1, size(pts,1), max_points_per_level));
                pts = pts(keep,:);
            end

            level_norm = (lev - vmin) / (vmax - vmin);
            marker_size = 8 + 34*level_norm.^1.7;
            h_iso = scatter3(ax, pts(:,1), pts(:,2), pts(:,3), ...
                marker_size, lev*ones(size(pts,1),1), 'o', 'LineWidth',0.45);
            h_iso.MarkerFaceColor = 'none';
            h_iso.MarkerEdgeAlpha = 0.86;
        end

        [best, idx] = max(ValueVol(:));
        [ib, is, iz] = ind2sub(size(ValueVol), idx);

        level_text = sprintf([level_fmt ' '], levels);
        fprintf('%s levels = %s%s\n', prefix, level_text, units);
        fprintf('%s max value = %.3g %s at r_tip = %.1f mm, beta = %.1f deg, zdot0 = %.2f m/s, omega_exit = %.0f deg/s\n', ...
            prefix, best, units, rtip_grid(is), beta_grid(ib), zdot_grid(iz), SpinVol(ib,is,iz));
    end

    xlabel(ax, 'r_{tip} [mm]', 'FontSize',9, 'FontName','Arial');
    ylabel(ax, '\beta [deg]', 'FontSize',9, 'FontName','Arial');
    zlabel(ax, 'z'' into water [m/s]', 'FontSize',9, 'FontName','Arial');
    xpad = 0.04 * (max(rtip_grid) - min(rtip_grid));
    ypad = 0.04 * (max(beta_grid) - min(beta_grid));
    zpad = 0.04 * (max(zdot_grid) - min(zdot_grid));
    xlim(ax, [min(rtip_grid)-xpad max(rtip_grid)+xpad]);
    ylim(ax, [min(beta_grid)-ypad max(beta_grid)+ypad]);
    zlim(ax, [min(zdot_grid)-zpad max(zdot_grid)+zpad]);
    view(ax, 135, 20);
    pbaspect(ax, [1.1 1 0.82]);
end

%% Exhaustive 3-D search: max hop subject to omega_exit >= controller floor
function optimize_hop(g, params)
    omin_deg = rad2deg(params.omega_min);
    ncases = numel(g.rtip)*numel(g.beta)*numel(g.zdot0);
    fprintf('\n3-D optimum search over %d designs...\n', ncases);
    best = -inf;  bopt = [];
    for is = 1:numel(g.rtip)
      fprintf('  rtip %d/%d\n', is, numel(g.rtip));
      for ib = 1:numel(g.beta)
        for iz = 1:numel(g.zdot0)
          [~,~,~, we, ze, ~] = run_case(g.rtip(is), g.beta(ib), g.zdot0(iz), params);
          if ~isnan(we) && rad2deg(we) >= omin_deg && ze > best
              best = ze;
              bopt = [g.rtip(is), g.beta(ib), g.zdot0(iz), rad2deg(we)];
          end
        end
      end
    end

    fprintf('\n===== Global optimum: max hop with omega_exit >= %.0f deg/s =====\n', omin_deg);
    if isempty(bopt)
        fprintf('  No feasible design found in the search grid.\n');
    else
        fprintf('  z''_exit (hop)  = %.2f m/s   (hop height ~ %.0f mm)\n', best, 1000*best^2/(2*params.g));
        fprintf('  r_tip          = %.1f mm\n', bopt(1)*1e3);
        fprintf('  beta           = %.1f deg\n', rad2deg(bopt(2)));
        fprintf('  zdot0 (entry)  = %.2f m/s\n', bopt(3));
        fprintf('  omega_exit     = %.0f deg/s\n', bopt(4));
    end
end

%% Simulate one water contact (entry -> submerged -> exit)
function [time, z_history, zdot_history, omega_history, ...
    z_exit, zdot_exit, omega_exit, t_exit] = simulate_water_skipping( ...
    rinner, rtip, beta, N, cf, m, I, rho, g, omega0, z0, zdot0, t, dt)

    z = z0;  zdot = zdot0;  omega = omega0;

    z_history     = zeros(size(t));
    zdot_history  = zeros(size(t));
    omega_history = zeros(size(t));

    z_exit = NaN;  zdot_exit = NaN;  omega_exit = NaN;  t_exit = NaN;
    has_submerged = false;

    for k = 1:length(t)
        z_history(k)     = z;
        zdot_history(k)  = zdot;
        omega_history(k) = omega;

        if z < 0
            has_submerged = true;
        end

        % Hydrodynamic vertical thrust and resisting torque
        [T, Q] = compute_thrust_and_torque(rinner, rtip, beta, N, cf, rho, omega, z, zdot);

        if isnan(T) || isnan(Q)
            time = t(1:k);
            z_history = NaN;  zdot_history = NaN;  omega_history = NaN;
            return;
        end

        % Vertical (upward positive) and rotational dynamics
        zddot    = T / m - g;
        omegadot = -Q / I;

        z_prev = z;  zdot_prev = zdot;  omega_prev = omega;  t_prev = t(k);

        % Semi-implicit Euler
        zdot  = zdot  + zddot * dt;
        z     = z     + zdot  * dt;
        omega = omega + omegadot * dt;
        omega = max(omega, 0);

        % Water exit: z crosses 0 upward after having submerged
        if has_submerged && z_prev < 0 && z >= 0 && zdot > 0
            ratio = (0 - z_prev) / (z - z_prev);
            t_exit     = t_prev + ratio * dt;
            z_exit     = 0;
            zdot_exit  = zdot_prev  + ratio * (zdot  - zdot_prev);
            omega_exit = omega_prev + ratio * (omega - omega_prev);

            time = [t(1:k), t_exit];
            z_history(k+1)     = z_exit;
            zdot_history(k+1)  = zdot_exit;
            omega_history(k+1) = omega_exit;
            z_history     = z_history(1:k+1);
            zdot_history  = zdot_history(1:k+1);
            omega_history = omega_history(1:k+1);
            return;
        end

        % Early termination for hopeless dives (no exit will occur):
        % spin fully drained while still sinking, or plunged far too deep.
        if has_submerged && ((omega <= 0 && zdot <= 0) || z < -0.1)
            time          = t(1:k);
            z_history     = z_history(1:k);
            zdot_history  = zdot_history(1:k);
            omega_history = omega_history(1:k);
            return;
        end
    end

    % No exit within simulation time
    time = t;
    z_history     = z_history(1:length(t));
    zdot_history  = zdot_history(1:length(t));
    omega_history = omega_history(1:length(t));
end

%% Total vertical thrust and resisting torque from N hydrofoils
function [T_total, Q_total] = compute_thrust_and_torque(rinner, rtip, beta, N, cf, rho, omega, z, zdot)
    % z >= 0 : foil above water, no force
    if z >= 0
        T_total = 0;  Q_total = 0;  return;
    end

    h = -z;                     % penetration depth

    if rtip <= rinner || rinner < 0
        T_total = NaN;  Q_total = NaN;  return;     % invalid geometry
    end

    % Submerged chord grows with penetration depth until the plate is fully
    % wetted (depth cf*sin(beta)); beyond that the wetted length is the
    % physical chord cf.
    c_sub = min(h / sin(beta), cf);
    if c_sub <= 0
        T_total = 0;  Q_total = 0;  return;
    end

    % Blade-element integration over the WETTED SPAN ONLY.
    % The foil does not reach the axis of rotation: it begins at the motor
    % mount radius rinner and extends to rtip, so the integral runs
    % rinner -> rtip. Integrating from 0 would add a stretch of foil that
    % does not exist, and because the element force scales as (omega*r)^2
    % the error is not a small one.
    nr = 60;
    r  = linspace(rinner, rtip, nr);

    vx    = omega .* r;                 % tangential velocity
    v     = sqrt(vx.^2 + zdot.^2);      % resultant velocity
    gamma = atan2(-zdot, vx);           % inflow angle (zdot<0 = downward)
    alpha = beta + gamma;               % angle of attack

    % Flat-plate (separated-flow) coefficients
    CL = 2 .* sin(alpha) .* cos(alpha);
    CD = 2 .* sin(alpha).^2;

    integrand_T = 0.5 .* rho .* c_sub      .* v.^2 .* (CL .* cos(gamma) + CD .* sin(gamma));
    integrand_Q = 0.5 .* rho .* c_sub .* r .* v.^2 .* (CD .* cos(gamma) - CL .* sin(gamma));

    T_foil = trapz(r, integrand_T);
    Q_foil = trapz(r, integrand_Q);

    T_total = N * T_foil;
    Q_total = N * Q_foil;
    Q_total = max(Q_total, 0);          % resisting torque is non-negative
end
