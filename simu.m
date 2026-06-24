clear; clc; close all;

% =========================================================================
%  Water-skipping hop study
%  Predicts the spin (omega) and vertical velocity (zdot) of the vehicle
%  through one water contact (entry -> submerged -> exit), and sweeps the
%  four engineering variables we can change:
%       1) chord   (hydrofoil chord, tangential to the sweep)
%       2) span    (hydrofoil radial span; rroot = rtip - span)
%       3) beta    (hydrofoil angle of attack / inclination)
%       4) zdot0   (vertical speed into the water at contact)
%  All other parameters are held constant.
% =========================================================================

%% ===== Fixed parameters (held constant) =====
N    = 4;            % number of hydrofoils
rtip = 7e-2;         % hydrofoil tip radius [m]
m    = 44e-3;        % vehicle mass [kg]
rho  = 1000;         % water density [kg/m^3]
g    = 9.81;         % gravitational acceleration [m/s^2]

% Moment of inertia about the spin axis: uniform 2-D disk of the vehicle mass
%   I = 1/2 * m * R_disk^2.  Change R_disk to update easily.
R_disk = rtip;       % disk radius [m] (assumed = tip radius; easy to change)
I      = 0.5 * m * R_disk^2;

% Spin: entry value and the controller minimum (both given in deg/s)
omega0    = deg2rad(3000);   % entry spin at contact [rad/s] (3000 deg/s)
omega_min = deg2rad(1000);   % controller minimum spin [rad/s] (1000 deg/s)

%% ===== Integration settings =====
z0    = 0;           % water surface position [m]
dt    = 1e-5;        % time step [s]
t_end = 0.3;        % max simulation time [s]
t     = 0:dt:t_end;

%% ===== Nominal values of the four engineering variables =====
nom.chord = 0.010;            % m (10 mm)
nom.span  = 0.045;            % m (30 mm)
nom.beta  = deg2rad(10);      % rad (15 deg)
nom.zdot0 = -1.5;             % m/s (downward)

%% ===== Sweep ranges for each variable =====
chord_list = [0.006 0.010 0.014 0.018];        % m
span_list  = [0.020 0.030 0.040 0.050];        % m
beta_list  = deg2rad([5 10 15 20 25 30]);      % rad
zdot0_list = [-2.0 -3.0 -4.0 -5.0];            % m/s

%% ===== Pack constants into a struct for the helpers =====
params = struct('N',N,'rtip',rtip,'m',m,'I',I,'rho',rho,'g',g, ...
                'omega0',omega0,'omega_min',omega_min,'z0',z0,'t',t,'dt',dt);

%% ===== Report nominal-case result =====
[~,~,~,we_nom,ze_nom,te_nom] = run_case(nom.chord,nom.span,nom.beta,nom.zdot0,params);
fprintf('\nNominal case (chord=%.0f mm, span=%.0f mm, beta=%.0f deg, zdot0=%.1f m/s):\n', ...
    nom.chord*1e3, nom.span*1e3, rad2deg(nom.beta), nom.zdot0);
fprintf('  omega_exit = %.0f deg/s (min %.0f),  zdot_exit = %.3f m/s,  t_contact = %.2f ms\n', ...
    rad2deg(we_nom), rad2deg(omega_min), ze_nom, te_nom*1e3);

%% ===== Four one-at-a-time sweeps (time-series of omega and zdot) =====
[eo_chord, ez_chord] = sweep_and_plot('chord', chord_list, chord_list*1e3,    'c = %.0f mm',     nom, params);
[eo_span,  ez_span ] = sweep_and_plot('span',  span_list,  span_list*1e3,     'span = %.0f mm',  nom, params);
[eo_beta,  ez_beta ] = sweep_and_plot('beta',  beta_list,  rad2deg(beta_list),'\\beta = %.0f deg',nom, params);
[eo_zd,    ez_zd   ] = sweep_and_plot('zdot0', zdot0_list, zdot0_list,        'z'' = %.1f m/s',   nom, params);

%% ===== Optimization: grids for the 2-D surfaces and the 4-D search =====
% Smooth grids for the 3-D surfaces (any two variables -> xy-plane)
gs.chord = linspace(0.006, 0.018, 30);
gs.span  = linspace(0.020, 0.050, 30);
gs.beta  = deg2rad(linspace(5, 30, 30));
gs.zdot0 = linspace(-1.0, -5.0, 30);

% Coarser grid for the exhaustive 4-D optimum search
g4.chord = linspace(0.006, 0.018, 9);
g4.span  = linspace(0.020, 0.050, 9);
g4.beta  = deg2rad(linspace(5, 30, 9));
g4.zdot0 = linspace(-1.0, -5.0, 9);

%% ===== 3-D surfaces: choose any two variables for the xy-plane =====
% Each call plots the hop (zdot_exit) and the retained spin (omega_exit) over
% a variable pair, with the other two held at nominal.  The controller floor
% is drawn and the best feasible point (max hop with omega_exit >= floor) is
% marked.  Swap the first two arguments to view any other pair.
surf_pair('beta','zdot0', gs, nom, params);    % strongest energy levers
surf_pair('chord','span', gs, nom, params);    % geometry / area pair

%% ===== Global optimum across all four variables =====
optimize_hop(g4, params);

% =========================================================================
%  Helper functions
% =========================================================================

%% Run one case and return time series + exit values
function [time, omega_t, zdot_t, omega_exit, zdot_exit, t_exit] = run_case(chord, span, beta, zdot0, p)
    [time, ~, zdot_t, omega_t, ~, zdot_exit, omega_exit, t_exit] = simulate_water_skipping( ...
        chord, span, beta, p.N, p.rtip, p.m, p.I, p.rho, p.g, p.omega0, p.z0, zdot0, p.t, p.dt);
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
        [time, omega_t, zdot_t, omega_exit, zdot_exit, t_exit] = ...
            run_case(cse.chord, cse.span, cse.beta, cse.zdot0, params);

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
        case 'chord'; d = v*1e3;      lab = 'chord [mm]';
        case 'span';  d = v*1e3;      lab = 'span [mm]';
        case 'beta';  d = rad2deg(v); lab = '\beta [deg]';
        case 'zdot0'; d = v;          lab = 'z'' into water [m/s]';
        otherwise;    d = v;          lab = field;
    end
end

%% 3-D surfaces of hop and retained spin over a pair of variables
function surf_pair(fieldX, fieldY, gridv, nom, params)
    vx = gridv.(fieldX);  nX = numel(vx);
    vy = gridv.(fieldY);  nY = numel(vy);
    Zhop  = nan(nY, nX);   % zdot_exit  [m/s]
    Zspin = nan(nY, nX);   % omega_exit [deg/s]

    for r = 1:nY
        for c = 1:nX
            cse = nom;
            cse.(fieldX) = vx(c);
            cse.(fieldY) = vy(r);
            [~,~,~, we, ze, ~] = run_case(cse.chord, cse.span, cse.beta, cse.zdot0, params);
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
    Zhop_feas(Zspin < omin_deg) = NaN;
    [best, idx] = max(Zhop_feas(:));

    figure('Name', sprintf('Surfaces: %s vs %s', fieldX, fieldY));
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

    % --- Hop surface ---
    nexttile;
    surf(DX, DY, Zhop, 'EdgeColor','none', 'FaceAlpha',0.9); hold on; grid on; view(135,30);
    if ~isnan(best)
        plot3(DX(idx), DY(idx), best, 'rp', 'MarkerSize',16, 'MarkerFaceColor','r');
    end
    xlabel(labx); ylabel(laby); zlabel('z''_{exit}  (hop) [m/s]');
    title('Hop: exit vertical velocity'); colorbar;

    % --- Retained-spin surface with controller floor plane ---
    nexttile;
    surf(DX, DY, Zspin, 'EdgeColor','none', 'FaceAlpha',0.9); hold on; grid on; view(135,30);
    surf(DX, DY, omin_deg*ones(nY,nX), 'FaceColor',[0.85 0.2 0.2], 'FaceAlpha',0.25, 'EdgeColor','none');
    xlabel(labx); ylabel(laby); zlabel('\omega_{exit} [deg/s]');
    title(sprintf('Retained spin (floor = %.0f deg/s)', omin_deg)); colorbar;

    if ~isnan(best)
        fprintf('[surf %s vs %s] best feasible hop z''_exit = %.2f m/s at %s = %.3g, %s = %.3g\n', ...
            fieldX, fieldY, best, strtrim(labx), DX(idx), strtrim(laby), DY(idx));
    else
        fprintf('[surf %s vs %s] no feasible point in this slice\n', fieldX, fieldY);
    end
end

%% Exhaustive 4-D search: max hop subject to omega_exit >= controller floor
function optimize_hop(g, params)
    omin_deg = rad2deg(params.omega_min);
    ncases = numel(g.chord)*numel(g.span)*numel(g.beta)*numel(g.zdot0);
    fprintf('\n4-D optimum search over %d designs...\n', ncases);
    best = -inf;  bopt = [];
    for ic = 1:numel(g.chord)
      fprintf('  chord %d/%d\n', ic, numel(g.chord));
      for is = 1:numel(g.span)
        for ib = 1:numel(g.beta)
          for iz = 1:numel(g.zdot0)
            [~,~,~, we, ze, ~] = run_case(g.chord(ic), g.span(is), g.beta(ib), g.zdot0(iz), params);
            if ~isnan(we) && rad2deg(we) >= omin_deg && ze > best
                best = ze;
                bopt = [g.chord(ic), g.span(is), g.beta(ib), g.zdot0(iz), rad2deg(we)];
            end
          end
        end
      end
    end

    fprintf('\n===== Global optimum: max hop with omega_exit >= %.0f deg/s =====\n', omin_deg);
    if isempty(bopt)
        fprintf('  No feasible design found in the search grid.\n');
    else
        fprintf('  z''_exit (hop)  = %.2f m/s   (hop height ~ %.0f mm)\n', best, 1000*best^2/(2*params.g));
        fprintf('  chord          = %.1f mm\n', bopt(1)*1e3);
        fprintf('  span           = %.1f mm\n', bopt(2)*1e3);
        fprintf('  beta           = %.1f deg\n', rad2deg(bopt(3)));
        fprintf('  zdot0 (entry)  = %.2f m/s\n', bopt(4));
        fprintf('  omega_exit     = %.0f deg/s\n', bopt(5));
    end
end

%% Simulate one water contact (entry -> submerged -> exit)
function [time, z_history, zdot_history, omega_history, ...
    z_exit, zdot_exit, omega_exit, t_exit] = simulate_water_skipping( ...
    chord, span, beta, N, rtip, m, I, rho, g, omega0, z0, zdot0, t, dt)

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
        [T, Q] = compute_thrust_and_torque(chord, span, beta, N, rtip, rho, omega, z, zdot);

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
function [T_total, Q_total] = compute_thrust_and_torque(chord, span, beta, N, rtip, rho, omega, z, zdot)
    % z >= 0 : foil above water, no force
    if z >= 0
        T_total = 0;  Q_total = 0;  return;
    end

    h     = -z;                 % penetration depth
    rroot = rtip - span;        % root radius set directly by span

    if rroot < 0 || rroot >= rtip
        T_total = NaN;  Q_total = NaN;  return;     % invalid geometry
    end

    % Effective submerged chord grows as the inclined plate penetrates
    c_sub = min(h / sin(beta), chord);
    if c_sub <= 0
        T_total = 0;  Q_total = 0;  return;
    end

    % Blade-element integration along the span
    nr = 100;
    r  = linspace(rroot, rtip, nr);

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
