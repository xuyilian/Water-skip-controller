clear; clc; close all;

%% Known parameters
S = 52e-4;          % hydrofoil area of one foil [m^2]
N = 4;                  % number of hydrofoils
rtip = 5e-2;            % tip radius [m]
m = 70e-3;              % mass [kg]
rho = 1000;             % water density [kg/m^3]
g = 9.81;               % gravitational acceleration [m/s^2]

I = 7e-5;               % moment of inertia [kg*m^2], replace with measured value if available

omega0 = 66;            % initial angular velocity [rad/s]
zdot0 = -3.5;           % initial vertical velocity [m/s], upward positive
z0 = 0;                 % water surface position [m]

%% Simulation settings
dt = 1e-5;              % time step [s]
t_end = 0.12;           % maximum simulation time [s]
t = 0:dt:t_end;

%% Design parameter ranges
c_list = [0.15 0.2 0.35];      % design chord length [m]
beta_list_deg = [5 10 15 20 25 30];       % hydrofoil inclination angle [deg]

%% Plot all results
figure_z = figure;
hold on; grid on; box on;

figure_omega = figure;
hold on; grid on; box on;

legend_text = {};

fprintf('\nSimulation results:\n');
fprintf('---------------------------------------------------------------\n');

for ci = 1:length(c_list)
    for bi = 1:length(beta_list_deg)

        c = c_list(ci);
        beta = deg2rad(beta_list_deg(bi));

        [time, z_history, zdot_history, omega_history, ...
            z_exit, zdot_exit, omega_exit, t_exit] = simulate_water_skipping( ...
            c, beta, S, N, rtip, m, I, rho, g, omega0, z0, zdot0, t, dt);

        % Skip invalid geometry
        if any(isnan(z_history)) || isnan(z_exit)
            fprintf('c = %.0f mm, beta = %.0f deg: invalid geometry or no exit\n', ...
                c * 1000, beta_list_deg(bi));
            continue;
        end

        label_text = sprintf('c = %.0f mm, \\beta = %.0f deg', ...
            c * 1000, beta_list_deg(bi));
        legend_text{end+1} = label_text;

        % Plot z(t)
        figure(figure_z);
        plot(time, z_history * 1000, 'LineWidth', 1.2);
        plot(t_exit, z_exit * 1000, 'o', 'MarkerSize', 6, 'LineWidth', 1.2);

        % Plot omega(t)
        figure(figure_omega);
        plot(time, omega_history, 'LineWidth', 1.2);
        plot(t_exit, omega_exit, 'o', 'MarkerSize', 6, 'LineWidth', 1.2);

        fprintf(['c = %.0f mm, beta = %.0f deg: ', ...
                 'z_exit = %.6f mm, zdot_exit = %.3f m/s, ', ...
                 'omega_exit = %.3f rad/s, t_exit = %.5f s\n'], ...
            c * 1000, beta_list_deg(bi), ...
            z_exit * 1000, zdot_exit, omega_exit, t_exit);
    end
end

%% Format z plot
figure(figure_z);
xlabel('Time [s]');
ylabel('Vertical position z [mm]');
title('Vertical motion under different c and \beta');
yline(0, '--', 'Water surface');
legend(legend_text, 'Location', 'eastoutside');

%% Format omega plot
figure(figure_omega);
xlabel('Time [s]');
ylabel('Angular velocity \omega [rad/s]');
title('Angular velocity under different c and \beta');
legend(legend_text, 'Location', 'eastoutside');

%% Function: simulate one case
function [time, z_history, zdot_history, omega_history, ...
    z_exit, zdot_exit, omega_exit, t_exit] = simulate_water_skipping( ...
    c, beta, S, N, rtip, m, I, rho, g, omega0, z0, zdot0, t, dt)

    z = z0;
    zdot = zdot0;
    omega = omega0;

    z_history = zeros(size(t));
    zdot_history = zeros(size(t));
    omega_history = zeros(size(t));

    z_exit = NaN;
    zdot_exit = NaN;
    omega_exit = NaN;
    t_exit = NaN;

    has_submerged = false;

    for k = 1:length(t)

        z_history(k) = z;
        zdot_history(k) = zdot;
        omega_history(k) = omega;

        if z < 0
            has_submerged = true;
        end

        % Compute hydrodynamic thrust and resisting torque
        [T, Q] = compute_thrust_and_torque(c, beta, S, N, rtip, rho, omega, z, zdot);

        if isnan(T) || isnan(Q)
            time = t(1:k);
            z_history = NaN;
            zdot_history = NaN;
            omega_history = NaN;
            return;
        end

        % Upward-positive vertical dynamics:
        % m*zddot = T - mg
        zddot = T / m - g;

        % Rotational dynamics:
        % I*omegadot = -Q
        omegadot = -Q / I;

        % Store previous state
        z_prev = z;
        zdot_prev = zdot;
        omega_prev = omega;
        t_prev = t(k);

        % Semi-implicit Euler integration
        zdot = zdot + zddot * dt;
        z = z + zdot * dt;
        omega = omega + omegadot * dt;

        % Avoid negative angular velocity
        omega = max(omega, 0);

        % Check water exit:
        % z crosses from negative to zero/positive with upward velocity
        if has_submerged && z_prev < 0 && z >= 0 && zdot > 0

            % Linear interpolation ratio
            ratio = (0 - z_prev) / (z - z_prev);

            % Interpolated exit states
            t_exit = t_prev + ratio * dt;
            z_exit = 0;
            zdot_exit = zdot_prev + ratio * (zdot - zdot_prev);
            omega_exit = omega_prev + ratio * (omega - omega_prev);

            % Append exact exit point
            time = [t(1:k), t_exit];

            z_history(k+1) = z_exit;
            zdot_history(k+1) = zdot_exit;
            omega_history(k+1) = omega_exit;

            z_history = z_history(1:k+1);
            zdot_history = zdot_history(1:k+1);
            omega_history = omega_history(1:k+1);

            return;
        end
    end

    % If no exit happens within simulation time
    time = t;
    z_history = z_history(1:length(t));
    zdot_history = zdot_history(1:length(t));
    omega_history = omega_history(1:length(t));
end

%% Function: compute total vertical thrust and resisting torque
function [T_total, Q_total] = compute_thrust_and_torque(c, beta, S, N, rtip, rho, omega, z, zdot)

    % Coordinate definition:
    % z = 0: water surface
    % z < 0: hydrofoil is submerged
    % z > 0: hydrofoil is above water
    if z >= 0
        T_total = 0;
        Q_total = 0;
        return;
    end

    % Penetration depth
    h = -z;

    % Root radius determined by design chord length
    rroot = rtip - S / c;

    % Invalid geometry
    if rroot < 0 || rroot >= rtip
        T_total = NaN;
        Q_total = NaN;
        return;
    end

    % Effective submerged chord length
    c_sub = min(h / sin(beta), c);

    if c_sub <= 0
        T_total = 0;
        Q_total = 0;
        return;
    end

    % Numerical integration along radius
    nr = 300;
    r = linspace(rroot, rtip, nr);

    % Local tangential velocity
    vx = omega .* r;

    % Resultant velocity
    v = sqrt(vx.^2 + zdot.^2);

    % Inflow angle
    % zdot < 0 means moving downward
    gamma = atan2(-zdot, vx);

    % Angle of attack
    alpha = beta + gamma;

    % Flat-plate lift and drag coefficients
    CL = 2 .* sin(alpha) .* cos(alpha);
    CD = 2 .* sin(alpha).^2;

    % Differential vertical thrust integrand
    integrand_T = 0.5 .* rho .* c_sub .* v.^2 .* ...
        (CL .* cos(gamma) + CD .* sin(gamma));

    % Differential resisting torque integrand
    integrand_Q = 0.5 .* rho .* c_sub .* r .* v.^2 .* ...
        (CD .* cos(gamma) - CL .* sin(gamma));

    % Integrate over one hydrofoil
    T_foil = trapz(r, integrand_T);
    Q_foil = trapz(r, integrand_Q);

    % Total force and torque from N hydrofoils
    T_total = N * T_foil;
    Q_total = N * Q_foil;

    % Ensure resisting torque is non-negative
    Q_total = max(Q_total, 0);
end