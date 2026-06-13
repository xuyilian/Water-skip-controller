%% ===== Fit C4, C5 consistent with GeoController.update_error() =====
% Assumes variables are already in workspace.

t = Abs_time(:);

% ---- choose measured states (use filtered if you have them) ----
% position
x = mocap_x(:);
y = mocap_y(:);

% velocity: prefer dp2_X_F_d / dp2_Y_F_d if you logged them
if exist('dp2_X_F_d','var') && exist('dp2_Y_F_d','var')
    xdot = double(dp2_X_F_d(:));
    ydot = double(dp2_Y_F_d(:));
else
    xdot = gradient(x, t);
    ydot = gradient(y, t);
end

% attitude tilt variables
r13  = double(R13(:));
r23  = double(R23(:));
r13d = double(R13_d(:));
r23d = double(R23_d(:));

% yaw rate (deg/s) from mocap processor
w_dps = double(mocap_yawrate_dps(:));

% controller outputs to fit
ux = double(U_X(:));
uy = double(U_Y(:));

% desired signals
xd = double(desired_x(:));
yd = double(desired_y(:));

% desired velocity: if you didn't log, approximate by gradient
if exist('desired_x_d','var') && exist('desired_y_d','var')
    xd_dot = double(desired_x_d(:));
    yd_dot = double(desired_y_d(:));
else
    xd_dot = gradient(xd, t);
    yd_dot = gradient(yd, t);
end

% ---- align length & clean ----
N0 = min([numel(t),numel(x),numel(y),numel(xdot),numel(ydot), ...
          numel(r13),numel(r23),numel(r13d),numel(r23d),numel(w_dps), ...
          numel(ux),numel(uy),numel(xd),numel(yd),numel(xd_dot),numel(yd_dot)]);
t=t(1:N0); x=x(1:N0); y=y(1:N0); xdot=xdot(1:N0); ydot=ydot(1:N0);
r13=r13(1:N0); r23=r23(1:N0); r13d=r13d(1:N0); r23d=r23d(1:N0);
w_dps=w_dps(1:N0); ux=ux(1:N0); uy=uy(1:N0);
xd=xd(1:N0); yd=yd(1:N0); xd_dot=xd_dot(1:N0); yd_dot=yd_dot(1:N0);

bad = any(~isfinite([t x y xdot ydot r13 r23 r13d r23d w_dps ux uy xd yd xd_dot yd_dot]),2);
t(bad)=[]; x(bad)=[]; y(bad)=[]; xdot(bad)=[]; ydot(bad)=[];
r13(bad)=[]; r23(bad)=[]; r13d(bad)=[]; r23d(bad)=[];
w_dps(bad)=[]; ux(bad)=[]; uy(bad)=[]; xd(bad)=[]; yd(bad)=[]; xd_dot(bad)=[]; yd_dot(bad)=[];

% remove duplicates in time
[t, ia] = unique(t,'stable');
x=x(ia); y=y(ia); xdot=xdot(ia); ydot=ydot(ia);
r13=r13(ia); r23=r23(ia); r13d=r13d(ia); r23d=r23d(ia);
w_dps=w_dps(ia); ux=ux(ia); uy=uy(ia); xd=xd(ia); yd=yd(ia); xd_dot=xd_dot(ia); yd_dot=yd_dot(ia);

N = numel(t);
fprintf('N=%d, dt~%.4g\n', N, median(diff(t)));

%% ===== Known controller parameters (YOU MUST FILL THESE) =====
% Use the exact values you used when running GeoController.Controller(...)
lambda_i_temp = 0.0;                 % set to lambda_i if integrator enabled
lambda0 = 0.0;                       % e.g., 45/100 if your code divides by 100
lambda1 = 0.0;                       % e.g., 30/100
lambda2 = 350.0;                     % from your init
lambda3 = 55.0;                      % from your init
C3 = 9.35;                           % use your fitted/assumed C3

% If in your Python you actually pass lambda0..3 already scaled, match that scaling here.
% (Some versions divide by 100 in the caller; ensure consistency!)

%% ===== Build features according to update_error() =====
% Errors in xy
ex = xd - x;
ey = yd - y;

% velocity errors
exd = xd_dot - xdot;
eyd = yd_dot - ydot;

% xi and xi_dot in the controller's form
xi1    = -r23;      % = [-R23; R13]
xi2    =  r13;
xi1dot = -r23d;
xi2dot =  r13d;

% a_temp2 * xi = [-C3*R13; -C3*R23]
a2_x = -C3 * r13;   % first component
a2_y = -C3 * r23;   % second component

% a_temp2 * xi_dot = [-C3*R13_d; -C3*R23_d]
a2d_x = -C3 * r13d;
a2d_y = -C3 * r23d;

% S = (b1+b2+b3+b4) without C4/C5 (vector components)
S1 = lambda_i_temp*0 + lambda0*ex + lambda1*exd + lambda2*a2_x + lambda3*a2d_x;
S2 = lambda_i_temp*0 + lambda0*ey + lambda1*eyd + lambda2*a2_y + lambda3*a2d_y;

% Controller output formula derived from your code:
% Ux = (C4/C3)*S1 + C5*w*R23_d
% Uy = (C4/C3)*S2 - C5*w*R13_d
% Note: w should be consistent with how dp1.angleY_d is used in Python (deg/s).
% If your Python expects rad/s, convert: w = w_dps*pi/180

w = w_dps;  % keep as deg/s to match your current code usage

% Stack linear regression: [Ux; Uy] = [ (S1/C3)  (w*R23_d);
%                                      (S2/C3)  (-w*R13_d) ] * [C4; C5]
A = [ S1./C3,  w.*r23d;
      S2./C3, -w.*r13d ];
b = [ ux;
      uy ];

%% ===== Optional: remove saturated samples (recommended) =====
% If you used circular saturation at 300, drop near-saturation points.
sat = sqrt(ux.^2 + uy.^2);
keep = sat < 0.95*300;     % adjust threshold
A = A([keep; keep], :);
b = b([keep; keep], :);

%% ===== Solve =====
theta = A \ b;
C4_hat = theta(1);
C5_hat = theta(2);

fprintf('Fitted (controller-consistent): C4=%.6g, C5=%.6g\n', C4_hat, C5_hat);

%% ===== Evaluate & plot =====
ux_pred = (C4_hat/C3).*S1 + C5_hat*w.*r23d;
uy_pred = (C4_hat/C3).*S2 - C5_hat*w.*r13d;

rmse_u = sqrt(mean(([ux;uy] - [ux_pred;uy_pred]).^2));
fprintf('RMSE(U)=%.6g\n', rmse_u);

figure('Name','U_X/U_Y vs pred (controller-consistent)'); tiledlayout(2,1);
nexttile; plot(t, ux, 'LineWidth',1.0); hold on; plot(t, ux_pred,'--','LineWidth',1.0); grid on;
legend('U_X','pred'); ylabel('U_X');

nexttile; plot(t, uy, 'LineWidth',1.0); hold on; plot(t, uy_pred,'--','LineWidth',1.0); grid on;
legend('U_Y','pred'); ylabel('U_Y'); xlabel('t');