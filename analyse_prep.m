% analyse_prep.m
% Time-base setup + field cleanup for analyse.m. Run this (or let analyse.m
% call it) after loading a DataExchange .mat. It builds t / dt / Fs, crops to
% the analysis window, and fills any missing expected signals with NaN so the
% plotting sections can run — including a single section on its own.
%
% Requires: Abs_time in the workspace (load a log first).

if ~exist('Abs_time', 'var')
    error(['analyse_prep: no ''Abs_time'' in the workspace. Load a log first, ' ...
        'e.g. load(''DataExchange/20260713_140141.mat'').']);
end

t = Abs_time(:);

% Analysis time window (seconds). Use [0, inf] for the full log.
t_start = 0;
t_end   = inf;
idx = (t >= t_start) & (t <= t_end);

% Crop every workspace vector whose length matches the time vector.
% Skip 't' itself so it isn't cropped twice (once in the loop, once below).
vars = whos;
for k = 1:numel(vars)
    name = vars(k).name;
    if strcmp(name, 't')
        continue
    end
    v = eval(name);
    if isnumeric(v) && isvector(v) && numel(v) == numel(t)
        eval([name ' = ' name '(idx);']);
    end
end
t = t(idx);

% Robustness to older logger versions: any expected per-sample signal that is
% absent from this log is filled with NaN (it shows as a blank gap in the
% plots) so a missing field can never abort a section.
expected = {'mocap_x_raw','mocap_y_raw','mocap_z_raw', ...
    'mocap_x_filt','mocap_y_filt','mocap_z_filt', ...
    'desired_x','desired_y','desired_z', ...
    'mocap_vx_filt','mocap_vy_filt','mocap_vz_filt', ...
    'mocap_roll_deg','mocap_pitch_deg','mocap_yaw_deg', ...
    'mocap_yawrate_deg','mocap_yawrate_deg_filt','yaw_deg', ...
    'cmd_roll','cmd_pitch','cmd_yaw','cmd_thrust', ...
    'R13','R23','R13_filt','R23_filt', ...
    'R13_d','R23_d','R13_d_filt','R23_d_filt', ...
    'pool_x','pool_y','autobi_desired_x','autobi_desired_y'};
missing_fields = {};
for k = 1:numel(expected)
    nm = expected{k};
    if ~exist(nm, 'var')
        eval([nm ' = nan(size(t));']);
        missing_fields{end+1} = nm; %#ok<AGROW>
    end
end
if ~isempty(missing_fields)
    warning('These expected signals are absent and were filled with NaN (blank in plots): %s', ...
        strjoin(missing_fields, ', '));
end

% Sample rate (used for the MATLAB-side filters in analyse.m).
dt = diff(t);
Fs = 1 / median(dt);
fprintf('Samples: %d, median dt: %.4f s, Fs ~ %.1f Hz\n', numel(t), median(dt), Fs);
