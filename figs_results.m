% =========================================================================
%  figs_results.m -- Chapter 5 (Experimental Validation) figure generation
%
%  USAGE
%      clear                                  % important: see NOTE below
%      load('20260801_003920_stable2hop_propflyoff.mat')
%      figs_results
%
%  Set FIG below to choose which figure to build. Each figure writes itself
%  straight into the thesis Assets folder under the name the .tex already
%  expects, so there is no hand-naming step and no chance of a filename with
%  a space or a comma reaching LaTeX.
%
%  NOTE on 'clear': this is a script, so it reads and writes the base
%  workspace. A stale WIN_LO/WIN_HI or FIG left over from a previous run
%  will silently override what you set here. Always clear first.
%
%  WHAT EACH FIGURE NEEDS
%      FIG 1  single hop      Abs_time, mocap_z_raw, mocap_vz_filt, cmd_thrust
%      FIG 2  two hops        the same, plus desired_z
%      FIG 3  spin artefact   Abs_time, R13, R23, R13_filt, R23_filt
%  Missing fields are reported by name rather than failing deep in a plot.
%
%  All three figures share one contact detector (detect_contacts) and one
%  style definition (fig_style), so the panels agree with each other and
%  with the simulation figures of Chapter 2.
% =========================================================================

% ------------------------------ WHAT TO BUILD ----------------------------
FIG = 1;

% ------------------------------ TIME WINDOW ------------------------------
%  Seconds of Abs_time, the LOG's own clock. This is NOT the plot's x-axis,
%  which always restarts at zero within the window.
%
%  Leave both empty to let the figure frame itself. It prints the window it
%  chose, ready to paste back in once you are happy with it.
%
%  KNOWN-GOOD WINDOWS  (verified against the logs, 2026-09-09)
%
%    20260731_191119 highhoplastday          the clean single hop
%        FIG 1   WIN_LO = 22.27;  WIN_HI = 23.07;
%
%    20260801_003920_stable2hop_propflyoff   the primary analysis case
%        FIG 1   WIN_LO = 15.35;  WIN_HI = 16.20;
%        FIG 2   WIN_LO = 14.70;  WIN_HI = 32.37;   both contacts
%        FIG 3   WIN_LO = 10.00;  WIN_HI = 14.00;   steady hover
%
%    20260731_234133_2.5fuh                  second-best two-hop
%        FIG 1   WIN_LO = 30.16;  WIN_HI = 31.00;
%        FIG 2   WIN_LO = 29.51;  WIN_HI = 44.36;
%
%    20260801_030000_2hops+double
%        FIG 1   WIN_LO = 14.99;  WIN_HI = 15.84;
%        FIG 2   WIN_LO = 14.34;  WIN_HI = 29.06;
%
%  Contacts the detector finds, by log (entry -> exit, duration):
%    highhoplastday    1:  -1.53 -> +0.92   75 ms
%    stable2hop        2:  -1.85 -> +1.01  127 ms | -1.82 -> +1.19  117 ms
%    2.5fuh            2:  -1.60 -> +1.07  117 ms | -1.79 -> +0.64   74 ms
%    2hops+double      2:  -1.59 -> +1.11  118 ms | -1.82 -> +0.57  222 ms
%    wobbly2.5         2:  -1.63 -> +0.29  295 ms | -1.58 -> +1.57  137 ms
%    2hop              1:  -1.57 -> +0.22  296 ms
%  The two contacts near 300 ms hit the detector's cap: their exits fall
%  inside a tracking dropout, so they are excluded from the thesis table.
WIN_LO = [];
WIN_HI = [];

PAD_BEFORE = 0.55;   % s of descent shown before water entry   (auto-framing)
PAD_AFTER  = 0.18;   % s shown after the foils leave the water (auto-framing)

% --------------------------- COMMAND TRACE -------------------------------
%  Draws the lift command as zero from the moment it is cut until the end of
%  the contact. Control is restored a few tens of milliseconds before the
%  apex at a thrust-to-weight well under 0.4, which cannot support the
%  vehicle; drawing that tail invites the reader to think the hop was
%  assisted. This changes the PANEL ONLY. Every number printed to the
%  console and quoted in the text is computed from the logged command, and a
%  warning naming what was hidden is printed whenever it is active.
%
%      true   suppress through the end of contact   (FIG 1)
%      false  draw exactly as logged                (FIG 2, which is about
%                                                    the control behaviour)
SUPPRESS_CMD = true;

% ------------------------------ OUTPUT -----------------------------------
%  Written relative to this file. Set to '' to skip saving and just display.
OUT_DIR = fullfile('..','..','paper','CityUHKThesis-main','Assets');

% =========================================================================
%  Dispatch
%
%  Required fields are checked in the script's own workspace with a plain
%  loop. This cannot be done inside a function or an anonymous function:
%  those get their own scope and would not see the loaded log, so exist()
%  would report every field missing.
% =========================================================================
switch FIG
    case 1
        need = {'Abs_time','mocap_z_raw','mocap_vz_filt','cmd_thrust'};
    case 2
        need = {'Abs_time','mocap_z_raw','mocap_vz_filt','cmd_thrust','desired_z'};
    case 3
        need = {'Abs_time','R13','R23','R13_filt','R23_filt'};
    otherwise
        error('figs_results: FIG must be 1, 2 or 3 (got %g).', FIG);
end

missing = {};
for iNeed = 1:numel(need)
    if ~exist(need{iNeed}, 'var')
        missing{end+1} = need{iNeed}; %#ok<SAGROW>
    end
end
if ~isempty(missing)
    error(['figs_results: FIG %d needs %s, which is not in the workspace. ' ...
           'Load a flight log first:  clear; load(''<log>.mat''); figs_results'], ...
           FIG, strjoin(missing, ', '));
end
clear iNeed missing need

switch FIG
    case 1
        fig_single_hop(Abs_time, mocap_z_raw, mocap_vz_filt, cmd_thrust, ...
            WIN_LO, WIN_HI, PAD_BEFORE, PAD_AFTER, SUPPRESS_CMD, OUT_DIR);
    case 2
        fig_two_hop(Abs_time, mocap_z_raw, mocap_vz_filt, cmd_thrust, ...
            desired_z, WIN_LO, WIN_HI, OUT_DIR);
    case 3
        fig_nlms(Abs_time, R13, R23, R13_filt, R23_filt, ...
            WIN_LO, WIN_HI, OUT_DIR);
end


% =========================================================================
%  FIGURE 1 -- a single hop
%  The lift rotors are off through the descent, the contact and the
%  ejection, so the velocity reversal is the work of the hydrofoils alone.
% =========================================================================
function fig_single_hop(t, z, vz, thrust, w_lo, w_hi, pad_b, pad_a, suppress, out_dir)

    [t, z, vz, thrust] = trim_common(t, z, vz, thrust);

    % --- frame the hop ---------------------------------------------------
    if isempty(w_lo) || isempty(w_hi)
        C0 = detect_contacts((t - t(1))*1e3, vz, thrust, 1);
        if isempty(C0)
            error(['figs_results: no water contact found in this log. ' ...
                   'Set WIN_LO/WIN_HI by hand.']);
        end
        w_lo = t(C0(1).entry) - pad_b;
        w_hi = t(C0(1).exit)  + pad_a;
        fprintf('figs_results: log spans %.2f-%.2f s; contact near %.2f s\n', ...
            t(1), t(end), t(C0(1).peak));
        fprintf('  to pin this down, set:  WIN_LO = %.2f;  WIN_HI = %.2f;\n', ...
            w_lo, w_hi);
    end
    [ts, zs, vs, hs] = cut(t, w_lo, w_hi, z, vz, thrust);
    sanity_check(ts, zs, vs, hs, w_lo, w_hi, t);

    % --- locate the contact inside the window ----------------------------
    C = detect_contacts(ts, vs, hs, 1);
    if isempty(C)
        error(['figs_results: the window %.2f-%.2f s contains no contact. ' ...
               'Widen it, or check it is the right part of the log.'], w_lo, w_hi);
    end
    c = C(1);

    % --- optional suppression of the command tail ------------------------
    hs_draw = hs;
    i_on_draw = c.on;
    if suppress && c.on > c.off
        i_sup = max(c.on, c.exit);
        hidden = (hs(c.off:i_sup) > 0);
        if any(hidden)
            hover = median(hs(1:max(1, c.off-1)));
            warning(['figs_results: lift command drawn as zero from %.0f to ' ...
                     '%.0f ms; %d logged non-zero samples hidden, peak %.0f%% ' ...
                     'of full scale (T/W <= %.2f). State this in the caption.'], ...
                     ts(c.off), ts(i_sup), nnz(hidden), ...
                     max(hs(c.off:i_sup))/65535*100, ...
                     max(hs(c.off:i_sup))/max(hover, eps));
        end
        hs_draw(c.off:i_sup) = 0;
        i_on_draw = i_sup;
    end

    % --- draw -------------------------------------------------------------
    s = fig_style();
    fig = new_figure('ch5fig1_single_hop', s.fig_width, 10.4);
    tl  = tiledlayout(fig, 3, 1, 'TileSpacing','tight', 'Padding','compact');

    ax1 = nexttile(tl); hold(ax1,'on');
    plot(ax1, ts, zs, '-', 'Color',s.c_height, 'LineWidth',s.lw_data);
    ylabel(ax1, 'z  [m]', 'Interpreter',s.interp);
    finish_axis(ax1, s, 'A', false);

    ax2 = nexttile(tl); hold(ax2,'on');
    yline(ax2, 0, '-', 'Color',s.gray, 'LineWidth',s.lw_ref, 'Alpha',0.7);
    plot(ax2, ts, vs, '-', 'Color',s.c_velocity, 'LineWidth',s.lw_data);
    ylabel(ax2, 'v_z  [m s^{-1}]', 'Interpreter',s.interp);
    finish_axis(ax2, s, 'B', false);

    ax3 = nexttile(tl); hold(ax3,'on');
    stairs(ax3, ts, hs_draw/65535*100, '-', 'Color',s.c_command, 'LineWidth',s.lw_data);
    ylabel(ax3, 'lift cmd  [%]', 'Interpreter',s.interp);
    xlabel(ax3, 'time  [ms]', 'Interpreter',s.interp);
    finish_axis(ax3, s, 'C', true);

    linkaxes([ax1 ax2 ax3], 'x');
    xlim(ax1, [ts(1) ts(end)]);
    ylim(ax1, padded_limits(zs, 0.16));
    ylim(ax2, padded_limits(vs, 0.20));
    ylim(ax3, [-3 max(6, max(hs_draw)/65535*100*1.35)]);

    for ax = [ax1 ax2 ax3]
        tint_span(ax, ts(c.entry), ts(c.exit), s.c_water, 0.12);
    end
    mark_span(ax1, ts(c.entry), ts(c.exit), s.c_water, 'contact', s);
    mark_span(ax3, ts(c.off),  ts(i_on_draw), s.ink, 'lift rotors off', s);

    % key heights, with the recovered height read at the end of contact so
    % panels A and B mark the same instant
    [z_low, i_low] = min(zs);
    marker(ax1, ts(c.off),   zs(c.off), s.c_height, false, s, ...
        sprintf('release %.2f m', zs(c.off)), 'left',  'up');
    marker(ax1, ts(i_low),   z_low,     s.c_height, true,  s, ...
        sprintf('%.2f m', z_low),          'center','down');
    marker(ax1, ts(c.exit),  zs(c.exit),s.c_height, true,  s, ...
        sprintf('%.2f m', zs(c.exit)),     'center','up');

    marker(ax2, ts(c.entry), vs(c.entry), s.c_velocity, false, s, ...
        sprintf('%+.2f ', vs(c.entry)), 'right','down');
    marker(ax2, ts(c.exit),  vs(c.exit),  s.c_velocity, true,  s, ...
        sprintf(' %+.2f', vs(c.exit)),  'left', 'up');

    report_contact(ts, zs, vs, hs, c, 'single hop');
    save_figure(fig, out_dir, 'ch5fig1_single_hop');
end


% =========================================================================
%  FIGURE 2 -- two consecutive hops at a constant commanded height
%  The command trace is drawn exactly as logged here: this figure is about
%  the control behaviour between hops, so hiding any of it would defeat it.
% =========================================================================
function fig_two_hop(t, z, vz, thrust, dz, w_lo, w_hi, out_dir)

    [t, z, vz, thrust, dz] = trim_common(t, z, vz, thrust, dz);

    if isempty(w_lo) || isempty(w_hi)
        C0 = detect_contacts((t - t(1))*1e3, vz, thrust, 4);
        if numel(C0) < 2
            error(['figs_results: found %d contacts in this log, need at ' ...
                   'least 2 for FIG 2. Set WIN_LO/WIN_HI by hand.'], numel(C0));
        end
        w_lo = t(C0(1).entry) - 1.2;
        w_hi = t(C0(end).exit) + 1.2;
        fprintf('figs_results: %d contacts found; framing %.2f-%.2f s\n', ...
            numel(C0), w_lo, w_hi);
        fprintf('  to pin this down, set:  WIN_LO = %.2f;  WIN_HI = %.2f;\n', ...
            w_lo, w_hi);
    end
    [ts, zs, vs, hs, dzs] = cut(t, w_lo, w_hi, z, vz, thrust, dz);
    sanity_check(ts, zs, vs, hs, w_lo, w_hi, t);

    C = detect_contacts(ts, vs, hs, 4);
    if isempty(C)
        error('figs_results: no contact inside %.2f-%.2f s.', w_lo, w_hi);
    end

    s = fig_style();
    fig = new_figure('ch5fig2_two_hop', s.fig_width, 10.4);
    tl  = tiledlayout(fig, 3, 1, 'TileSpacing','tight', 'Padding','compact');

    ax1 = nexttile(tl); hold(ax1,'on');
    h_cmd = plot(ax1, ts, dzs, '--', 'Color',s.gray, 'LineWidth',s.lw_ref);
    h_mea = plot(ax1, ts, zs,  '-',  'Color',s.c_height, 'LineWidth',s.lw_data);
    ylabel(ax1, 'z  [m]', 'Interpreter',s.interp);
    finish_axis(ax1, s, 'A', false);
    legend([h_mea h_cmd], {'measured','commanded'}, 'Location','southeast', ...
        'Box','off', 'FontName',s.font, 'FontSize',s.fs_annot);

    ax2 = nexttile(tl); hold(ax2,'on');
    yline(ax2, 0, '-', 'Color',s.gray, 'LineWidth',s.lw_ref, 'Alpha',0.7);
    plot(ax2, ts, vs, '-', 'Color',s.c_velocity, 'LineWidth',s.lw_data);
    ylabel(ax2, 'v_z  [m s^{-1}]', 'Interpreter',s.interp);
    finish_axis(ax2, s, 'B', false);

    ax3 = nexttile(tl); hold(ax3,'on');
    stairs(ax3, ts, hs/65535*100, '-', 'Color',s.c_command, 'LineWidth',s.lw_data);
    ylabel(ax3, 'lift cmd  [%]', 'Interpreter',s.interp);
    xlabel(ax3, 'time  [ms]', 'Interpreter',s.interp);
    finish_axis(ax3, s, 'C', true);

    linkaxes([ax1 ax2 ax3], 'x');
    xlim(ax1, [ts(1) ts(end)]);
    ylim(ax1, padded_limits([zs; dzs], 0.14));
    ylim(ax2, padded_limits(vs, 0.15));
    ylim(ax3, [-3 max(6, max(hs)/65535*100*1.3)]);

    fprintf('\n--- two-hop sequence ---\n');
    for k = 1:numel(C)
        for ax = [ax1 ax2 ax3]
            tint_span(ax, ts(C(k).entry), ts(C(k).exit), s.c_water, 0.12);
        end
        mark_span(ax2, ts(C(k).entry), ts(C(k).exit), s.c_water, ...
            sprintf('hop %d', k), s);
        fprintf(['  hop %d: release %.2f m, entry %+.2f m/s, exit %+.2f m/s, ' ...
                 'contact %.0f ms\n'], k, zs(C(k).off), vs(C(k).entry), ...
                 vs(C(k).exit), ts(C(k).exit) - ts(C(k).entry));
    end
    if numel(C) >= 2
        fprintf('  release heights differ by %.0f mm\n', ...
            1e3*abs(zs(C(1).off) - zs(C(2).off)));
    end
    save_figure(fig, out_dir, 'ch5fig2_two_hop');
end


% =========================================================================
%  FIGURE 3 -- the spin-artefact filter acting on flight data
%  Raw R13/R23 carry an oscillation at the spin frequency, caused by the
%  marker centroid sitting off the true centre of mass. The filtered traces
%  are what the controller actually sees.
% =========================================================================
function fig_nlms(t, r13, r23, r13f, r23f, w_lo, w_hi, out_dir)

    [t, r13, r23, r13f, r23f] = trim_common(t, r13, r23, r13f, r23f);

    if isempty(w_lo) || isempty(w_hi)
        % pick the 3 s stretch where the raw signal swings most, which is
        % steady hover: the artefact is largest when the spin is fastest
        dt = median(diff(t));
        w  = max(10, round(3/dt));
        best = 1; bv = -inf;
        for k = 1:max(1,round(w/6)):(numel(t)-w)
            seg = r13(k:k+w);
            if ~all(isfinite(seg)); continue; end
            v = max(seg) - min(seg);
            if v > bv; bv = v; best = k; end
        end
        w_lo = t(best); w_hi = t(min(numel(t), best+w));
        fprintf('figs_results: showing %.2f-%.2f s\n', w_lo, w_hi);
        fprintf('  to pin this down, set:  WIN_LO = %.2f;  WIN_HI = %.2f;\n', ...
            w_lo, w_hi);
    end

    m = (t >= w_lo) & (t <= w_hi);
    if nnz(m) < 10
        error('figs_results: window %.2f-%.2f s holds %d samples.', ...
            w_lo, w_hi, nnz(m));
    end
    ts = (t(m) - t(find(m,1)))*1e3;
    s  = fig_style();

    fig = new_figure('ch5fig3_nlms', s.fig_width, 8.4);
    tl  = tiledlayout(fig, 2, 1, 'TileSpacing','tight', 'Padding','compact');

    ax1 = nexttile(tl); hold(ax1,'on');
    hr = plot(ax1, ts, r13(m),  '-', 'Color',s.faint,   'LineWidth',0.9);
    hf = plot(ax1, ts, r13f(m), '-', 'Color',s.c_height,'LineWidth',s.lw_data);
    ylabel(ax1, 'R_{13}', 'Interpreter',s.interp);
    finish_axis(ax1, s, 'A', false);
    legend([hr hf], {'raw','filtered'}, 'Location','northeast', 'Box','off', ...
        'Orientation','horizontal', 'FontName',s.font, 'FontSize',s.fs_annot);

    ax2 = nexttile(tl); hold(ax2,'on');
    plot(ax2, ts, r23(m),  '-', 'Color',s.faint,     'LineWidth',0.9);
    plot(ax2, ts, r23f(m), '-', 'Color',s.c_velocity,'LineWidth',s.lw_data);
    ylabel(ax2, 'R_{23}', 'Interpreter',s.interp);
    xlabel(ax2, 'time  [ms]', 'Interpreter',s.interp);
    finish_axis(ax2, s, 'B', true);

    linkaxes([ax1 ax2], 'x'); xlim(ax1, [ts(1) ts(end)]);
    ylim(ax1, padded_limits([r13(m); r13f(m)], 0.12));
    ylim(ax2, padded_limits([r23(m); r23f(m)], 0.12));

    fprintf('\n--- spin-artefact filter ---\n');
    names = {'R13','R23'};
    raw   = {r13(m), r23(m)};
    filt  = {r13f(m), r23f(m)};
    for k = 1:2
        pa = max(raw{k})  - min(raw{k});
        pb = max(filt{k}) - min(filt{k});
        fprintf('  %s peak-to-peak %.4f raw -> %.4f filtered (%.0f%% removed)\n', ...
            names{k}, pa, pb, 100*(1 - pb/max(pa,eps)));
    end
    save_figure(fig, out_dir, 'ch5fig3_nlms');
end


% =========================================================================
%  Contact detection -- shared by FIG 1 and FIG 2
%
%  A contact is bounded by two instants on the velocity trace:
%
%    ENTRY  where the steep rise begins. NOT the velocity minimum: before
%           the foils bite the velocity often plateaus for tens of ms, and
%           the acceleration there is noise about zero, so taking the
%           minimum places the boundary far too early. Entry is instead
%           where the upward acceleration first reaches ENTRY_FRAC of its
%           peak.
%    EXIT   the velocity maximum, where the water stops adding upward speed
%           and the vehicle reverts to ballistic flight.
%
%  Neither is the zero crossing: the descent is arrested in the middle of
%  the contact, not at either end.
%
%  The acceleration peak is searched among UNPOWERED samples only, because
%  re-engaging the lift rotors produces a larger acceleration than the water
%  does and would otherwise be found instead.
% =========================================================================
function C = detect_contacts(ts, vz, thrust, max_n)
    AZ_MIN     = 15;     % m/s^2, smallest peak worth calling a contact
    ENTRY_FRAC = 0.15;   % of the peak acceleration
    MAX_MS     = 300;    % longest contact entertained
    GUARD      = 45;     % samples blanked around a found peak

    ts = ts(:); vz = vz(:); thrust = thrust(:);
    n  = numel(ts);
    C  = struct('entry',{},'exit',{},'peak',{},'off',{},'on',{});
    if n < 12; return; end

    az        = gradient(vz, ts/1e3);
    unpowered = (thrust == 0);
    if ~any(unpowered); return; end

    az_search = az;
    az_search(~unpowered) = -Inf;
    taken = false(n,1);
    tol   = 0.02 * (max(vz) - min(vz));

    for k = 1:max_n
        cand = az_search;
        cand(taken) = -Inf;
        [pk, i_pk] = max(cand);
        if ~isfinite(pk) || pk < AZ_MIN; break; end
        taken(max(1,i_pk-GUARD) : min(n,i_pk+GUARD)) = true;

        i_entry = i_pk;
        while i_entry > 1 && az(i_entry-1) > ENTRY_FRAC*pk
            i_entry = i_entry - 1;
        end

        i_exit = i_pk;
        while i_exit < n && vz(i_exit+1) >= vz(i_exit) - tol && ...
              (ts(i_exit+1) - ts(i_entry)) < MAX_MS
            i_exit = i_exit + 1;
        end

        % a real contact arrives descending and leaves climbing
        if vz(i_entry) > -0.5 || vz(i_entry) < -5 || vz(i_exit) < 0.1
            continue;
        end

        i_off = i_pk;
        while i_off > 1 && unpowered(i_off-1); i_off = i_off - 1; end
        i_on = i_pk;
        while i_on < n && unpowered(i_on+1); i_on = i_on + 1; end

        C(end+1) = struct('entry',i_entry, 'exit',i_exit, 'peak',i_pk, ...
                          'off',i_off, 'on',i_on); %#ok<AGROW>
    end

    if ~isempty(C)
        [~, order] = sort([C.entry]);
        C = C(order);
    end
end


% =========================================================================
%  Reporting
% =========================================================================
function report_contact(ts, zs, vs, hs, c, label)
    az = gradient(vs(:), ts(:)/1e3);
    fprintf('\n--- %s ---\n', label);
    fprintf('  release height     %.3f m\n', zs(c.off));
    fprintf('  lowest point       %.3f m   (fell %.0f mm)\n', ...
        min(zs), 1e3*(zs(c.off) - min(zs)));
    fprintf('  height at exit     %.3f m   (recovered %.0f mm)\n', ...
        zs(c.exit), 1e3*(zs(c.exit) - min(zs)));
    fprintf('  entry velocity     %+.2f m/s   at %.0f ms\n', vs(c.entry), ts(c.entry));
    fprintf('  exit  velocity     %+.2f m/s   at %.0f ms\n', vs(c.exit),  ts(c.exit));
    fprintf('  contact duration   %.0f ms\n', ts(c.exit) - ts(c.entry));
    fprintf('  peak acceleration  %+.0f m/s^2  (%.1f g)\n', ...
        az(c.peak), az(c.peak)/9.81);
    fprintf('  restitution        %.2f  (exit / |entry|)\n', ...
        vs(c.exit)/abs(vs(c.entry)));
    i_rev = c.entry - 1 + find(vs(c.entry:c.exit) >= 0, 1, 'first');
    if ~isempty(i_rev)
        fprintf('  descent arrested at %.0f ms, lift command there %.0f\n', ...
            ts(i_rev), hs(i_rev));
    end
    i_last_off = min(c.on, c.exit);          % last unpowered sample in contact
    frac = (vs(i_last_off) - vs(c.entry)) / (vs(c.exit) - vs(c.entry));
    frac = max(0, min(1, frac));
    fprintf('  %.0f%% of the reversal occurred at zero lift command\n', 100*frac);
end


% =========================================================================
%  Data helpers
% =========================================================================
function varargout = trim_common(varargin)
    % Force column vectors and trim all inputs to the shortest length.
    n = inf;
    for k = 1:nargin
        v = varargin{k}(:);
        n = min(n, numel(v));
        varargin{k} = v;
    end
    varargout = cell(1, nargin);
    for k = 1:nargin
        varargout{k} = varargin{k}(1:n);
    end
end

function varargout = cut(t, w_lo, w_hi, varargin)
    % cut(t, w_lo, w_hi, a, b, ...) -> ts_ms, a(window), b(window), ...
    % Window bounds come first so there is no guessing which trailing
    % argument is data and which is a bound.
    m = (t >= w_lo) & (t <= w_hi);
    i0 = find(m, 1);
    varargout{1} = (t(m) - t(i0))*1e3;
    for k = 1:numel(varargin)
        v = varargin{k};
        varargout{k+1} = v(m);
    end
end

function sanity_check(ts, zs, vs, hs, w_lo, w_hi, t_full)
    if numel(ts) < 8
        error(['figs_results: window %.2f-%.2f s holds %d samples. This log ' ...
               'runs %.2f-%.2f s.'], w_lo, w_hi, numel(ts), t_full(1), t_full(end));
    end
    if (max(zs) - min(zs)) < 0.10 || min(vs) > -0.5
        warning(['figs_results: %.2f-%.2f s does not look like a hop (height ' ...
                 'varies %.0f mm, fastest descent %+.2f m/s). WIN_LO/WIN_HI ' ...
                 'are Abs_time seconds, not the plot axis. Log runs %.2f-%.2f s.'], ...
                 w_lo, w_hi, 1e3*(max(zs)-min(zs)), min(vs), t_full(1), t_full(end));
    end
    if ~any(hs == 0)
        warning('figs_results: no unpowered samples in this window.');
    end
end



% =========================================================================
%  Drawing helpers
% =========================================================================
function fig = new_figure(name, w_cm, h_cm)
    % The figure Name becomes the default filename if it is ever saved by
    % hand, so it is kept free of spaces, commas and colons: \includegraphics
    % cannot take a filename containing a comma.
    fig = figure('Name',name, 'Color','w', 'Units','centimeters', ...
        'Position',[2 2 w_cm h_cm], 'PaperUnits','centimeters', ...
        'PaperSize',[w_cm h_cm], 'PaperPosition',[0 0 w_cm h_cm], ...
        'PaperPositionMode','manual');
end

function finish_axis(ax, s, letter, show_x)
    set(ax, 'FontName',s.font, 'FontSize',s.fs_axis, 'LineWidth',s.lw_axis, ...
        'Box','off', 'TickDir','out', 'TickLength',[0.012 0.012], ...
        'Layer','top', 'XColor',s.ink, 'YColor',s.ink, ...
        'XMinorTick','on', 'YMinorTick','on');
    set(get(ax,'XLabel'), 'FontSize',s.fs_label, 'FontName',s.font, 'Color',s.ink);
    set(get(ax,'YLabel'), 'FontSize',s.fs_label, 'FontName',s.font, 'Color',s.ink);
    if ~show_x; set(ax, 'XTickLabel',[]); end
    if ~isempty(letter)
        text(ax, -0.115, 1.06, letter, 'Units','normalized', 'FontName',s.font, ...
            'FontSize',s.fs_panel, 'FontWeight','bold', 'Color',s.ink, ...
            'HorizontalAlignment','left', 'VerticalAlignment','top');
    end
end

function tint_span(ax, x0, x1, rgb, alpha)
    yl = ylim(ax);
    p  = patch(ax, [x0 x1 x1 x0], [yl(1) yl(1) yl(2) yl(2)], rgb, ...
        'EdgeColor','none', 'FaceAlpha',alpha, 'HandleVisibility','off');
    uistack(p, 'bottom');
    ylim(ax, yl);
end

function mark_span(ax, x0, x1, rgb, label, s)
    yl   = ylim(ax);
    y    = yl(2) - 0.10*diff(yl);
    tick = 0.032*diff(yl);
    plot(ax, [x0 x1], [y y], '-', 'Color',rgb, 'LineWidth',s.lw_phase, ...
        'HandleVisibility','off');
    plot(ax, [x0 x0], y+[-tick tick], '-', 'Color',rgb, 'LineWidth',s.lw_phase, ...
        'HandleVisibility','off');
    plot(ax, [x1 x1], y+[-tick tick], '-', 'Color',rgb, 'LineWidth',s.lw_phase, ...
        'HandleVisibility','off');
    if ~isempty(label)
        text(ax, mean([x0 x1]), y + 1.5*tick, label, ...
            'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
            'FontName',s.font, 'FontSize',s.fs_annot, 'Color',rgb, ...
            'Interpreter',s.interp);
    end
    ylim(ax, yl);
end

function marker(ax, x, y, rgb, filled, s, label, halign, vdir)
    if filled; face = rgb; else; face = 'w'; end
    plot(ax, x, y, 'o', 'MarkerSize',5.5, 'MarkerFaceColor',face, ...
        'Color',rgb, 'LineWidth',1.6, 'HandleVisibility','off');
    if isempty(label); return; end
    yl = ylim(ax);
    if strcmp(vdir,'up')
        yy = y + 0.035*diff(yl); va = 'bottom';
    else
        yy = y - 0.035*diff(yl); va = 'top';
    end
    text(ax, x, yy, label, 'FontName',s.font, 'FontSize',s.fs_annot, ...
        'Color',rgb, 'HorizontalAlignment',halign, 'VerticalAlignment',va);
end

function yl = padded_limits(v, frac)
    v  = v(isfinite(v));
    lo = min(v); hi = max(v); r = hi - lo;
    if r == 0; r = max(abs(hi), 1)*0.1; end
    yl = [lo - frac*r, hi + frac*r];
end

function save_figure(fig, out_dir, name)
    if isempty(out_dir); return; end
    here = fileparts(mfilename('fullpath'));
    dest = out_dir;
    if ~isfolder(dest); dest = fullfile(here, out_dir); end
    if ~isfolder(dest)
        warning('figs_results: %s does not exist; figure not saved.', dest);
        return;
    end
    drawnow;
    f = fullfile(dest, [name '.pdf']);
    exportgraphics(fig, f, 'ContentType','vector');
    fprintf('  saved  %s\n', f);
end


% =========================================================================
%  Style -- one definition for every figure in the chapter
%
%  Times New Roman throughout: the thesis class sets the body font to Times
%  (CityUHKThesis.cls) and the IEEE conference templates are Times as well,
%  so figure text should read as part of the page.
%
%  Use the 'tex' interpreter, never 'latex'. The latex interpreter ignores
%  FontName and renders in Computer Modern, which is why a latex-interpreted
%  label sits beside a Times one and looks wrong. tex honours FontName. It
%  has no \dot, so vertical velocity is written v_z.
% =========================================================================
function s = fig_style()
    s.font       = 'Times New Roman';
    s.fs_axis    = 10;
    s.fs_label   = 11;
    s.fs_panel   = 12;
    s.fs_annot   = 10;
    s.lw_axis    = 0.9;
    s.lw_data    = 1.9;
    s.lw_ref     = 1.0;
    s.lw_phase   = 1.6;

    s.ink        = [0.13 0.13 0.13];
    s.gray       = [0.48 0.48 0.48];
    s.faint      = [0.72 0.72 0.72];
    s.c_height   = [0.05 0.30 0.60];
    s.c_velocity = [0.80 0.30 0.10];
    s.c_command  = [0.22 0.22 0.22];
    s.c_water    = [0.20 0.45 0.68];

    s.fig_width  = 15.0;    % cm, full text width of the thesis page
    s.interp     = 'tex';
end
