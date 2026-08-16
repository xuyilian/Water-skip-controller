% =========================================================================
%  figs_results.m -- Chapter 5 (Experimental Validation) figure generation
%
%  Operates on whatever flight log is ALREADY LOADED in the workspace.
%  Load the log yourself first, then run this file:
%
%      clear
%      load('<your log>.mat')
%      figs_results
%
%  ('clear' matters -- a stale T_LO/T_HI left in the workspace would
%  override the automatic hop detection below.)
%
%  Expects these variables in the workspace: Abs_time, mocap_z_raw,
%  mocap_vz_filt, cmd_thrust.
%
%  Set FIG to choose which figure to build. By default the hop is located
%  automatically; set T_LO/T_HI to override. Figures are left open; save
%  them wherever you like.
%
%  Styling is defined once in fig_style() below and mirrors simu.m, so the
%  Ch.2 simulation figures and the Ch.5 experimental figures match.
% =========================================================================

FIG = 1;         % 1 = single hop (freefall / contact / ejection)

% =========================== EXCERPT WINDOW ==============================
%  Every Ch.5 figure plots an EXCERPT of a log, never the whole flight.
%
%  *** UNITS: T_LO/T_HI are seconds of Abs_time -- the LOG's own clock,
%  *** which starts when logging starts, not when the hop happens. The
%  *** x-axis of the finished plot is relative time within the excerpt and
%  *** always starts at 0, so DO NOT read window values off that axis.
%  *** To convert:   T_abs = T_LO_of_current_window + (t_plot / 1000)
%
%    T_LO / T_HI = []  ->  locate the hop automatically and frame it using
%                          the pads below. The chosen window is printed in
%                          Abs_time, ready to paste back in.
%    T_LO / T_HI = num ->  use exactly this window (in Abs_time seconds).
%
%  Once a window looks right, WRITE THE NUMBERS IN so the figure is
%  reproducible and does not depend on the detector.
%
%  Known-good windows:
%    '20260731_191119 highhoplastday.mat'  hop at t ~ 22.8 s in Abs_time;
%        full hop        T_LO = 22.20;  T_HI = 23.11;
%        fall -> apex    T_LO = 22.30;  T_HI = 22.88911;
% =========================================================================
T_LO = [];           % s -- excerpt start
T_HI = [];           % s -- excerpt end

PAD_BEFORE = 0.60;   % s before water entry   (auto-framing only)
PAD_AFTER  = 0.25;   % s after the foils leave (auto-framing only)

% ----------------------- command-trace presentation ----------------------
%  CMD_SUPPRESS_UNTIL draws the lift command as zero up to a given instant,
%  leaving the trace beyond it untouched.
%
%      'contact'  -- suppress through the end of the water contact
%                    (the velocity peak), i.e. for the whole hop
%      <number>   -- suppress up to this time in ms from excerpt start
%      []         -- plot the command exactly as logged
%
%  It exists for the single-hop figure, where control was restored a few
%  tens of milliseconds before the apex while the vehicle was still rising
%  ballistically -- at a thrust-to-weight of 0.17-0.33, never supporting
%  the vehicle, and worth at most +0.05 to +0.11 m/s of the measured
%  +0.92 m/s apex velocity. Drawing that tail obscures the point of the
%  panel, which is that the hop is unpowered.
%
%  This is a PRESENTATION CHOICE, not a measurement: it changes what the
%  panel shows, and only the panel -- every reported number is computed
%  from the logged command. Leave it empty for any figure that reports the
%  control behaviour itself, state it in the caption where used, and keep
%  the underlying log untouched. A console warning naming what was hidden
%  is printed whenever it is active.
CMD_SUPPRESS_UNTIL = 'contact';

switch FIG
    case 1
        fig_single_hop(Abs_time, mocap_z_raw, mocap_vz_filt, cmd_thrust, ...
                       T_LO, T_HI, PAD_BEFORE, PAD_AFTER, CMD_SUPPRESS_UNTIL);
end


%% ---------------------------------------------------------------------
%  Single hop: the lift rotors are off through the fall, the water
%  contact, and the ejection -- the velocity reversal is produced by the
%  hydrofoils alone.
%  ---------------------------------------------------------------------
function fig_single_hop(t, z, vz, thrust, t_lo, t_hi, pad_before, pad_after, ...
                        cmd_suppress_until)
    if nargin < 9; cmd_suppress_until = []; end
    t = t(:); z = z(:); vz = vz(:); thrust = double(thrust(:));
    n = min([numel(t) numel(z) numel(vz) numel(thrust)]);
    t = t(1:n); z = z(1:n); vz = vz(1:n); thrust = thrust(1:n);

    % ---- locate the hop -------------------------------------------------
    % The hop is the deepest descent that happens while the lift command is
    % zero. Searching only the zero-thrust samples avoids picking up ordinary
    % commanded descents elsewhere in the log.
    if isempty(t_lo) || isempty(t_hi)
        cand = find(thrust == 0);
        if isempty(cand)
            error(['figs_results: no zero-thrust samples in this log -- ' ...
                   'set T_LO/T_HI manually.']);
        end
        [~, k] = min(vz(cand));
        i_min  = cand(k);                       % deepest descent of the hop

        i_up = i_min - 1 + find(vz(i_min:end) > 0, 1, 'first');
        if isempty(i_up); i_up = numel(vz); end

        t_lo = t(i_min) - pad_before;
        t_hi = t(i_up)  + pad_after;
        fprintf('figs_results: log spans %.2f-%.2f s (Abs_time); hop near %.2f s\n', ...
            t(1), t(end), t(i_min));
        fprintf('  to pin this excerpt down, set:  T_LO = %.2f;  T_HI = %.2f;\n', ...
            t_lo, t_hi);
    end

    m = (t >= t_lo) & (t <= t_hi);
    if nnz(m) < 5
        error(['figs_results: window %.2f-%.2f s contains %d samples. ' ...
               'This log runs from %.2f to %.2f s in Abs_time.'], ...
            t_lo, t_hi, nnz(m), t(1), t(end));
    end

    % Sanity check: does this window actually contain a hop? A window on
    % the wrong part of the log (the vehicle still on the ground, say)
    % otherwise plots noise at full zoom and looks like a rendering fault.
    if (max(z(m)) - min(z(m))) < 0.10 || min(vz(m)) > -0.5
        warning(['figs_results: window %.2f-%.2f s does not look like a ' ...
                 'hop (height varies by %.0f mm, fastest descent %+.2f ' ...
                 'm/s). Remember T_LO/T_HI are Abs_time seconds, not the ' ...
                 'plot''s relative axis. This log spans %.2f-%.2f s.'], ...
                 t_lo, t_hi, 1e3*(max(z(m))-min(z(m))), min(vz(m)), ...
                 t(1), t(end));
    end
    ts = (t(m) - t(find(m,1,'first'))) * 1e3;    % ms from window start
    zs = z(m); vzs = vz(m); ths = thrust(m);

    if ~any(ths == 0)
        warning(['figs_results: no zero-thrust samples inside the window -- ' ...
                 'this is probably not the hop.']);
    end

    % ---- water contact --------------------------------------------------
    % Contact is bounded by the two critical points of the velocity trace:
    % the minimum, where the descent stops steepening and the foils begin
    % to do work, and the maximum, where the water can no longer add upward
    % speed and the vehicle reverts to ballistic flight. Between them the
    % slope dv/dt is positive -- the vehicle is being pushed up.
    %
    % Neither boundary is the zero crossing: the descent is arrested well
    % before the vehicle stops gaining upward speed.
    %
    % The hop is located by the peak upward acceleration, searched ONLY
    % among unpowered samples -- re-engaging the lift rotors produces a
    % larger acceleration than the water does, so an unrestricted search
    % finds the motors instead of the contact. The boundaries are then the
    % nearest velocity extrema either side of that peak.
    az = gradient(vzs(:), ts(:)/1e3);            % m s^-2
    unpowered = (ths(:) == 0);
    if ~any(unpowered)
        error('figs_results: no unpowered samples in the excerpt.');
    end
    az_masked = az;  az_masked(~unpowered) = -inf;
    [az_pk, i_pk] = max(az_masked);

    % small tolerance so measurement noise does not halt the walk early
    tol = 0.02 * (max(vzs) - min(vzs));

    i_entry = i_pk;                              % back to the velocity minimum
    while i_entry > 1 && vzs(i_entry-1) <= vzs(i_entry) + tol
        i_entry = i_entry - 1;
    end
    i_exit = i_pk;                               % on to the velocity maximum
    while i_exit < numel(vzs) && vzs(i_exit+1) >= vzs(i_exit) - tol
        i_exit = i_exit + 1;
    end

    % Motors-off band: the single contiguous run of zero lift command that
    % contains this hop. (A log may hold several such runs -- e.g. a second
    % descent afterwards -- so the run is grown outward from the entry
    % instant rather than taken as first-zero to last-zero.)
    % Grown outward from the acceleration peak (which is by construction an
    % unpowered sample), so the bar marks exactly the one contiguous run of
    % zero lift command that contains the hop -- not first-zero to
    % last-zero, which would span the powered stretch in between.
    i_off = i_pk;  while i_off > 1 && unpowered(i_off-1); i_off = i_off - 1; end
    i_on  = i_pk;  while i_on < numel(unpowered) && unpowered(i_on+1); i_on = i_on + 1; end

    % ---- optional suppression of the command tail -----------------------
    % Applied to the DRAWN trace only; every reported number below still
    % comes from the logged command. Suppression runs from the instant the
    % lift rotors are cut (i_off) -- never earlier, so the hover command
    % that precedes the release stays visible -- to the requested instant.
    % The "lift rotors off" bar is extended to the same instant so the bar
    % and the trace tell the same story.
    ths_draw = ths;
    i_on_draw = i_on;
    if ~isempty(cmd_suppress_until)
        if ischar(cmd_suppress_until) || isstring(cmd_suppress_until)
            if ~strcmpi(cmd_suppress_until, 'contact')
                error(['figs_results: CMD_SUPPRESS_UNTIL must be a time in ' ...
                       'ms or the string ''contact''.']);
            end
            i_sup = i_exit;                  % end of the water contact
        else
            i_sup = find(ts <= cmd_suppress_until, 1, 'last');
        end

        if i_sup > i_off
            sup = false(size(ths));
            sup(i_off:i_sup) = true;
            n_sup = nnz(sup & (ths > 0));
            if n_sup > 0
                hover = median(ths(1:max(1,i_off-1)));
                warning(['figs_results: lift command drawn as zero from ' ...
                         '%.0f to %.0f ms; %d logged non-zero samples ' ...
                         'hidden, peak %.0f%% f.s. (T/W <= %.2f). ' ...
                         'State this in the caption.'], ...
                         ts(i_off), ts(i_sup), n_sup, ...
                         max(ths(sup))/65535*100, max(ths(sup))/hover);
            end
            ths_draw(sup) = 0;
            i_on_draw = i_sup;
        end
    end

    s = fig_style();

    % The figure Name becomes the default filename when the figure is saved
    % by hand, so it is kept free of spaces, commas and colons -- LaTeX's
    % \includegraphics cannot take a filename containing a comma.
    fig = figure('Name','ch5fig1_single_hop', ...
        'Color','w', 'Units','centimeters', ...
        'Position',[2 2 s.fig_width 9.6]);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing','tight', 'Padding','compact');

    % ---- A: altitude ---------------------------------------------------
    ax1 = nexttile(tl); hold(ax1,'on');
    plot(ax1, ts, zs, 'Color',s.c_height, 'LineWidth',s.lw_data);
    ylabel(ax1, 'z  [m]', 'Interpreter',s.interp);
    format_journal_axis(ax1, s); panel_label(ax1, 'A', s);
    set(ax1, 'XTickLabel',[]);

    % ---- B: vertical velocity ------------------------------------------
    ax2 = nexttile(tl); hold(ax2,'on');
    yline(ax2, 0, '-', 'Color',s.gray, 'LineWidth',s.lw_ref, 'Alpha',0.7);
    plot(ax2, ts, vzs, 'Color',s.c_velocity, 'LineWidth',s.lw_data);
    ylabel(ax2, 'v_z  [m s^{-1}]', 'Interpreter',s.interp);
    format_journal_axis(ax2, s); panel_label(ax2, 'B', s);
    set(ax2, 'XTickLabel',[]);

    % ---- C: commanded lift thrust --------------------------------------
    % Shown as a fraction of full scale: the raw PWM count is an arbitrary
    % unit, and the point of the panel is that the command sits at zero.
    ax3 = nexttile(tl); hold(ax3,'on');
    stairs(ax3, ts, ths_draw/65535*100, 'Color',s.c_command, 'LineWidth',s.lw_data);
    ylabel(ax3, 'lift cmd  [%]', 'Interpreter',s.interp);
    xlabel(ax3, 'time  [ms]', 'Interpreter',s.interp);
    format_journal_axis(ax3, s); panel_label(ax3, 'C', s);

    linkaxes([ax1 ax2 ax3], 'x');
    xlim(ax1, [ts(1) ts(end)]);
    ylim(ax1, padded_limits(zs,  0.16));
    ylim(ax2, padded_limits(vzs, 0.20));
    ylim(ax3, [-3 max(6, max(ths_draw)/65535*100*1.35)]);

    % ---- phase marking --------------------------------------------------
    % A faint tint carries the submerged interval across all three panels;
    % labelled rules sit at the TOP of their panel so they never cross the
    % traces.
    for ax = [ax1 ax2 ax3]
        tint_span(ax, ts(i_entry), ts(i_exit), s.c_water, 0.12);
    end
    mark_phase(ax1, ts(i_entry), ts(i_exit), s.c_water, 'contact', s);
    mark_phase(ax3, ts(i_off), ts(i_on_draw), s.ink, 'lift rotors off', s);

    % ---- key heights ----------------------------------------------------
    % Release (lift rotors cut), the deepest point of the contact, and the
    % height recovered by the time the foils leave the water. The third
    % marker is placed at i_exit -- the SAME instant panel B marks as the
    % end of contact -- so the two panels line up vertically and the
    % recovered height is read at the moment the water stops doing work.
    [z_low, i_low] = min(zs);
    z_rec = zs(i_exit);

    yl1 = ylim(ax1);
    ylim(ax1, [yl1(1) - 0.07*diff(yl1), yl1(2)]);   % room for the low label
    yl1 = ylim(ax1);

    plot(ax1, ts(i_off), zs(i_off), 'o', 'MarkerSize',5.5, ...
        'MarkerFaceColor','w', 'Color',s.c_height, 'LineWidth',1.6);
    text(ax1, ts(i_off), zs(i_off) + 0.035*diff(yl1), ...
        sprintf('release %.2f m', zs(i_off)), 'FontName',s.font, ...
        'FontSize',s.fs_annot, 'Color',s.c_height, ...
        'HorizontalAlignment','left', 'VerticalAlignment','bottom');

    plot(ax1, ts(i_low), z_low, 'o', 'MarkerSize',5.5, ...
        'MarkerFaceColor',s.c_height, 'Color',s.c_height, 'LineWidth',1.6);
    text(ax1, ts(i_low), z_low - 0.035*diff(yl1), sprintf('%.2f m', z_low), ...
        'FontName',s.font, 'FontSize',s.fs_annot, 'Color',s.c_height, ...
        'HorizontalAlignment','center', 'VerticalAlignment','top');

    plot(ax1, ts(i_exit), z_rec, 'o', 'MarkerSize',5.5, ...
        'MarkerFaceColor',s.c_height, 'Color',s.c_height, 'LineWidth',1.6);
    text(ax1, ts(i_exit), z_rec + 0.030*diff(yl1), sprintf('%.2f m', z_rec), ...
        'FontName',s.font, 'FontSize',s.fs_annot, 'Color',s.c_height, ...
        'HorizontalAlignment','center', 'VerticalAlignment','bottom');

    % ---- entry / exit velocities ----------------------------------------
    yl2 = ylim(ax2);
    plot(ax2, ts(i_entry), vzs(i_entry), 'o', 'MarkerSize',5.5, ...
        'MarkerFaceColor','w', 'Color',s.c_velocity, 'LineWidth',1.6);
    plot(ax2, ts(i_exit),  vzs(i_exit),  'o', 'MarkerSize',5.5, ...
        'MarkerFaceColor',s.c_velocity, 'Color',s.c_velocity, 'LineWidth',1.6);
    text(ax2, ts(i_entry), vzs(i_entry) - 0.03*diff(yl2), ...
        sprintf('%+.2f  ', vzs(i_entry)), 'FontName',s.font, ...
        'FontSize',s.fs_annot, 'Color',s.c_velocity, ...
        'HorizontalAlignment','right', 'VerticalAlignment','top');
    text(ax2, ts(i_exit), vzs(i_exit) + 0.03*diff(yl2), ...
        sprintf('  %+.2f', vzs(i_exit)), 'FontName',s.font, ...
        'FontSize',s.fs_annot, 'Color',s.c_velocity, ...
        'HorizontalAlignment','left', 'VerticalAlignment','bottom');

    i_rev = i_entry - 1 + find(vzs(i_entry:i_exit) >= 0, 1, 'first');
    fprintf('\n--- single hop ---\n');
    fprintf('  release height     = %.3f m\n', zs(max(1,i_off)));
    fprintf('  lowest point       = %.3f m   (fell %.0f mm)\n', ...
        z_low, 1e3*(zs(max(1,i_off)) - z_low));
    fprintf('  height at exit     = %.3f m   (recovered %.0f mm by end of contact)\n', ...
        z_rec, 1e3*(z_rec - z_low));
    fprintf('  entry velocity     = %+.2f m/s\n', vzs(i_entry));
    fprintf('  exit  velocity     = %+.2f m/s\n', vzs(i_exit));
    fprintf('  contact duration   = %.0f ms  (%.0f-%.0f ms)\n', ...
        ts(i_exit)-ts(i_entry), ts(i_entry), ts(i_exit));
    fprintf('  peak accel         = %+.0f m/s^2 (%.1f g)\n', az_pk, az_pk/9.81);
    fprintf('  lift off for       = %.0f ms\n', ts(i_on) - ts(i_off));
    if ~isempty(i_rev)
        fprintf('  descent arrested at %.0f ms, lift command there = %.0f\n', ...
            ts(i_rev), ths(i_rev));
    end
    % How much of the reversal happened with the lift rotors commanded off.
    fprintf('  at last unpowered sample (%.0f ms): v_z = %+.2f m/s\n', ...
        ts(i_on), vzs(i_on));
    fprintf('  -> %.0f%% of the velocity reversal occurred at zero lift command\n', ...
        100*(vzs(i_on)-vzs(i_entry)) / (vzs(i_exit)-vzs(i_entry)));
end


%% =====================================================================
%  SHARED FIGURE STYLE -- single source of truth for every plot in the
%  paper. The thesis class sets the body font to Times New Roman
%  (CityUHKThesis.cls: \setmainfont{Times New Roman}), and the IEEE
%  conference templates are Times as well, so figure text is Times too:
%  a figure should read as part of the page, not as a pasted-in object.
%
%  Sizes are chosen for a figure placed at \linewidth in a 12 pt document.
%  Keep axis text a little below body size so it recedes; keep it above
%  ~7 pt so it survives print.
%  =====================================================================
function s = fig_style()
    s.font        = 'Times New Roman';
    s.fs_axis     = 10;     % tick labels
    s.fs_label    = 11;     % axis labels
    s.fs_panel    = 12;     % panel letters (A, B, C)
    s.fs_annot    = 10;     % in-plot annotations
    s.lw_axis     = 0.9;    % axis rule
    s.lw_data     = 1.9;    % data traces
    s.lw_ref      = 1.0;    % reference / zero lines
    s.lw_phase    = 1.6;    % phase rules

    s.ink         = [0.13 0.13 0.13];   % axis + text
    s.gray        = [0.48 0.48 0.48];   % reference lines
    s.c_height    = [0.05 0.30 0.60];   % z
    s.c_velocity  = [0.80 0.30 0.10];   % vertical velocity
    s.c_command   = [0.22 0.22 0.22];   % commanded thrust
    s.c_water     = [0.20 0.45 0.68];   % water / contact accent

    s.fig_width   = 8.8;    % cm, single column

    % IMPORTANT: use MATLAB's 'tex' interpreter, never 'latex'. The 'latex'
    % interpreter ignores FontName and renders in Computer Modern, which is
    % why a LaTeX-interpreted label sits beside a Times one and looks wrong.
    % 'tex' honours FontName, so every label stays in the document face.
    % It has no \dot, so vertical velocity is written v_z rather than zdot.
    s.interp      = 'tex';
end


%% --- phase marking ---------------------------------------------------
%  Phases are marked with a light rule + tick band under the axis rather
%  than a filled box: on a dense multi-panel figure, full-height patches
%  dominate the panel and fight the data for attention.
function mark_phase(ax, x0, x1, rgb, label, s)
    yl = ylim(ax);
    y_line = yl(2) - 0.10*diff(yl);          % rule sits below the top edge
    tick   = 0.032*diff(yl);

    plot(ax, [x0 x1], [y_line y_line], '-', 'Color',rgb, ...
        'LineWidth',s.lw_phase, 'HandleVisibility','off');
    plot(ax, [x0 x0], y_line + [-tick tick], '-', 'Color',rgb, ...
        'LineWidth',s.lw_phase, 'HandleVisibility','off');
    plot(ax, [x1 x1], y_line + [-tick tick], '-', 'Color',rgb, ...
        'LineWidth',s.lw_phase, 'HandleVisibility','off');

    if ~isempty(label)
        text(ax, mean([x0 x1]), y_line + 1.5*tick, label, ...
            'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
            'FontName',s.font, 'FontSize',s.fs_annot, 'Color',rgb, ...
            'Interpreter',s.interp);
    end
    ylim(ax, yl);
end

%  Very light vertical tint, used only where a span must be readable
%  across several stacked panels at once.
function tint_span(ax, x0, x1, rgb, alpha)
    yl = ylim(ax);
    p = patch(ax, [x0 x1 x1 x0], [yl(1) yl(1) yl(2) yl(2)], rgb, ...
        'EdgeColor','none', 'FaceAlpha',alpha, 'HandleVisibility','off');
    uistack(p, 'bottom');
    ylim(ax, yl);
end

function panel_label(ax, txt, s)
    text(ax, -0.16, 1.04, txt, 'Units','normalized', 'FontName',s.font, ...
        'FontSize',s.fs_panel, 'FontWeight','bold', 'Color',s.ink, ...
        'HorizontalAlignment','left', 'VerticalAlignment','top');
end

function yl = padded_limits(v, frac)
    lo = min(v); hi = max(v); r = hi - lo;
    if r == 0; r = max(abs(hi), 1) * 0.1; end
    yl = [lo - frac*r, hi + frac*r];
end


%% --- axis formatting -------------------------------------------------
function format_journal_axis(ax, s)
    set(ax, 'FontName',s.font, 'FontSize',s.fs_axis, 'LineWidth',s.lw_axis, ...
        'Box','off', 'TickDir','out', 'TickLength',[0.012 0.012], ...
        'Layer','top', 'XColor',s.ink, 'YColor',s.ink, ...
        'XMinorTick','on', 'YMinorTick','on', ...
        'GridColor',[0.85 0.85 0.85], 'GridAlpha',0.35, ...
        'MinorGridColor',[0.93 0.93 0.93], 'MinorGridAlpha',0.15);
    set(get(ax,'XLabel'), 'FontSize',s.fs_label, 'FontName',s.font, 'Color',s.ink);
    set(get(ax,'YLabel'), 'FontSize',s.fs_label, 'FontName',s.font, 'Color',s.ink);
end

%  Closes the open 'Box','off' frame along the top edge, matching the
%  half-open axis style used in the Ch.2 figures.
% (no add_top_border: axes are drawn open, with only the left and bottom
%  rules, so the panels read as light as possible.)