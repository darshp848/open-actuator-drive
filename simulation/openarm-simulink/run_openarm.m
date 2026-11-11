function R = run_openarm(name, opts)
% Runs one scenario on openarm_system.slx and returns logged signals plus pass/fail screens.
%   name: 'lift6' (6 kg total end load, J1 0 -> 90 deg in 1 s, hold, lower in 1 s; Codex load-study case),
%         'drop6' (same load lowered from horizontal in 1.0 s, near the GIM8115 speed limit: high regen), 'drop6_auxfail' (aux lost before the drop,
%         primary chopper dead), 'drop6_nobrake' (aux lost and no backup: what the backup prevents).
%   Harder cases: 'multi_fast' (all seven joints, raised pose to a folded low pose and back, each segment timed so the
%         slowest joint peaks at 85 % of its listed speed), 'multi_fast_auxfail', 'estop_coast' (drop6 with gates off at
%         peak speed: the arm falls and the motors regenerate through the body diodes), 'estop_coast_auxfail',
%         'estop_ramp' (drop6 stopped from peak speed in 0.15 s), 'release6' (6 kg held horizontal, payload drops at
%         1.6 s without telling the controller). A '_batt' suffix runs any case from the 12S battery instead of the PSU.
arguments
    name (1,1) string
    opts.source (1,1) string = "psu"
    opts.bank_f (1,1) double {mustBePositive,mustBeFinite} = 1.248e-3
    opts.rotor_j_scale (1,1) double {mustBeNonnegative,mustBeFinite} = 1
    opts.build (1,1) logical = false
    opts.stop_time (1,1) double {mustBeNonnegative,mustBeFinite} = 0
    opts.timeout_s (1,1) double {mustBePositive,mustBeFinite} = 120
    opts.solver (1,1) string {mustBeMember(opts.solver,["saved","ode23t","ode15s","daessc"])} = "saved"
    opts.max_step (1,1) double {mustBeNonnegative,mustBeFinite} = 0
    opts.rel_tol (1,1) double {mustBeNonnegative,mustBeFinite} = 0
    opts.validate_only (1,1) logical = false
end
here = fileparts(mfilename('fullpath'));
mdl = 'openarm_system';
assert(~opts.build, 'OpenArm:SeparateBuild', ...
    'Build separately with build_openarm_system, then validate before solving.');
assert(isfile(fullfile(here,[mdl '.slx'])), 'OpenArm:MissingModel', ...
    'Missing saved model. Build separately with build_openarm_system.');
assert(strcmp(which([mdl '.slx']),fullfile(here,[mdl '.slx'])), ...
    'OpenArm:ModelPath', 'Add this system_model folder to the MATLAB path first.');
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'FileName'),fullfile(here,[mdl '.slx'])) && ...
        strcmp(get_param(mdl,'Dirty'),'off'), 'OpenArm:LoadedModel', ...
        'A different or unsaved openarm_system is loaded. Use an isolated MATLAB process.');
end
if endsWith(name, "_batt"), opts.source = "battery"; base = extractBefore(name, "_batt"); else, base = name; end
P = openarm_params(source=opts.source, bank_f=opts.bank_f, rotor_j_scale=opts.rotor_j_scale);
P.k_lim = 50; P.pri_en = 1; P.t_aux_fail = inf; P.t_drv_off = inf;
horiz = [pi/2 0 0 0 0 0 0];
ramp = [];
switch base
    case "lift6"
        wp = [0 zeros(1,7); 0.2 zeros(1,7); 1.2 horiz; 2.2 horiz; 3.2 zeros(1,7); 3.5 zeros(1,7)]; mass = 6.0;
    case {"drop6","drop6_auxfail","drop6_nobrake"}
        wp = [0 horiz; 0.3 horiz; 1.3 zeros(1,7); 1.8 zeros(1,7)]; mass = 6.0;   % 1 s: J1 peak 2.95 rad/s of 3.77
        if base ~= "drop6", P.t_aux_fail = 0.1; end
        if base == "drop6_nobrake", P.backup = false; end
    case {"multi_fast","multi_fast_auxfail"}
        up = [2.0 0.6 0.8 0.2 0.8 0.5 1.0]; lo = [0 0 -0.8 1.6 -0.8 -0.5 -1.0];
        T = max(1.875*abs(lo - up)./(0.85*P.w_max));
        wp = [0 up; 0.3 up; 0.3+T lo; 0.7+T lo; 0.7+2*T up; 1.0+2*T up]; mass = 6.0;
        if base == "multi_fast_auxfail", P.t_aux_fail = 0.1; end
    case {"estop_coast","estop_coast_auxfail"}
        wp = [0 horiz; 0.3 horiz; 1.3 zeros(1,7); 1.8 zeros(1,7)]; mass = 6.0;
        P.t_drv_off = 0.8;                                    % gates off at peak lowering speed
        P.Ts_dyn = 2e-5;                                      % the arm whips once free: hold arm terms only 20 us
        if base == "estop_coast_auxfail", P.t_aux_fail = 0.1; end
    case "estop_ramp"
        wp = [0 horiz; 0.3 horiz; 1.3 zeros(1,7); 1.8 zeros(1,7)]; mass = 6.0;
        ramp = [0.8 0.15];                                    % stop from peak speed: start, duration
    case "release6"
        wp = [0 zeros(1,7); 0.2 zeros(1,7); 1.2 horiz; 2.6 horiz]; mass = 6.0;
        P.t_release = 1.6;
    otherwise
        error('unknown scenario %s', name)
end
robot = copy(P.robot);
payload = rigidBody('test_payload');
payload.Mass = max(0, mass - 0.768019047366671); payload.CenterOfMass = [0 0 -P.payload_offset];
payload.Inertia = [0 0 0 0 0 0];
addBody(robot, payload, 'openarm_right_ee_base_link');
t = (0:1e-3:wp(end,1))';
[q, qd, qdd] = quintic_path(wp, t);
if ~isempty(ramp)
    k0 = find(t >= ramp(1), 1); v0 = qd(k0,:);
    for k = k0+1:numel(t)
        f = max(0, 1 - (t(k) - ramp(1))/ramp(2));
        qd(k,:) = v0*f; qdd(k,:) = -v0/ramp(2)*(f > 0); q(k,:) = q(k-1,:) + 0.5*(qd(k,:) + qd(k-1,:))*(t(k) - t(k-1));
    end
end
ref_ts = timeseries([q qd qdd], t);
assert(all(max(abs(qd), [], 1) <= 0.9*P.w_max + 1e-9), 'reference exceeds 90 %% of a joint''s listed maximum speed')
scen = struct('q0', wp(1,2:8)', 'v0', P.v_src - 0.3);
Pm = numeric_only(P);
stop = wp(end,1); if opts.stop_time > 0, stop = opts.stop_time; end
assert(stop <= wp(end,1), 'OpenArm:ReferenceDuration', 'Requested stop exceeds the scenario reference.');
assert(all(isfinite([q qd qdd]),'all'), 'OpenArm:ReferenceFinite', 'Reference contains nonfinite values.');
assert(all(q >= P.q_lo & q <= P.q_hi,'all'), 'OpenArm:ReferenceLimits', 'Reference exceeds a joint hard stop.');
in = Simulink.SimulationInput(mdl);
in = in.setVariable('Pm',Pm);
in = in.setVariable('robot_sim',robot);
in = in.setVariable('robot_free',copy(P.robot));
in = in.setVariable('ref_ts',ref_ts);
in = in.setVariable('scen',scen);
in = in.setModelParameter('StopTime',num2str(stop,17), ...
    'TimeOut',opts.timeout_s,'CaptureErrors','on');
if opts.solver ~= "saved", in = in.setModelParameter('Solver',char(opts.solver)); end
if opts.max_step > 0, in = in.setModelParameter('MaxStep',num2str(opts.max_step,17)); end
if opts.rel_tol > 0, in = in.setModelParameter('RelTol',num2str(opts.rel_tol,17)); end
validate(in);
if opts.validate_only
    R = struct('name',name,'validated',true,'requested_stop_s',stop, ...
        'matlab_version',string(version),'options',opts);
    R.simulation_input = in;
    return
end
tic; out = sim(in); R.wall_s = toc;
R.sim_error = string(out.ErrorMessage);              % non-empty: the solver stopped early; data run up to that time
if strlength(R.sim_error) > 0, fprintf(2, '%s stopped early: %s\n', name, extractBefore(R.sim_error + newline, newline)); end
assert(any(strcmp(who(out),'arm_log')), 'OpenArm:MissingLog', ...
    'Simulation produced no arm_log. Error: %s',R.sim_error);
L = reshape(out.arm_log, size(out.arm_log,1), [])';   % To Workspace array logs 68 x 1 x N
assert(size(L,2) == 68 && size(L,1) >= 2, 'OpenArm:LogShape', ...
    'Expected at least two samples with 68 logged channels.');
% log columns: 1 t | 2-8 q | 9-15 qd | 16-22 q ref | 23-29 iq | 30-36 iq ref | 37-43 tau | 44-50 pdc | 51-57 v local |
% 58 v_bat | 59 v_bus | 60 i_src | 61 v_ds | 62 r_hs | 63 chopper on | 64 primary | 65 backup | 66 aux | 67 hs off | 68 i_hop1
blk7 = @(k) L(:, 2 + 7*k + (0:6));
R.t = L(:,1); R.q = blk7(0); R.qd = blk7(1); R.qr = blk7(2); R.iq = blk7(3); R.iq_ref = blk7(4); R.tau = blk7(5);
R.pdc = blk7(6); R.vloc = blk7(7);
R.v_bat = L(:,58); R.v_bus = L(:,59); R.i_src = L(:,60); R.v_ds = L(:,61); R.r_hs = L(:,62); R.chop = L(:,63);
R.pri = L(:,64); R.bak = L(:,65); R.aux = L(:,66); R.hs_off = L(:,67); R.i_hop1 = L(:,68);
R.name = name; R.P = rmfield(P,'robot'); R.mass = mass;
R.requested_stop_s = stop;
R.scenario_stop_s = wp(end,1);
R.options = opts;
R.matlab_version = string(version);
R.model_file = string(fullfile(here,[mdl '.slx']));
R.simulation_metadata = out.SimulationMetadata;
R.validation = openarm_run_integrity(L,stop,P.Ts_log,R.sim_error, ...
    string(out.SimulationMetadata.ExecutionInfo.StopEvent));
R.validation.warning_free = isempty(out.SimulationMetadata.ExecutionInfo.WarningDiagnostics);
R.validation.full_scenario = R.validation.completed && abs(stop - wp(end,1)) < 1e-9;
dt = [diff(R.t); 0];
R.metrics = struct( ...
    'track_err_max_deg', max(abs(R.q - R.qr), [], 'all')*180/pi, ...
    'v_bus_max', max(R.v_bus), 'v_bus_min', min(R.v_bus), 'v_local_min', min(R.vloc, [], 'all'), ...
    'v_local_max', max(R.vloc, [], 'all'), 'i_src_max', max(R.i_src), ...
    'i_src_mean', sum(R.i_src.*dt)/R.t(end), 'i_hop1_max', max(abs(R.i_hop1)), ...
    'i_hop1_rms', sqrt(sum(R.i_hop1.^2.*dt)/R.t(end)), ...
    'iq_max', max(abs(R.iq), [], 1), 'iq_sat_frac', mean(abs(R.iq_ref) >= R.P.i_max - 1e-6, 1), ...
    'p_regen_min_w', min(sum(R.pdc, 2)), 'brake_energy_j', sum(R.chop.*R.v_bus.^2/R.P.r_brake.*dt), ...
    'hot_swap_trips', sum(diff(R.hs_off) > 0.5), 'chop_on_frac', mean(R.chop), 'wall_s', R.wall_s, 't_end', R.t(end));
R.screens = struct('run_complete',R.validation.completed,'finite_monotonic_log',R.validation.finite_monotonic, ...
    'solver_warning_free',R.validation.warning_free, ...
    'full_scenario',R.validation.full_scenario, ...
    'bus_below_80v', R.metrics.v_bus_max < 80, 'boards_above_20v', R.metrics.v_local_min > 20, ...
    'no_hot_swap_trip', R.metrics.hot_swap_trips == 0, 'source_peak_le_50a', R.metrics.i_src_max <= 50, ...
    'tracking_lt_2deg', R.metrics.track_err_max_deg < 2);
R.metrics.qd_max_frac = max(abs(R.qd), [], 1)./R.P.w_max;     % actual joint speed / listed maximum
R.metrics.e_bus_peak_j = 0.5*R.P.bank_c*(R.metrics.v_bus_max^2 - R.P.v_src^2);
end

function [q, qd, qdd] = quintic_path(wp, t)
q = zeros(numel(t),7); qd = q; qdd = q;
for k = 1:size(wp,1)-1
    t0 = wp(k,1); t1 = wp(k+1,1); T = t1 - t0;
    m = t >= t0 & (t < t1 | (k == size(wp,1)-1 & t <= t1));
    s = (t(m) - t0)/T;
    b = 10*s.^3 - 15*s.^4 + 6*s.^5; db = (30*s.^2 - 60*s.^3 + 30*s.^4)/T; ddb = (60*s - 180*s.^2 + 120*s.^3)/T^2;
    d = wp(k+1,2:8) - wp(k,2:8);
    q(m,:) = wp(k,2:8) + b.*d; qd(m,:) = db.*d; qdd(m,:) = ddb.*d;
end
end

function S = numeric_only(P)
S = struct();
for f = fieldnames(P)'
    v = P.(f{1});
    if isnumeric(v) || islogical(v), S.(f{1}) = double(v); end
end
end
