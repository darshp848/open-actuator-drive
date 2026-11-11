function P = openarm_params(opts)
% Parameters for the one-arm OpenArm v2 system model (openarm_system.slx): SteadyWin GIM actuators driven by the
% two-story actuator boards, one support board per arm. Every number below cites its source or is marked ASSUMED.
%
% Motors (steadywin.cn product pages, read 2026-10-25):
%   J1-J2 GIM8115-36, 48 V delta winding: Kt 11.05 Nm/A (output), stall 21 A, 0.75 rpm/V output, R_ll 0.98 ohm,
%         L_ll 0.55 mH, 21 pole pairs, 920 g, 36:1, 15 arcmin.
%   J3-J4 GIM8108-36: Kt 7.32 Nm/A, stall 19.8 A, 1.25 rpm/V output, R_ll 0.22 ohm, L_ll 0.21 mH, 21 pp, 720 g, 36:1.
%         Page stall torque 109.5 Nm does not equal 7.32 x 19.8 = 145 Nm; the model limits torque at 109.5 Nm.
%   J5-J7 GIM4310-40: Kt 1.79 Nm/A, stall 17.1 A, 3.58 rpm/V output, R_ll 0.27 ohm, L_ll 0.8 mH, 14 pp, 337 g, 40:1.
%         Rated 12-40 V (24 V nominal); the 48 V bus is above it: OPEN with SteadyWin.
%   Listed Kt is taken as output torque per motor amp. Motor Kt = Kt_out / (N x eta). Rotor inertia, gear efficiency
%   and friction are not published: ASSUMED values below, varied in the sensitivity runs.
% Boards: support board (support-board-finalization-20261025.md): hot-swap 0.9 mOhm sense, VCL 48.5/55/61.5 mV,
%   power limit 305 W, fault timer 0.71 ms (0.42-1.35), LM5069-2 retry at 0.5 % duty; LTC4359 + 2 x BSC040N10NS5;
%   bank 12 x 130 uF (1.248 mF at -20 %); primary chopper 57.1/55.0 V on aux; backup 61.13/60.29 V on BUS+;
%   brake 4 x 10 ohm = 2.5 ohm. Daisy chain 0.2 uH + 10 mOhm per hop and 76 uF local ceramics per actuator board
%   (capture-20261024.md section 15 assumptions). Actuator inverter ISC060N10NM6 6 mOhm max.
arguments
    opts.bank_f (1,1) double = 1.248e-3
    opts.source (1,1) string {mustBeMember(opts.source,["psu","battery"])} = "psu"
    opts.source_v (1,1) double = 48
    opts.aux_ok (1,1) logical = true
    opts.backup (1,1) logical = true
    opts.rotor_j_scale (1,1) double = 1
end
P.opts = opts;
%% actuators, joint order J1..J7
%% VARIANT 'coupled size_ndfeb': per-joint actuators from the Inspiron coupled sizing (set size_ndfeb); only this block differs from the original.
%% Gearbox types: planetary2, planetary2, planetary2, planetary2, planetary2, planetary2, planetary2. Rotor + gearbox inertia from the CAD index (reflected CAD inertia / N^2). Torque constant = FEMM torque / I_peak.
N   = [64 64 64 64 64 64 64];
eta = [0.9025 0.9025 0.9025 0.9025 0.9025 0.9025 0.9025];
P.N = N; P.eta = eta;
P.kt_m = [0.181146936 0.181146936 0.168350714 0.168350714 0.166624458 0.166624458 0.166624458];
P.pp = [28 28 28 28 14 14 14];
P.lambda = [0.00431333687 0.00431333687 0.00400855871 0.00400855871 0.00793503045 0.00793503045 0.00793503045];
P.R = [0.373216175 0.373216175 0.911338263 0.911338263 2.43863031 2.43863031 2.43863031];
P.L = [0.000165495115 0.000165495115 0.000314659226 0.000314659226 0.000683753209 0.000683753209 0.000683753209];
P.i_max = [6.37148021 6.37148021 4.02093749 4.02093749 1.92317513 1.92317513 1.92317513];
P.tau_max = N.*P.kt_m.*eta.*P.i_max;
P.J_rotor = opts.rotor_j_scale*[0.000101856686 0.000101856686 5.29366608e-05 5.29366608e-05 2.46931194e-05 2.46931194e-05 2.46931194e-05];
P.J_refl = P.J_rotor.*N.^2;
P.tau_coulomb = [1.01459011 1.01459011 0.565790813 0.565790813 0.253477537 0.253477537 0.253477537];
P.b_visc = [0.634118819 0.634118819 0.377193875 0.377193875 0.0844925124 0.0844925124 0.0844925124];
P.motor_mass = [0.786884613 0.786884613 0.54690417 0.54690417 0.349633311 0.349633311 0.349633311];
P.w_max = (0.95/sqrt(3)*48)./(P.pp.*P.N.*P.lambda);
%% current loop (per joint), bandwidth wc
wc = 2*pi*1000;
P.kp_i = P.L*wc; P.ki_i = P.R*wc;
P.v_frac = 0.95/sqrt(3);                                    % SVPWM phase-voltage limit, x Vbus
%% inverter and board losses
P.rds = 6e-3*1.5;                                           % ISC060N10NM6 max x 1.5 hot (ASSUMED factor)
P.p_board = 3.0;                                            % ASSUMED logic + gate drive + supplies per actuator board, W
P.fsw = 100e3; P.t_sw = 25e-9;                              % 100 kHz center-aligned TIM1 PWM (MCU sheet); ASSUMED edge time
P.i_eps = 0.3;                                              % gates-off diode-bridge smoothing current, A (model only)
%% joint position loop (computed torque, natural frequency wn)
wn = 2*pi*5;
P.kp_q = 3*wn^2*ones(1,7); P.kd_q = 3*wn*ones(1,7); P.ki_q = wn^3*ones(1,7);   % PID, triple pole at wn
P.eta_hat = 0.95;                                           % controller's nominal gear efficiency (plant: 0.9 / 1/0.9)
%% arm
root = fileparts(fileparts(mfilename('fullpath')));
robot = importrobot(fullfile(root,'build','openarm-v2-load','right-load-only.urdf'),DataFormat='column');
robot.Gravity = [0 0 -9.81];
% the URDF masses include the stock Damiao motors; add the GIM excess at each joint's child body (ASSUMED stock masses)
stock = [0.80 0.80 0.36 0.36 0.30 0.30 0.30];
child = [compose("openarm_right_link%d",1:6) "openarm_right_ee_base_link"];
for j = 1:7
    b = getBody(robot,child(j));
    m0 = b.Mass; dm = max(P.motor_mass(j)-stock(j),0);
    b.CenterOfMass = (b.CenterOfMass*m0)/(m0+dm); b.Mass = m0+dm;
end
P.robot = robot;
lim = zeros(7,2);
for j = 1:7, lim(j,:) = getBody(robot, child(j)).Joint.PositionLimits; end
P.q_lo = lim(:,1)' - 2*pi/180; P.q_hi = lim(:,2)' + 2*pi/180;     % hard stops 2 deg outside the URDF limits
P.k_stop = 5e3; P.c_stop = 50;                              % ASSUMED stop stiffness Nm/rad and damping Nm s/rad
P.payload_kg = 0; P.payload_offset = 0.15; P.t_release = inf;
%% support board
P.vcl = [48.5 55 61.5]*1e-3; P.rs = 0.9e-3;
P.i_lim = P.vcl(2)/P.rs;                                    % 61.1 A typical
P.p_lim = 305;                                              % W at 54 V
P.t_flt = 0.71e-3; P.retry_duty = 0.005;
P.r_hs = 2*2.4e-3*1.2/2 + P.rs;                              % 2 x CSD19536KTT parallel, warm, + sense
P.r_id = 4e-3*1.2/2;                                        % 2 x BSC040N10NS5 parallel
P.bank_c = opts.bank_f; P.bank_esr = 1.333e-3;
P.r_brake = 2.5;
P.chop_pri = [57.1 55.0]; P.chop_bak = [61.13 60.29]; P.t_bak = [29e-6 18e-6];
P.aux_ok = opts.aux_ok; P.backup = opts.backup;
P.p_aux = 2.0;
P.Ts_sup = 20e-6;                                          % supervisor / sampled-logic rate
P.Ts_log = 50e-6;
P.Ts_dyn = 2e-4;                                           % arm M / Coriolis / gravity update rate (held)                                          % logging rate                                              % ASSUMED support-board housekeeping, W
%% source and wiring
P.source = opts.source;
if opts.source == "battery"
    P.v_src = 50.4; P.r_src = 40e-3;                       % 12S full charge, ASSUMED internal resistance
else
    P.v_src = opts.source_v; P.r_src = 10e-3;              % bench PSU, ASSUMED output resistance, no sink
end
P.l_lead = 1e-6; P.r_lead = 10e-3;                         % ASSUMED source lead
P.l_hop = 0.2e-6; P.r_hop = 10e-3; P.c_local = 76e-6; P.esr_local = 5e-3;
end
