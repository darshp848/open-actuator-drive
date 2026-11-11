function [dx, qdd, idc, lg] = ctrl(t, q, qd, ref, M, vp, g, x, vloc, en, M0, vp0, g0, Pm)
% Joint control, averaged FOC actuators and arm acceleration for openarm_system.slx (all vectors are joint 1..7).
% Computed-torque PID position loop -> iq command -> dq PI current loops with back-EMF feedforward and the SVPWM voltage
% limit set by each board's local bus -> torque through gear efficiency (eta motoring, 1/eta generating) and friction
% -> qdd = (M + J_refl) \ (tau - vp - g). DC current per board = (dq power + conduction + switching + board) / V.
% M, vp, g include the payload and are what the controller uses throughout; M0, vp0, g0 are the arm without it. The plant
% switches to them at Pm.t_release (the controller is not told). en = 0 is gates off (safe torque off): each inverter is
% a diode bridge, so the motor drives current into its board's bus only when the back-EMF exceeds vloc / sqrt(3).
% Joint hard stops 2 deg outside the URDF limits: spring Pm.k_stop and damper Pm.c_stop while in contact.
N = Pm.N(:); kt = Pm.kt_m(:); eta = Pm.eta(:); L = Pm.L(:); R = Pm.R(:); lam = Pm.lambda(:); pp = Pm.pp(:);
id = x(1:7); iq = x(8:14); zd = x(15:21); zq = x(22:28); ze = x(29:35);
qr = ref(1:7); qdr = ref(8:14); qddr = ref(15:21);
Mt = M + diag(Pm.J_refl(:));
fric = Pm.tau_coulomb(:).*tanh(qd/0.01) + Pm.b_visc(:).*qd;
a = qddr + Pm.kp_q(:).*(qr - q) + Pm.kd_q(:).*(qdr - qd) + Pm.ki_q(:).*ze;
tau_cmd = Mt*a + vp + g + fric;
tau_cmd = min(max(tau_cmd, -Pm.tau_max(:)), Pm.tau_max(:));
iq_ref = en*tau_cmd./(N.*kt*Pm.eta_hat);       % controller assumes one nominal efficiency
iq_ref = min(max(iq_ref, -Pm.i_max(:)), Pm.i_max(:));
wm = N.*qd; we = pp.*wm;
ed = -id; eq = iq_ref - iq;
vd = Pm.kp_i(:).*ed + zd - we.*L.*iq;
vq = Pm.kp_i(:).*eq + zq + we.*lam + we.*L.*id;
vmax = Pm.v_frac*max(vloc, 0);
vm = sqrt(vd.^2 + vq.^2);
s = min(1, vmax./max(vm, 1e-9));
vd = vd.*s; vq = vq.*s;
free = double(s >= 0.999);
if en < 0.5
    imag0 = sqrt(id.^2 + iq.^2 + Pm.i_eps^2);
    vdi = max(vloc, 0)/sqrt(3);
    vd = -vdi.*id./imag0; vq = -vdi.*iq./imag0;
end
dzd = en*Pm.ki_i(:).*ed.*free - (1-en)*zd*1e3;
dzq = en*Pm.ki_i(:).*eq.*free - (1-en)*zq*1e3;
did = (vd - R.*id + we.*L.*iq)./L;
diq = (vq - R.*iq - we.*L.*id - we.*lam)./L;
tm = kt.*iq;
pm = tm.*wm;
eff = eta.^tanh(pm/2);
tau = N.*tm.*eff - fric;
pen = max(q - Pm.q_hi(:), 0) + min(q - Pm.q_lo(:), 0);
tau = tau - Pm.k_stop*pen - Pm.c_stop*qd.*min(1, abs(pen)/1e-3);
pl = double(t < Pm.t_release);
Mp = pl*Mt + (1 - pl)*(M0 + diag(Pm.J_refl(:)));
qdd = Mp \ (tau - pl*(vp + g) - (1 - pl)*(vp0 + g0));
imag = sqrt(id.^2 + iq.^2);
pe = 1.5*(vd.*id + vq.*iq);
pdc = pe + 1.5*imag.^2*Pm.rds + en*3*vloc.*imag*(2/pi)*Pm.t_sw*Pm.fsw + Pm.p_board;
idc = pdc./max(vloc, 10);
dx = [did; diq; dzd; dzq; en*(qr - q)];
lg = [t; q; qd; qr; iq; iq_ref; tau; pdc; vloc];
end
