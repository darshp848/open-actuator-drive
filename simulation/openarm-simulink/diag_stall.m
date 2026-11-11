% Diagnose the multi_fast solver stall near 0.70 s (fidelity review, 2026-11-02). Writes results/diag_stall.log/.mat.
here = fileparts(mfilename('fullpath')); cd(here);
build_openarm_system();
R = run_openarm("multi_fast", stop_time=0.72);
k = R.t > 0.60;
fprintf('t_end %.5f wall %.0f s err "%s"\n', R.t(end), R.wall_s, extractBefore(R.sim_error + newline, newline));
fprintf('vbus %.3f-%.3f V, vbat %.3f V, chopper on %.3f, hs_off %d\n', min(R.v_bus(k)), max(R.v_bus(k)), mean(R.v_bat(k)), mean(R.chop(k)), sum(R.hs_off(k)));
fprintf('qd/wmax at end: %s\n', mat2str(R.qd(end,:)./R.P.w_max, 3));
fprintf('iq end: %s\niq_ref end: %s\n', mat2str(R.iq(end,:), 3), mat2str(R.iq_ref(end,:), 3));
fprintf('pdc end: %s (sum %.1f W)\n', mat2str(R.pdc(end,:), 3), sum(R.pdc(end,:)));
pen = max(R.q(end,:) - R.P.q_hi, 0) + min(R.q(end,:) - R.P.q_lo, 0);
fprintf('hard-stop penetration end (rad): %s\n', mat2str(pen, 3));
vmax = R.P.v_frac*R.vloc(end,:);
fprintf('board phase-voltage limit end: %s\n', mat2str(vmax, 3));
save(fullfile('results','diag_stall.mat'), 'R');
