build_openarm_system();
R = run_openarm("multi_fast", stop_time=1.0);
fprintf('t_end %.4f wall %.0f s err "%s"\n', R.t(end), R.wall_s, extractBefore(R.sim_error + newline, newline));
disp(R.metrics)
