function V = openarm_run_integrity(log_data, requested_stop, log_period, sim_error, stop_event)
% Check completion separately from physical performance of an OpenArm run.
arguments
    log_data (:,68) double
    requested_stop (1,1) double {mustBePositive,mustBeFinite}
    log_period (1,1) double {mustBePositive,mustBeFinite}
    sim_error (1,1) string
    stop_event (1,1) string
end
V.finite_monotonic = size(log_data,1) >= 2 && all(isfinite(log_data),'all') && ...
    all(diff(log_data(:,1)) > 0);
V.completed = V.finite_monotonic && strlength(sim_error) == 0 && stop_event == "ReachedStopTime" && ...
    abs(log_data(1,1)) <= 1e-9 && ...
    abs(log_data(end,1) - requested_stop) <= log_period*1.01;
V.requested_stop_s = requested_stop;
V.stop_event = stop_event;
if isempty(log_data), V.last_log_s = NaN; else, V.last_log_s = log_data(end,1); end
end
