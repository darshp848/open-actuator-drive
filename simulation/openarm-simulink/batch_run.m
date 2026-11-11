function result_dir = batch_run(names, stop_override)
% Runs a previously built model. Each invocation writes to a new results folder.
% Incomplete runs are retained for diagnosis and stop the batch with an error.
if nargin < 2, stop_override = 0; end
here = fileparts(mfilename('fullpath')); cd(here);
result_dir = fullfile(here,'results',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(result_dir);
fprintf('Results: %s\n',result_dir);
for k = 1:numel(names)
    try
        R = run_openarm(string(names{k}), stop_time=stop_override);
    catch err
        fprintf(2, '%s FAILED: %s\n', names{k}, getReport(err, 'extended')); rethrow(err)
    end
    save(fullfile(result_dir, string(names{k}) + ".mat"), 'R');
    fprintf('%s wall %.1f s\n', names{k}, R.wall_s); disp(R.metrics); disp(R.screens);
    assert(R.validation.completed, 'OpenArm:IncompleteRun', ...
        'Incomplete run retained at %s. Remaining cases were not run.',result_dir);
    assert(R.validation.warning_free, 'OpenArm:SolverWarnings', ...
        'Run retained at %s. Review solver warnings before running more cases.',result_dir);
    plot_run(string(names{k}),result_dir);
end
end
