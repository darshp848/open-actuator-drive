function [result, result_dir] = run_mujoco_probe()
% Run a headless MuJoCo integration check through the existing Python venv.
% This is a synthetic contact smoke test, not OpenArm or physical validation.
here = fileparts(mfilename('fullpath'));
python_exe = fullfile(fileparts(here),'.venv','Scripts','python.exe');
probe = fullfile(here,'matlab_mujoco_probe.py');
assert(isfile(python_exe),'MuJoCo:PythonMissing','Existing project Python environment is missing.');
result_dir = fullfile(here,'results',['matlab_probe_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
parts = string({python_exe,probe,result_dir});
assert(~any(contains(parts,["""","%","&","|","<",">","^",newline,char(13)]),'all'), ...
    'MuJoCo:ShellPath','Project path contains shell metacharacters.');
for stage = ["validate","solve"]
    command = sprintf('"%s" "%s" --stage %s --output "%s"',python_exe,probe,stage,result_dir);
    [status, output] = system(command,'MATLAB_SHELL','cmd');
    assert(status == 0,'MuJoCo:ProbeFailed','MuJoCo %s failed: %s',stage,output);
end
result = jsondecode(fileread(fullfile(result_dir,'result.json')));
assert(result.passed && abs(result.actual_duration_s - result.requested_duration_s) < 1e-10, ...
    'MuJoCo:IncompleteRun','Probe did not complete the requested interval.');
fprintf('MuJoCo integration passed, %.3f simulated seconds, %.3f wall seconds.\n', ...
    result.actual_duration_s,result.wall_s);
fprintf('Evidence: %s\n',result_dir);
end
