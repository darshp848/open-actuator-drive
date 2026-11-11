function res = profile_openarm_stall()
% Capture the known multi_fast transition in a disposable matlab -batch process.
assert(~usejava('desktop'),'OpenArm:ProfilerIsolation','Run this diagnostic with matlab -batch.');
here = fileparts(mfilename('fullpath'));
prep = run_openarm('multi_fast',stop_time=0.7041,validate_only=true);
in = prep.simulation_input;
applyToModel(in);
cleanup = onCleanup(@() close_system('openarm_system',0));
profile_dir = fullfile(here,'results',['profile_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
mkdir(profile_dir);
res = solverprofiler.profileModel('openarm_system',StopTime=0.7041, ...
    SaveStates='on',SaveSimscapeStates='on',SaveJacobian='on',SaveZCSignals='on', ...
    TimeOut=90,BufferSize=100000,OpenSP='off',DataFullFile=fullfile(profile_dir,'profile.mat'));
save(fullfile(profile_dir,'summary.mat'),'res');
disp(res.summary);
fprintf('Solver profile retained: %s\n',res.file);
end
