function report = check_openarm_setup()
% Validate saved model inputs and run-integrity regression cases without solving.
here = fileparts(mfilename('fullpath'));
assert(strcmp(pwd,here),'OpenArm:WorkingFolder','Run from the system_model folder.');
report.matlab_version = string(version);
if ~isempty(which('library.settingsLookup'))
    report.library_settings = library.settingsLookup();
    assert(~report.library_settings.found || report.library_settings.gatePass, ...
        'OpenArm:LibraryConfiguration','Custom library configuration requires setup.');
else
    report.library_settings = struct('toolkit_available',false);
end
report.licenses = [license('test','Simulink') license('test','Simscape') ...
    license('test','Robotics_System_Toolbox')];
assert(all(report.licenses),'OpenArm:Licenses','Required toolbox license is unavailable.');
files = {'run_openarm.m','batch_run.m','openarm_run_integrity.m','check_openarm_setup.m','plot_run.m'};
for k = 1:numel(files)
    issues = checkcode(fullfile(here,files{k}),'-id');
    report.code_analysis.(erase(files{k},'.m')) = issues;
end
names = ["drop6","multi_fast","estop_coast","drop6_auxfail","drop6_nobrake","release6"];
for k = 1:numel(names)
    prep = run_openarm(names(k),validate_only=true);
    report.inputs.(names(k)) = prep.validated;
end
charts = sfroot().find('-isa','Stateflow.EMChart');
for file = ["ctrl_fcn.m","supervisor_fcn.m"]
    source = strtrim(strrep(fileread(fullfile(here,file)),char(13),''));
    matched = false;
    for chart = charts'
        if startsWith(chart.Path,'openarm_system/') && ...
                strcmp(strtrim(strrep(chart.Script,char(13),'')),source)
            matched = true;
        end
    end
    assert(matched,'OpenArm:StaleChart','Saved SLX code differs from %s. Rebuild as a separate stage.',file);
end
report.embedded_scripts_match_source = true;
blocks = find_system('openarm_system','SearchDepth',1,'Type','Block');
unconnected = strings(0,1);
for k = 1:numel(blocks)
    ports = get_param(blocks{k},'PortHandles');
    for port_type = ["Inport","Outport","LConn","RConn"]
        for handle = ports.(port_type)
            if get_param(handle,'Line') == -1
                unconnected(end+1) = string(blocks{k}) + ":" + port_type;
            end
        end
    end
end
report.unconnected_top_level_ports = unconnected;
assert(isempty(unconnected),'OpenArm:UnconnectedPorts','Unconnected top-level model ports.');
sample = zeros(3,68); sample(:,1) = [0;0.05;0.1];
v = openarm_run_integrity(sample,0.1,0.05,"","ReachedStopTime"); assert(v.completed);
v = openarm_run_integrity(sample,1.8,0.05,"","ReachedStopTime"); assert(~v.completed);
v = openarm_run_integrity(sample,0.1,0.05,"solver error","DiagnosticError"); assert(~v.completed);
v = openarm_run_integrity(sample,0.1,0.05,"","TimeOut"); assert(~v.completed);
bad = sample; bad(2,10) = NaN;
v = openarm_run_integrity(bad,0.1,0.05,"","ReachedStopTime"); assert(~v.completed);
bad = sample; bad(2,1) = 0;
v = openarm_run_integrity(bad,0.1,0.05,"","ReachedStopTime"); assert(~v.completed);
v = openarm_run_integrity(zeros(0,68),0.1,0.05,"","ReachedStopTime"); assert(~v.completed);
report.integrity_regressions_pass = true;
report.validated_without_solving = true;
fprintf('OpenArm setup validation passed. No simulation was run.\n');
end
