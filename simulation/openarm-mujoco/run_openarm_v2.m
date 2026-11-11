function [result, result_dir] = run_openarm_v2()
% Validate and run the pinned stock-servo OpenArm v2 numerical diagnostics.
here = fileparts(mfilename('fullpath'));
python_exe = fullfile(fileparts(here),'.venv-openarm','Scripts','python.exe');
runner = fullfile(here,'batch_stage.py');
assert(isfile(python_exe),'OpenArm:PythonMissing','Pinned .venv-openarm environment is missing.');
result_dir = fullfile(here,'results',['matlab_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
parts = string({python_exe,runner,result_dir});
assert(~any(contains(parts,["""","%","&","|","<",">","^",newline,char(13)]),'all'), ...
    'OpenArm:ShellPath','Project path contains shell metacharacters.');
result = struct('passed',false,'hardware_accuracy_qualified',false);
for stage = ["validate","motion","contact","render"]
    command = sprintf('"%s" "%s" --stage %s --output "%s"',python_exe,runner,stage,result_dir);
    [status, output] = system(command,'MATLAB_SHELL','cmd');
    if isfolder(result_dir)
        fid = fopen(fullfile(result_dir,stage + "-process.log"),'w');
        assert(fid >= 0,'OpenArm:LogFailed','Could not open process log.');
        fprintf(fid,'%s',output);
        fclose(fid);
    end
    assert(status == 0,'OpenArm:StageFailed','OpenArm %s failed: %s',stage,output);
    report = jsondecode(fileread(fullfile(result_dir,stage + "-result.json")));
    assert(report.passed,'OpenArm:ValidationFailed','Stage %s did not pass.',stage);
    result.(stage) = report;
end
result.passed = true;
result.matlab_version = version;
fid = fopen(fullfile(result_dir,'result.json'),'w');
assert(fid >= 0,'OpenArm:ReportFailed','Could not open final report.');
fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
fclose(fid);
fprintf('OpenArm v2 numerical diagnostics passed. Hardware accuracy remains unqualified.\nEvidence: %s\n',result_dir);
end
