function autlFlowLog(flow_log_file, module_name, stage_name, payload)
% AUTLFLOWLOG Append a single JSON line event for data-flow tracing.
%
%   autlFlowLog(flow_log_file, module_name, stage_name, payload)

if nargin < 4 || isempty(payload)
    payload = struct();
end
if nargin < 3 || strlength(string(stage_name)) == 0
    stage_name = 'unspecified';
end
if nargin < 2 || strlength(string(module_name)) == 0
    module_name = 'unknown_module';
end
if nargin < 1 || strlength(string(flow_log_file)) == 0
    return;
end

try
    log_dir = fileparts(flow_log_file);
    if strlength(string(log_dir)) > 0 && ~isfolder(log_dir)
        mkdir(log_dir);
    end

    event = struct();
    event.ts = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSS'));
    event.epoch_s = posixtime(datetime('now'));
    event.module = char(string(module_name));
    event.stage = char(string(stage_name));
    event.payload = payload;

    fid = fopen(flow_log_file, 'a');
    if fid < 0
        return;
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, '%s\n', jsonencode(event));
catch
    % Do not interrupt mission flow due to debug logging failures.
end
end
