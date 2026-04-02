function report = autlGenerateFlowStopReport(session_dir)
% AUTLGENERATEFLOWSTOPREPORT Build a summary of where data-flow stopped.
%
%   report = autlGenerateFlowStopReport(session_dir)

report = struct();
report.ok = false;
report.session_dir = char(string(session_dir));
report.total_events = 0;
report.worker_last_stage = struct();
report.worker_last_ts = struct();
report.error_events = struct([]);
report.report_path = '';

if nargin < 1 || strlength(string(session_dir)) == 0 || ~isfolder(session_dir)
    return;
end

flow_dir = fullfile(session_dir, 'flow_logs');
if ~isfolder(flow_dir)
    return;
end

flow_files = dir(fullfile(flow_dir, '*.jsonl'));
if isempty(flow_files)
    return;
end

events = struct([]);
for i = 1:numel(flow_files)
    fpath = fullfile(flow_dir, flow_files(i).name);
    lines = autlReadAllLines(fpath);
    for li = 1:numel(lines)
        line = strtrim(lines{li});
        if strlength(string(line)) == 0
            continue;
        end
        try
            ev = jsondecode(line);
            if ~isfield(ev, 'module') || ~isfield(ev, 'stage')
                continue;
            end
            events(end+1).ts = autlGetFieldOr(ev, 'ts', ''); %#ok<AGROW>
            events(end).module = autlGetFieldOr(ev, 'module', 'unknown_module');
            events(end).stage = autlGetFieldOr(ev, 'stage', 'unknown_stage');
            if isfield(ev, 'payload')
                events(end).payload = ev.payload;
            else
                events(end).payload = struct();
            end
        catch
            % skip malformed line
        end
    end
end

if isempty(events)
    return;
end

report.total_events = numel(events);

worker_keys = {};
worker_stage = {};
worker_ts = {};
error_events = struct([]);

for i = 1:numel(events)
    ev = events(i);

    wkey = autlExtractWorkerKey(ev);
    if strlength(string(wkey)) > 0
        idx = find(strcmp(worker_keys, wkey), 1);
        if isempty(idx)
            worker_keys{end+1} = wkey; %#ok<AGROW>
            worker_stage{end+1} = char(string(ev.stage)); %#ok<AGROW>
            worker_ts{end+1} = char(string(ev.ts)); %#ok<AGROW>
        else
            worker_stage{idx} = char(string(ev.stage));
            worker_ts{idx} = char(string(ev.ts));
        end
    end

    stage_s = lower(char(string(ev.stage)));
    if contains(stage_s, 'fail') || contains(stage_s, 'error') || contains(stage_s, 'timeout')
        e = struct();
        e.ts = char(string(ev.ts));
        e.module = char(string(ev.module));
        e.stage = char(string(ev.stage));
        e.worker = char(string(wkey));
        e.payload = ev.payload;
        error_events = [error_events; e]; %#ok<AGROW>
    end
end

for i = 1:numel(worker_keys)
    key = matlab.lang.makeValidName(worker_keys{i});
    report.worker_last_stage.(key) = worker_stage{i};
    report.worker_last_ts.(key) = worker_ts{i};
end
report.error_events = error_events;

report_path = fullfile(flow_dir, 'flow_stop_report.txt');
fid = fopen(report_path, 'w');
if fid >= 0
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'AutoLanding Flow Stop Report\n');
    fprintf(fid, 'Session: %s\n', char(string(session_dir)));
    fprintf(fid, 'Generated: %s\n', char(datetime('now')));
    fprintf(fid, 'Total events: %d\n\n', report.total_events);

    fprintf(fid, '[Last stage by worker]\n');
    for i = 1:numel(worker_keys)
        fprintf(fid, '- %s: %s @ %s\n', worker_keys{i}, worker_stage{i}, worker_ts{i});
    end
    fprintf(fid, '\n');

    fprintf(fid, '[Recent error/timeout/fail events]\n');
    max_err = min(40, numel(error_events));
    if max_err == 0
        fprintf(fid, '- none\n');
    else
        start_idx = numel(error_events) - max_err + 1;
        for i = start_idx:numel(error_events)
            e = error_events(i);
            fprintf(fid, '- %s | %s | %s | worker=%s\n', e.ts, e.module, e.stage, e.worker);
        end
    end
end

report.ok = true;
report.report_path = report_path;
end

function lines = autlReadAllLines(file_path)
lines = {};
try
    fid = fopen(file_path, 'r');
    if fid < 0
        return;
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    tline = fgetl(fid);
    while ischar(tline)
        lines{end+1} = tline; %#ok<AGROW>
        tline = fgetl(fid);
    end
catch
    lines = {};
end
end

function out = autlGetFieldOr(s, field_name, fallback)
out = fallback;
if isstruct(s) && isfield(s, field_name)
    out = s.(field_name);
end
end

function key = autlExtractWorkerKey(ev)
key = '';
try
    if isfield(ev, 'payload') && isstruct(ev.payload)
        p = ev.payload;
        if isfield(p, 'worker_id')
            key = sprintf('worker_%d', double(p.worker_id));
            return;
        end
        if isfield(p, 'worker')
            key = char(string(p.worker));
            return;
        end
    end

    m = regexp(char(string(ev.module)), 'worker[_\s-]?(\d+)', 'tokens', 'once');
    if ~isempty(m)
        key = sprintf('worker_%s', m{1});
    end
catch
    key = '';
end
end
