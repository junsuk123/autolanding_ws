function cfg = autlLoadOrchestrationConfig(configPath)
% autlLoadOrchestrationConfig
% Load the workspace orchestration YAML into a flat MATLAB struct.

cfg = struct();

if nargin < 1 || strlength(string(configPath)) == 0 || ~isfile(configPath)
    return;
end

lines = regexp(fileread(configPath), '\r\n|\r|\n', 'split');
for i = 1:numel(lines)
    line = strtrim(lines{i});
    if isempty(line) || startsWith(line, '#') || startsWith(line, '%')
        continue;
    end

    tokens = regexp(line, '^([A-Za-z0-9_]+):\s*(.*)$', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end

    key = char(tokens{1});
    raw_value = strtrim(tokens{2});
    cfg.(key) = localParseYamlScalar(raw_value);
end
end

function value = localParseYamlScalar(raw_value)
if isempty(raw_value)
    value = '';
    return;
end

if startsWith(raw_value, '[') && endsWith(raw_value, ']')
    inner = strtrim(raw_value(2:end-1));
    if isempty(inner)
        value = [];
        return;
    end

    parts = split(inner, ',');
    parsed = cell(1, numel(parts));
    for idx = 1:numel(parts)
        parsed{idx} = localParseYamlScalar(strtrim(parts{idx}));
    end
    if all(cellfun(@(x) isnumeric(x) && isscalar(x), parsed))
        value = cell2mat(parsed);
    else
        value = parsed;
    end
    return;
end

if (startsWith(raw_value, '"') && endsWith(raw_value, '"')) || (startsWith(raw_value, '''') && endsWith(raw_value, ''''))
    value = raw_value(2:end-1);
    return;
end

lower_value = lower(raw_value);
switch lower_value
    case {'true', 'yes', 'on'}
        value = true;
        return;
    case {'false', 'no', 'off'}
        value = false;
        return;
    case {'null', '~'}
        value = [];
        return;
end

num_value = str2double(raw_value);
if isfinite(num_value) && ~isempty(regexp(raw_value, '^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$', 'once'))
    value = num_value;
    return;
end

value = raw_value;
end