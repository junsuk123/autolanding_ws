function did_handle = autlRunPythonLauncher(varargin)
% autlRunPythonLauncher
% MATLAB compatibility shim that forwards AutoLanding entrypoints to Python.

did_handle = false;

rootDir = fileparts(fileparts(mfilename('fullpath')));
launcher = fullfile(rootDir, 'scripts', 'autolanding_launcher.py');
if ~exist(launcher, 'file')
    return;
end

mode = 'pipeline';
args = {};

if nargin >= 1 && ~isempty(varargin)
    first_arg = string(varargin{1});
    if strcmpi(first_arg, 'gui') || strcmpi(first_arg, 'server') || strcmpi(first_arg, 'headless')
        mode = 'sim';
        args = {'--workspace-root', rootDir, '--gui'};
        if strcmpi(first_arg, 'server') || strcmpi(first_arg, 'headless')
            args{end} = '--headless';
        end
    elseif strcmpi(first_arg, 'pipeline') || strcmpi(first_arg, 'mission') || strcmpi(first_arg, 'collect') || strcmpi(first_arg, 'collect_parallel') || strcmpi(first_arg, 'sim')
        mode = char(first_arg);
        args = {'--workspace-root', rootDir};
        if nargin > 1 && strcmpi(mode, 'collect_parallel')
            args = [args, {'--workers', num2str(varargin{2})}]; %#ok<AGROW>
            if nargin > 2
                args = [args, {'--scenarios-per-worker', num2str(varargin{3})}]; %#ok<AGROW>
            end
        end
    end
else
    args = {'--workspace-root', rootDir};
end

switch lower(mode)
    case {'pipeline', 'mission', 'sim'}
        args = [args, {'--semantic-input', fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json'), ...
            '--config', fullfile(rootDir, 'ai', 'configs', 'policy_config.yaml')}]; %#ok<AGROW>
    case 'collect'
        args = [args, {'--semantic-input', fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json'), ...
            '--config', fullfile(rootDir, 'ai', 'configs', 'policy_config.yaml')}]; %#ok<AGROW>
    case 'collect_parallel'
        args = [args, {'--semantic-input', fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json'), ...
            '--config', fullfile(rootDir, 'ai', 'configs', 'policy_config.yaml')}]; %#ok<AGROW>
end

pythonCmd = getenv('AUTOLANDING_PYTHON');
if isempty(strtrim(pythonCmd))
    pythonCmd = 'python3';
end

cmd = sprintf('%s %s %s', pythonCmd, autlQuoteShell(launcher), mode);
for idx = 1:numel(args)
    cmd = sprintf('%s %s', cmd, autlQuoteShell(args{idx}));
end

[status, output] = system(cmd);
if ~isempty(output)
    fprintf('%s\n', output);
end

if status ~= 0
    error('[autlRunPythonLauncher] Python launcher failed with exit code %d', status);
end

did_handle = true;
end

function quoted = autlQuoteShell(value)
value = strrep(char(value), '"', '\"');
quoted = sprintf('"%s"', value);
end