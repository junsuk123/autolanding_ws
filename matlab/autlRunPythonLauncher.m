function did_handle = autlRunPythonLauncher(varargin)
% autlRunPythonLauncher
% MATLAB compatibility shim for older entrypoints.

did_handle = true;

try
    AutoLandingMainFull(varargin{:});
catch ME
    did_handle = false;
    error('[autlRunPythonLauncher] MATLAB launcher failed: %s', ME.message);
end
end