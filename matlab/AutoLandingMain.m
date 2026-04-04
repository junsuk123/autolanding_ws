function AutoLandingMain(varargin)
% AUTOLANDINGMAIN
% MATLAB compatibility entrypoint.
% All orchestration is handled by scripts/autolanding_launcher.py.

if autlRunPythonLauncher(varargin{:})
    return;
end

error('[AutoLandingMain] Python launcher not found. Expected scripts/autolanding_launcher.py');
end
