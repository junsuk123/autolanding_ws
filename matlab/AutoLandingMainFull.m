function AutoLandingMainFull(varargin)
% AUTOLANDINGMAINFULL
% MATLAB compatibility entrypoint.
% All orchestration is handled by scripts/autolanding_launcher.py.

if autlRunPythonLauncher(varargin{:})
    return;
end

error('[AutoLandingMainFull] Python launcher not found. Expected scripts/autolanding_launcher.py');
end
