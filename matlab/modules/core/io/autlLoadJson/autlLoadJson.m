function data = autlLoadJson(path)
% autlLoadJson
% Read JSON file into MATLAB struct.

if ~isfile(path)
    error('autlLoadJson:FileNotFound', 'JSON file not found: %s', path);
end

txt = fileread(path);
data = jsondecode(txt);
end
