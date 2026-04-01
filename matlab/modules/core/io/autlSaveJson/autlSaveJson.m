function autlSaveJson(path, data)
% autlSaveJson
% Write MATLAB data as pretty JSON.

outDir = fileparts(path);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

txt = jsonencode(data, PrettyPrint=true);
fid = fopen(path, 'w');
if fid < 0
    error('autlSaveJson:OpenFailed', 'Cannot open file: %s', path);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', txt);
end
