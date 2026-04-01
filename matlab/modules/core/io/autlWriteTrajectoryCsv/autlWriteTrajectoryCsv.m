function autlWriteTrajectoryCsv(path, trajTbl)
% autlWriteTrajectoryCsv
% Write trajectory table to CSV file.

outDir = fileparts(path);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end
writetable(trajTbl, path);
end
