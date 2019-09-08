function cleanDir(path)
    if exist(path) == 7 && ~rmdir(path, 's')
        error(['Removing directory ''' path ...
            ''' failed.']);
    end
    if exist(path) ~= 7 && ~mkdir(path)
        error(['Making directory ''' path ...
            ''' failed.']);
    end
end
