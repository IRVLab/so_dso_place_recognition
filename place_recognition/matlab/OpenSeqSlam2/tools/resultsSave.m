function err = resultsSave(path, data, fname)
    % The only possible error is not an existing directory
    if exist(path) ~= 7
        err = ['The folder ''' path ''' does not exist']
        return;
    end

    % Save differently based on the specified fname
    if strcmp(fname, 'config.xml')
        settings2xml(data, fullfile(path, fname));
    elseif strcmp(fname, 'preprocessed.mat')
        preprocessed = data;
        save(fullfile(path, fname), 'preprocessed');
    elseif strcmp(fname, 'diff_matrix.mat')
        diffMatrix = data;
        save(fullfile(path, fname), 'diffMatrix');
    elseif strcmp(fname, 'matching.mat')
        matching = data;
        save(fullfile(path, fname), 'matching');
    elseif strcmp(fname, 'precision_recall.mat')
        pr = data;
        save(fullfile(path, fname), 'pr');
    elseif strcmp(fname, 'batch_results.mat')
        results = data;
        save(fullfile(path, fname), 'results');
    elseif strcmp(fname, 'batch_config.xml')
        settings2xml(data, fullfile(path, fname));
    else
        err = ['There is not supprted saving method for filename ''' fname '''.'];
    end
end
