function [results, config, err] = resultsOpen(path)
    results = [];
    config = [];
    err = [];

    % Check the results are present
    [present, err] = resultsPresent(path);
    if ~isempty(err)
        return;
    end

    % Attempt to load the config
    config = xml2settings(fullfile(path, 'config.xml'));
    if isempty(config)
        err = 'Failed to load the config struct from ''config.xml''.';
        return;
    end

    % Attempt to load each of the results sections individually
    load(fullfile(path, 'preprocessed.mat'));
    if isempty(preprocessed)
        err = ...
            'Failed to load preproccessing database from ''preprocessed.mat''.';
        return;
    end
    load(fullfile(path, 'diff_matrix.mat'));
    if isempty(diffMatrix)
        err = ...
            'Failed to load difference matrix from ''diff_matrix.mat''.';
        return;
    end
    load(fullfile(path, 'matching.mat'));
    if isempty(matching)
        err = ...
            'Failed to load matching database from ''matching.mat''.';
        return;
    end
    load(fullfile(path, 'precision_recall.mat'));
    if isempty(pr)
        err = ...
            'Failed to load matching database from ''precision_recall.mat''.';
        return;
    end

    % We have loaded everything successfully if we have made it here... combine
    % results and return
    results.preprocessed = preprocessed;
    results.diff_matrix = diffMatrix;
    results.matching = matching;
    results.pr = pr;
end
