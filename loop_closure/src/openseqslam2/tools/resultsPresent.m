function [present, err] = resultsPresent(path)
    % Determines if valid results exist in the provided directory
    err = [];

    % Do some tests
    exists = exist(path) == 7;
    batchConfig = exist(fullfile(path, 'batch_config.xml'));
    batchResults = exist(fullfile(path, 'batch_results.mat'));
    batch = batchConfig && batchResults;
    config = exist(fullfile(path, 'config.xml'));
    preprocessed = exist(fullfile(path, 'preprocessed.mat'));
    diffMatrix = exist(fullfile(path, 'diff_matrix.mat'));
    matching = exist(fullfile(path, 'matching.mat'));
    individual = config && preprocessed && diffMatrix && matching;

    % Make a decision on what the result is
    if ~exists
        err = ['The folder ''' path ''' does not exist']
    elseif ~batch && ~individual
        if ~config
            err = 'The file ''config.xml'' could not be found';
        elseif ~preprocessed
            err = 'The file ''preprocessed.mat'' could not be found';
        elseif ~diffMatrix
            err = 'The file ''diff_matrx.mat'' could not be found';
        elseif ~matching
            err = 'The file ''matching.mat'' could not be found';
        elseif ~batchConfig
            err = 'The file ''batch_config.xml'' could not be found';
        elseif ~batchResults
            err = 'The file ''batch_results.xml'' could not be found';
        end
    end
    present = isempty(err);
end
