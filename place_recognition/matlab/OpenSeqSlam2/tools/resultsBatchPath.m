function path = resultsBatchPath(config, iterationNum)
    fs = strsplit(config.batch.param, '.');
    path = fullfile(config.results.path, [fs{end} '-' ...
        strrep(num2str(config.batch.values(iterationNum)), '.', '-')]);
end
