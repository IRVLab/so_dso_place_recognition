function l = datasetImageList(datasetConfig, numbers)
    % Get all relevant information for the dataset
    path = datasetConfig.path;
    info = datasetConfig.(datasetConfig.type);
    isVideo = strcmp(datasetConfig.type, 'video');

    % Populate the list
    l = cell(length(numbers),1);
    if isVideo
        l = arrayfun( ...
            @(x) datasetFrameInfo(numbers(x), info.frame_rate, 1, ...
            path, x), ...
            1:length(numbers), 'UniformOutput', false);
    else
        l = arrayfun( ...
            @(x) datasetPictureInfo(path, info.token_start, ...
            info.token_end, numbers(x), info.numbers(end), 1, x), ...
            1:length(numbers), 'UniformOutput', false);
    end
end
