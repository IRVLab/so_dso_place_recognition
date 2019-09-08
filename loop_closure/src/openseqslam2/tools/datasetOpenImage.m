function img = datasetOpenImage(ds_config, index, numbers, varargin)
    % varargin can be used to supply an existing video reader object
    if length(varargin) > 1 || (length(varargin) == 1 && ...
            ~isempty(varargin{1}) && ~strcmp('VideoReader', class(varargin{1})))
        error('Invalid: Extra argument must be a ''VideoReader'' instance.');
    end
    if ~isempty(varargin)
        v = varargin{1};
    else
        v = [];
    end

    % Use one of two options, depending on whether the dataset is from a video
    % or a collection of images
    if strcmp('video', ds_config.type)
        % Use a video reader to open the requested frame
        if isempty(v)
            v = VideoReader(ds_config.path);
        end
        v.CurrentTime = datasetFrameInfo(numbers(index)-1, v.FrameRate, 0);
        img = v.readFrame();
    else
        % Open the relevant image file
        info = ds_config.(ds_config.type);
        img = imread(datasetPictureInfo(ds_config.path, info.token_start, ...
            info.token_end, numbers(index), info.numbers(end), 0));
    end
end
