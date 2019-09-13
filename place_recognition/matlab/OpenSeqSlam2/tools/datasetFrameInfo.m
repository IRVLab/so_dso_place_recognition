function out = datasetFrameInfo(frame_num, frame_rate, verbose, varargin)
    % Optional arguments for verbose mode:
    % path, index in dataset

    % Get the time of the frame
    frame_time = frame_num / frame_rate;

    % Return the appropriate version
    if verbose
        % Verbose description
        [p n e] = fileparts(varargin{1});
        out = ['Frame #' num2str(varargin{2}) ', @' num2str(frame_time) ...
            's (...' filesep() n e ')'];
    else
        % Frame time
        out = frame_time;
    end
end
