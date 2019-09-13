function out = datasetPictureInfo(path, token_start, token_end, image_num, ...
        num_max, verbose, varargin)
    % Optional arguments for verbose mode:
    % index in dataset

    % Start by getting the image number string
    image_num = num2str(image_num, ['%0' num2str(numel(num2str(num_max))) 'd']);

    % Get the image name
    image_name = [token_start image_num token_end];

    % Return the appropriate version
    if verbose
        % Verbose description
        out = ['Image #' num2str(varargin{1}) ' (..' filesep() image_name ')'];
    else
        % Full path
        out = fullfile(path, image_name);
    end
end
