function err = evaluateConfig(config)
    % NOTE: we do not check any of the derived parameters. A deriving failure
    % should be verbalised by the deriving process...
    err = [];

    OPTS_DATASET_TYPE = {'image', 'video'};
    OPTS_DOWNSAMPLE_METHOD = {'lanczos3'};
    OPTS_SEARCH_METHOD = {'traj', 'cone', 'hybrid'};
    OPTS_MATCH_METHOD = {'window', 'thresh'};
    OPTS_PROGRESS = {'graphical', 'console'};
    OPTS_GROUND_TRUTH = {'velocity', 'csv', 'mat'};
    OPTS_AUTO_OPTIMISE = {'p', 'r', 'f1'};

    % Perform all independent evaluation checks
    if ~checkProperties(config.reference.path, 'ex')
        err = 'Reference dataset filepath must exist';
    elseif ~checkProperties(config.reference.subsample_factor, 'enps')
        err = 'Reference supsample factor must be a positive scalar';
    elseif ~checkProperties(config.reference.type, 'L', OPTS_DATASET_TYPE)
        err = ['Reference dataset type is invalid ' ...
            '(it probably failed to successfully open)'];
    elseif ~checkProperties(config.query.path, 'ex')
        err = 'Query dataset filepath must exist';
    elseif ~checkProperties(config.query.subsample_factor, 'enps')
        err = 'Query supsample factor must be a positive scalar';
    elseif ~checkProperties(config.query.type, 'L', OPTS_DATASET_TYPE)
        err = ['Query dataset type is invalid ' ...
            '(it probably failed to successfully open)'];
    elseif ~checkProperties( ...
            config.seqslam.image_processing.downsample.width, 'enpsi')
        err = 'Image downsample width must be a positive integer';
    elseif ~checkProperties( ...
            config.seqslam.image_processing.downsample.height, 'enpsi')
        err = 'Image downsample height must be a positive integer';
    elseif ~checkProperties( ...
            config.seqslam.image_processing.downsample.method, 'L', ...
            OPTS_DOWNSAMPLE_METHOD)
        err = 'Image downsample method must be valid for imresize';
    elseif ~isempty(config.seqslam.image_processing.crop.reference) && ...
            ~checkProperties( ...
            config.seqslam.image_processing.crop.reference, 'Vip', 4)
        err = ['Reference image crop, if provided, must be an integer ' ...
            'vector with length 4'];
    elseif ~isempty(config.seqslam.image_processing.crop.query) && ...
            ~checkProperties(config.seqslam.image_processing.crop.query, ...
            'Vip', 4)
        err = ['Query image crop, if provided, must be an integer ' ...
            'vector of length 4'];
    elseif ~checkProperties( ...
            config.seqslam.image_processing.normalisation.threshold, ...
            'espgl', 0, 1)
        err = 'Normalisation threshold must be between 0 and 1';
    elseif ~checkProperties( ...
            config.seqslam.image_processing.normalisation.strength, ...
            'espgl', 0, 1)
        err = 'Normalisation strength must be between 0 and 1';
    elseif ~checkProperties(config.seqslam.diff_matrix.contrast.r_window, ...
            'espg', 2)
        err = ['Diff matrix normalisation window must be an integer ' ...
            'greater than or equal to 2'];
    elseif ~checkProperties(config.seqslam.search.d_s, 'espig', 1)
        err = 'Search sequence length must be an integer greater than 1';
    elseif ~checkProperties(config.seqslam.search.v_min, 'esp')
        err = 'Minimum search velocity must be a positive value';
    elseif ~checkProperties(config.seqslam.search.v_max, 'esp')
        err = 'Maximum search velocity must be a positive value';
    elseif ~checkProperties(config.seqslam.search.method, 'L', ...
            OPTS_SEARCH_METHOD)
        err = 'Search method must be either traj, hybrid, or cone';
    elseif checkProperties(config.seqslam.search.method, 'L', ...
            OPTS_SEARCH_METHOD) && ~checkProperties( ...
            config.seqslam.search.method_traj.v_step, 'esp')
        err = 'Trajectory search velocity step must be a positive value';
    elseif ~checkProperties(config.seqslam.matching.method, 'L', ...
            OPTS_MATCH_METHOD)
        err = 'Matching method must be either window or thresh';
    elseif checkProperties(config.seqslam.matching.method, 'L', ...
            OPTS_MATCH_METHOD(1)) && ~checkProperties( ...
            config.seqslam.matching.method_window.r_window, 'espi')
        err = 'Matching method window must be a positive integer';
    elseif checkProperties(config.seqslam.matching.method, 'L', ...
            OPTS_MATCH_METHOD(1)) && ~checkProperties( ...
            config.seqslam.matching.method_window.u, 'epsg', 1)
        err = 'Matching method uniqueness must be greater than 1';
    elseif checkProperties(config.seqslam.matching.method, 'L', ...
            OPTS_MATCH_METHOD(2)) && ~checkProperties( ...
            config.seqslam.matching.method_thresh.threshold, 'eps')
        err = 'Matching method threshold must be a positive scalar';
    elseif ~checkProperties(config.ui.progress.type, 'L', OPTS_PROGRESS)
        err = 'Progress UI must be either graphical or console';
    elseif ~checkProperties(config.ui.progress.percent_freq, 'esp')
        err = 'Progress UI percent frequency must be a positive scalar';
    elseif ~checkProperties(config.ui.progress.preprocess_freq, 'esp')
        err = 'Progress UI preprocess frequency must be a positive scalar';
    elseif ~checkProperties(config.ui.progress.diff_matrix_freq, 'esp')
        err = ['Progress UI difference matrix frequency must ' ...
            'be a positive scalar'];
    elseif ~checkProperties(config.ui.progress.enhance_freq, 'esp')
        err = ['Progress UI difference matrix enhancement frequency ' ...
            'must be a positive scalar'];
    elseif ~checkProperties(config.ui.progress.match_freq, 'esp')
        err = 'Progress UI matching frequency must be a positive scalar';
    elseif ~checkProperties(config.ui.results, 'eb')
        err = 'Results UI choice must be a boolean value';
    elseif ~checkProperties(config.batch.enabled, 'eb')
        err = 'Batch mode selection must be a boolean value';
    elseif ~checkProperties(config.ground_truth.exists, 'eb')
        err = 'Ground truth status must be a boolean value';
    end

    if ~isempty(err)
        return;
    end

    % Perform the checks that are dependent on values
    if config.batch.enabled
        [exists, value] = findDeepField(config, config.batch.param);
        if ~exists
            err = ['The batch parameter must exist in the config (see ' ...
                '"tools/emptyConfig.m" for parameter names)'];
        elseif ~checkProperties(value, 'ns')
            err = 'The batch parameter must have a numeric scalar value';
        elseif ~checkProperties(config.batch.values, 'vn')
            err = 'The batch values must be a numeric vector';
        elseif ~checkProperties(config.batch.parallelise, 'eb')
            err = 'Batch parallelisation must be a boolean value';
        elseif config.batch.parallelise && ~checkProperties( ...
                config.ui.progress.type, 'L', OPTS_PROGRESS(2))
            err = ['Parallelisation is not compatible with graphical ' ...
                'progress mode'];
        elseif ~checkProperties(config.batch.trim_results, 'eb')
            err = 'Trimming of batch results must be a boolean value';
        elseif ~config.ground_truth.exists
            err = 'A ground truth must be provided if requesting batch mode';
        end
    end
    if ~isempty(err)
        return;
    end
    if config.ground_truth.exists
        if ~checkProperties(config.ground_truth.matrix, 'emgl', 0, 1)
            err = ['Ground truth matrix must be an array with all values ' ...
                'between 0 and 1'];
        elseif ~checkProperties(config.ground_truth.type, 'L', ...
                OPTS_GROUND_TRUTH)
            err = 'Ground truth type must be either velocity, csv, or mat';
        elseif ~isempty(config.ground_truth.use_to_auto_optimise) && ...
                ~checkProperties(config.ground_truth.use_to_auto_optimise, ...
                'L', OPTS_AUTO_OPTIMISE)
            err = ['Ground truth optimisation must be empty, or ' ...
                'either p, r, or f1'];
        elseif checkProperties(config.ground_truth.type, 'L', ...
                OPTS_GROUND_TRUTH(1)) && ~checkProperties( ...
                config.ground_truth.velocity.vel, 'enps')
            err = 'Ground truth velocity must be a positive numeric scalar';
        elseif checkProperties(config.ground_truth.type, 'L', ...
                OPTS_GROUND_TRUTH(1)) && ~checkProperties( ...
                config.ground_truth.velocity.tol, 'enpsi')
            err = 'Ground truth tolerance must be a positive scalar integer';
        elseif checkProperties(config.ground_truth.type, 'L', ...
                OPTS_GROUND_TRUTH(2:3)) && ~checkProperties( ...
                config.ground_truth.file.path, 'ex')
            err = 'Ground truth file path must exist';
        elseif checkProperties(config.ground_truth.type, 'L', ...
                OPTS_GROUND_TRUTH(2:3)) && ~checkProperties( ...
                config.ground_truth.file.var, 'ec')
            err = 'Ground truth variable name must be a string';
        end
    end

function passed = checkProperties(value, props, varargin)
    passed = true;
    ind = 0;
    argC = 0;
    while passed && ind < length(props)
        ind = ind + 1;
        x = false;  % Fail by default if invalid character provided
        if props(ind) == 'e'
            x = ~isempty(value);
        elseif props(ind) == 'x'
            x = exist(value);
        elseif props(ind) == 'c'
            x = ischar(value);
        elseif props(ind) == 'n'
            x = all(isnumeric(value(:)));
        elseif props(ind) == 'p'
            x = all(value(:) >= 0);
        elseif props(ind) == 's'
            x = isscalar(value);
        elseif props(ind) == 'i'
            x = all(mod(value(:),1) == 0);
        elseif props(ind) == 'b'
            x = all(islogical(value(:)));
        elseif props(ind) == 'L'
            argC = argC + 1;
            x = any(strcmp(value, varargin{argC}));
        elseif props(ind) == 'v'
            x = isvector(value);
        elseif props(ind) == 'V'
            argC = argC + 1;
            x = isvector(value) && length(value) == varargin{argC};
        elseif props(ind) == 'm'
            x = ismatrix(value);
        elseif props(ind) == 'g'
            argC = argC + 1;
            x = all(value(:) >= varargin{argC});
        elseif props(ind) == 'l'
            argC = argC + 1;
            x = all(value(:) <= varargin{argC});
        end
        passed = passed && x;
    end
end
end
