function empty = emptyConfig()
    % Creates and returns an empty config (creates all of the possible struct
    % elements)

    % Reference dataset settings
    empty.reference.path = [];
    empty.reference.subsample_factor = [];

    % Query dataset settings
    empty.query.path = [];
    empty.query.subsample_factor = [];

    % Results settings
    empty.results.path = [];

    % SeqSLAM settings (image processing)
    empty.seqslam.image_processing.downsample.width = [];
    empty.seqslam.image_processing.downsample.height = [];
    empty.seqslam.image_processing.downsample.method = [];

    empty.seqslam.image_processing.crop.reference = [];
    empty.seqslam.image_processing.crop.query = [];

    empty.seqslam.image_processing.normalisation.threshold = [];
    empty.seqslam.image_processing.normalisation.strength = [];

    % SeqSLAM settings (difference matrix)
    empty.seqslam.diff_matrix.contrast.r_window = [];

    % SeqSLAM settings (search)
    empty.seqslam.search.d_s = [];
    empty.seqslam.search.v_min = [];
    empty.seqslam.search.v_max = [];
    empty.seqslam.search.method = []; % traj, cone, hybrid
    empty.seqslam.search.method_traj.v_step = [];

    % SeqSLAM settings (matching)
    empty.seqslam.matching.method = []; % window, thresh
    empty.seqslam.matching.method_window.r_window = [];
    empty.seqslam.matching.method_window.u = [];
    empty.seqslam.matching.method_thresh.threshold = [];

    % Ground truth settings
    empty.ground_truth = emptyGroundTruth();

    % Batch mode settings
    empty.batch.enabled = []; % true or false
    empty.batch.param = [];
    empty.batch.values = [];
    empty.batch.parallelise = []; % true or false
    empty.batch.trim_results = []; % true or false

    % User intergace settings
    empty.ui.progress.type = []; % graphical, console
    empty.ui.results = []; % true or false
    empty.ui.progress.percent_freq = [];
    empty.ui.progress.preprocess_freq = [];
    empty.ui.progress.diff_matrix_freq = [];
    empty.ui.progress.enhance_freq = [];
    empty.ui.progress.match_freq = [];

    % NOTE: These are derived and should never need to be set manually!!!!
    % --- START DERIVED ---
    empty.reference.type = []; % image, video
    empty.reference.image.ext = [];
    empty.reference.image.numbers = [];
    empty.reference.image.token_start = [];
    empty.reference.image.token_end = [];
    empty.reference.video.ext = [];
    empty.reference.video.frames = [];

    empty.query.type = []; % image, video
    empty.query.image.ext = [];
    empty.query.image.numbers = [];
    empty.query.image.token_start = [];
    empty.query.image.token_end = [];
    empty.query.video.ext = [];
    empty.query.video.frames = [];
    empty.query.video.frame_rate = [];
    % --- END DERIVED ---

end
