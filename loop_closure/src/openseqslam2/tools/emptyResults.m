function empty = emptyResults()
    % Creates and returns an empty config (creates all of the possible struct
    % elements)

    % Preprocessing
    empty.preprocessed.reference = [];
    empty.preprocessed.reference_numbers = [];
    empty.preprocessed.query = [];
    empty.preprocessed.query_numbers = [];

    % Difference matrices
    empty.diff_matrix.base = [];
    empty.diff_matrix.enhanced = [];

    % Matching
    empty.matching.all.min_scores = [];
    empty.matching.all.best_trajectories = [];
    empty.matching.selected.mask = [];
    empty.matching.selected.matches = [];
    empty.matching.selected.trajectories = [];

    % Precision recall data
    empty.pr.matching_method = [];
    empty.pr.sweep_var.start = [];
    empty.pr.sweep_var.end = [];
    empty.pr.sweep_var.num_steps = [];

    empty.pr.values.precisions = [];
    empty.pr.values.recalls = [];
end
