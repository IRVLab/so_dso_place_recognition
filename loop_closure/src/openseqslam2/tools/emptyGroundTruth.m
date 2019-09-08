function empty = emptyGroundTruth()
    % Creates and returns an empty config (creates all of the possible struct
    % elements)

    % Ground truth data
    empty.exists = []; % true or false
    empty.matrix = [];
    empty.type = []; % velocity, csv, or mat
    empty.use_to_auto_optimise = []; % empty, p, r, f1
    empty.velocity.vel = [];
    empty.velocity.tol = [];
    empty.file.path = [];
    empty.file.var = [];
end
