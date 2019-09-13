% Force the directory to change
oldDir = pwd;
cd(fileparts(which('toolboxInit')));

% Only do anything if the toolbox is NOT on the path
if isempty(which('OpenSeqSLAM2'))
    % Use the root directory to add to the path
    root = toolboxRoot();
    addpath(genpath(root));

    % Explicitly remove any unwanted directories
    rmpath(genpath(fullfile(root, '.git')));
    rmpath(genpath(fullfile(root, '.config')));
    rmpath(genpath(fullfile(root, 'datasets')));
    rmpath(genpath(fullfile(root, 'results')));
end

% Return the directory
cd(oldDir);
