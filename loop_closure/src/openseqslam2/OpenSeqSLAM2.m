function [results, config] = OpenSeqSLAM2(varargin)
    results = []; config = [];

    % Add the toolbox to the path
    run(fullfile(fileparts(which('OpenSeqSLAM2')), 'tools', 'toolboxInit'));

    % Create the configuration (either graphically, or through a provided *.xml
    % configuration file). By the time this block below is completed, we have
    % a configuration that has all derived data, and has passed validation.
    if isempty(varargin)
        config = OpenSeqSLAMSetup();
        if isempty(config)
            % warning('No configuration was selected in the GUI. Aborting.');
            return;
        end
    else
        config = xml2settings(varargin{1});
        if isempty(config)
            error(['Loading configuration from ''' varargin{1} ''' failed.']);
        end
        derRef = SetupGUI.deriveDatasetData(config.reference.path);
        if isempty(derRef)
            error(['Failed to derive data from the reference dataset. ' ...
                'Check configuraiton in GUI for further details.'])
        end
        derQuery = SetupGUI.deriveDatasetData(config.query.path);
        if isempty(derQuery)
            error(['Failed to derive data from the query dataset. ' ...
                'Check configuraiton in GUI for further details.'])
        end
        config = SetupGUI.mergeWithDerived(config, derRef, derQuery);
        if config.ground_truth.exists
            [gt, err] = SetupGUI.loadGroundTruthMatrix(config);
            if ~isempty(err)
                error(['Failed to load requested ground truth matrix: ' err]);
            end
            config.ground_truth.matrix = gt;
        end
        err = evaluateConfig(config);
        if ~isempty(err)
            error(['Config failed validation: ' err]);
        end
    end

    % Decide parameters that have configuration dependent defaults
    % (there currently are none, and it should stay that way)

    % Run the SeqSLAM process, and get the results
    results = OpenSeqSLAMRun(config);

    % Run the appropriate results exploration UI if requested
    if config.ui.results
        [results, config] = OpenSeqSLAMResults(results, config);
    end
end
