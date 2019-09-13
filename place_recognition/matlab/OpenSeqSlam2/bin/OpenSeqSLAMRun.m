function results = OpenSeqSLAMRun(config)
    PROGRESS_MODES = {'graphical', 'console'};

    % Construct a list of configurations to be run (there will only be one if
    % we aren't running in batch mode)
    if config.batch.enabled
        for k = 1:length(config.batch.values)
            configs(k) = config;
            configs(k) = setDeepField(configs(k), config.batch.param, ...
                config.batch.values(k));
            configs(k).results.path = resultsBatchPath(config, k);
        end

        % Clean out the root results directory
        cleanDir(config.results.path);
    else
        configs = [config];
    end

    % Create a UI for the batch mode if necessary
    if config.batch.enabled && ...
            strcmp(config.ui.progress.type, PROGRESS_MODES{1})
        batchui = BatchPopup(length(configs));
    else
        batchui = [];
    end

    % Start running through each of the jobs (we need a different path of
    % execution for the parallel option)
    % TODO make this more succint...
    if config.batch.enabled && config.batch.parallelise
        fprintf('Setting up the parallel pool...\n');
        if isempty(gcp('nocreate'))
            parpool(feature('numcores'));
        end
        fprintf('Done.\nParellel SeqSLAM instances will now begin:\n');
        parfor k = 1:length(configs)
            % Use the quiet mode for console progress
            progress = ProgressConsole(configs(k), true);

            % Only print the surrounding strings, and run
            fprintf('%s\n', consoleString(true, config.batch.param, ...
                config.batch.values(k)));
            progress.run();
            fprintf('%s\n', consoleString(false, config.batch.param, ...
                config.batch.values(k)));

            % Extract the results, auto-optimising if required
            if ~isempty(config.ground_truth.use_to_auto_optimise)
                progress.results.matching.selected = optimalMatch( ...
                    progress.results.matching.all, ...
                    config.seqslam.matching, config.ground_truth.matrix, ...
                    config.ground_truth.use_to_auto_optimise);
            end
            rs(k) = progress.results;
        end
    else
        for k = 1:length(configs)
            % Execute the job by wrapping it in a progress handler (assuming
            % graphical by default)
            if strcmp(config.ui.progress.type, PROGRESS_MODES{2})
                % Console mode
                progress = ProgressConsole(configs(k));

                % Run with some cute strings surrounding
                if config.batch.enabled
                    fprintf(consoleString(true, config.batch.param, ...
                        config.batch.values(k)));
                else
                    fprintf(consoleString(true));
                end
                progress.run();
                if config.batch.enabled
                    fprintf('%s\n', consoleString(false, ...
                        config.batch.param, config.batch.values(k)));
                else
                    fprintf('%s\n', consoleString(false));
                end
            else
                % Graphical mode
                progress = ProgressGUI(configs(k));

                % Update the batch UI if necessary
                if ~isempty(batchui)
                    batchui.updateJob( ...
                        {config.batch.param, config.batch.values(k)}, k);
                    batchui.hFig.WindowStyle = 'modal';
                end

                % Run and store the results
                progress.run();
            end

            % Extract the results, auto-optimising if required
            if ~isempty(config.ground_truth.use_to_auto_optimise)
                progress.results.matching.selected = optimalMatch( ...
                    progress.results.matching.all, ...
                    config.seqslam.matching, config.ground_truth.matrix, ...
                    config.ground_truth.use_to_auto_optimise);
            end
            rs(k) = progress.results;
        end
    end

    % Post process the results as required
    results = [];
    if config.batch.enabled
        % Include full results only if requested
        if ~config.batch.trim_results
            results.tests = rs;
        else
            results.tests = [];
        end

        % Add in all details necessary for precision recall
        results.batch_param = config.batch.param;
        results.batch_values = config.batch.values;
        results.ground_truth = config.ground_truth;
        for k = 1:length(rs)
            [results.precisions(k), results.recalls(k)] = calcPR( ...
                rs(k).matching.selected.matches, results.ground_truth.matrix);
        end

        % Save the batch results and config (they are not saved anywhere earlier)
        resultsSave(config.results.path, config, 'batch_config.xml');
        resultsSave(config.results.path, results, 'batch_results.mat');
    else
        results = rs(1);
    end

    % Close the batch mode UI if necessary
    if config.batch.enabled && ...
            strcmp(config.ui.progress.type, PROGRESS_MODES{1})
        close(batchui.hFig);
    end
end
