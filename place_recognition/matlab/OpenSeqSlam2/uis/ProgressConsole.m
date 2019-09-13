classdef ProgressConsole < handle

    properties
        config = emptyConfig();
        progress;

        lastPercentRefresh = 0;
        newState = false;

        instance;

        results = emptyResults();

        quietMode;
    end

    methods
        function obj = ProgressConsole(config, varargin)
            % Save the config
            obj.config = config;

            % Create an initial progress state
            obj.progress = [];
            obj.progress.state = SeqSLAMInstance.STATE_START;
            obj.progress.percent = 0;

            % Turn on quiet mode if provided
            if ~isempty(varargin) && varargin{1}
                obj.quietMode = true;
            else
                obj.quietMode = false;
            end

            % Create, and attach to, the SeqSLAM instance
            obj.instance = SeqSLAMInstance(config);
            obj.instance.attachUI(obj);
        end

        function due = refreshPercentDue(obj, state, perc)
            rate = obj.config.ui.progress.percent_freq;
            obj.newState = state ~= obj.progress.state;
            if obj.newState
                obj.progress.state = state;
            end
            due = obj.newState || ...
                floor(perc / rate) > floor(obj.lastPercentRefresh / rate);
        end

        function refreshPercent(obj, percent)
            % Move to the next state if a change was detected
            if obj.newState
                obj.newState = false;
                if ~obj.quietMode
                    ProgressConsole.printStateText(obj.progress.state);
                else
                    % Need this arbitrary print in here to make MATLAB actually
                    % flush the print to the console when in parallel for loops
                    % (otherwise it doesn't print anything until the end of the
                    % loop iteration.... stupid stupid MATLAB...)
                    fprintf('\b');
                end
            end

            % Update the cached percent values
            obj.progress.percent = percent;
            obj.lastPercentRefresh = percent;

            % Print the new percent values
            if obj.progress.state ~= SeqSLAMInstance.STATE_DONE && ...
                    ~obj.quietMode
                fprintf('\b\b\b\b%s%%', ...
                    pad(num2str(floor(percent)), 3, 'left'));
            end
        end

        function run(obj)
            obj.instance.run();
            obj.results = obj.instance.results;
        end
    end

    methods (Static, Access = private)
        function printStateText(state)
            if state == SeqSLAMInstance.STATE_PREPROCESS_REF
                fprintf('\n\tPreprocessing reference images:\t.....   0%%');
            elseif state == SeqSLAMInstance.STATE_PREPROCESS_QUERY
                fprintf('\n\tPreprocessing query images:\t.....   0%%');
            elseif state == SeqSLAMInstance.STATE_DIFF_MATRIX
                fprintf('\n\tConstructing difference matrix:\t.....   0%%');
            elseif state == SeqSLAMInstance.STATE_DIFF_MATRIX_CONTRAST
                fprintf('\n\tEnhancing difference matrix:\t.....   0%%');
            elseif state == SeqSLAMInstance.STATE_MATCHING
                fprintf('\n\tSearching for match candidates:\t.....   0%%');
            elseif state == SeqSLAMInstance.STATE_MATCHING_FILTERING
                fprintf('\n\tFiltering match list:\t\t.....   0%%');
            elseif state == SeqSLAMInstance.STATE_DONE
                fprintf('\n');
            end
        end
    end
end
