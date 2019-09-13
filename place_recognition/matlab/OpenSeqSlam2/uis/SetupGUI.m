classdef SetupGUI < handle

    properties (Access = private, Constant)
        % Sizing parameters
        FIG_WIDTH_FACTOR = 10;      % Times largest button on bottom row
        FIG_HEIGHT_FACTOR = 26;     % Times height of buttons at font size
    end

    properties
        hFig;

        hTitle;
        hInfo;

        hHelp;

        hPrevResults;

        hConfigImport;
        hConfigExport;

        hRef;
        hRefLocation;
        hRefPicker;
        hRefStatus;
        hRefSample;
        hRefSampleValue;

        hQuery;
        hQueryLocation;
        hQueryPicker;
        hQueryStatus;
        hQuerySample;
        hQuerySampleValue;

        hResults;
        hResultsLocation;
        hResultsPicker;
        hResultsStatus;

        hConfigure;
        hStart;

        config = emptyConfig();

        cachedReference;
        cachedQuery;

        done = false;
    end

    methods
        function obj = SetupGUI()
            % Create and size the GUI
            obj.createGUI();
            obj.sizeGUI();

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);
            HelpPopup.setDestination(obj.hHelp, 'configuration');
            SpecPosition.positionRelative(obj.hHelp, obj.hConfigImport, ...
                SpecPosition.CENTER_Y);

            % Finally, show the figure when we are done configuring
            obj.hFig.Visible = 'on';

            % Samples popup
            obj.samplesPopup();
        end

        function dumpConfigToXML(obj, filename)
            % Strip the GUI's config, and dump to XML
            obj.strip();
            settings2xml(obj.config, filename);
        end

        function success = loadConfigFromXML(obj, filename)
            % Extract the struct from the file
            s = xml2settings(filename);
            if isempty(s)
                success = false;
                return;
            end

            % Save the struct as the config, and populate the GUI
            obj.config = s;
            obj.populate();
            success = true;
        end
    end

    methods (Static)
        function [data, status, col] = deriveDatasetData(path)
            data = []; status = []; col = 'k';

            % Takes a path, and derives all of related dataset data
            [p, n, e] = fileparts(path);
            if ~exist(path, 'file')
                % Inform that the path does not point to an existing file
                status = 'File does not exist!';
                col = GUISettings.COL_ERROR;
            elseif isdir(path)
                % Attempt to profile the requested image dataset
                [numbers, ext startToken, endToken] = ...
                    datasetPictureProfile(path);

                % Report the results
                if length(numbers) == 0 || isempty(ext)
                    status = ['No dataset was found (' ...
                        'a filename patterns wasn''t identified)!'];
                    col = GUISettings.COL_ERROR;
                else
                    status = ['Dataset with filenames ''' ...
                        startToken '[' num2str(numbers(1), ...
                        ['%0' num2str(numel(num2str(numbers(end)))) 'd']) ...
                        '-' num2str(numbers(end)) ']' endToken ...
                        ''' identified!'];
                    col = GUISettings.COL_SUCCESS;

                    % Save the derived data
                    data = [];
                    data.type = 'image';
                    data.image.ext = ext;
                    data.image.numbers = numbers;
                    data.image.token_start = startToken;
                    data.image.token_end = endToken;
                end
            elseif any(ismember({VideoReader.getFileFormats().Extension}, ...
                    e(2:end)))
                % Attempt to read the video
                v = VideoReader(path);
                if v.Duration > 0
                    status= ['Video read (' ...
                        int2str(v.Duration/60) 'm ' ...
                        num2str(round(mod(v.Duration,60)), '%02d') 's)!'];
                    col = GUISettings.COL_SUCCESS;

                    % Save the results
                    data = [];
                    data.type = 'video';
                    data.video.ext = e(2:end);
                    data.video.frames = floor(v.Duration * v.FrameRate);
                    data.video.frame_rate = v.FrameRate;
                else
                    status = ['An empty video ' ...
                        '(duration of 0 seconds) was read!'];
                    col = GUISettings.COL_ERROR;
                end
            else
                % Unsupported video format
                status = ['''*' e ...
                    ''' is an unsupported video format'];
                col = GUISettings.COL_ERROR;
            end
        end

        function merged = mergeWithDerived(config, derivedRef, derivedQuery)
            % TODO: this is feral.....
            merged = config;
            merged.reference = derivedRef;
            merged.query = derivedQuery;
            merged.reference.path = config.reference.path;
            merged.query.path = config.query.path;
            merged.reference.subsample_factor = ...
                config.reference.subsample_factor;
            merged.query.subsample_factor = ...
                config.query.subsample_factor;
        end

        function [gt, err] = loadGroundTruthMatrix(config)
            [gt, err] = GroundTruthPopup.getGtMatrix(config.ground_truth, ...
                [length(SeqSLAMInstance.numbers(config.query)), ...
                length(SeqSLAMInstance.numbers(config.reference))]);
        end
    end

    methods (Access = private)
        function cbClose(obj, src, event)
            HelpPopup.requestClose();
            delete(obj.hFig);
        end

        function cbChooseDataset(obj, src, event, edit, status)
            obj.interactivity(false);

            % Select the format of the dataset
            % TODO Dialog is flakey as hell (menubar sometimes overlays...)
            choice = questdlg(...
                'Is the dataset a collection of images or a video?', ...
                'Dataset Format?', 'Images', 'Video', 'Cancel', 'Cancel');

            % Select the dataset
            dataSet = '';
            if (strcmp(choice,'Images'))
                % Choose the directory where the images reside
                dataSet = uigetdir('', ...
                    'Select the directory containing the dataset images');
            elseif (strcmp(choice,'Video'))
                % Choose the video file
                [f, p] = uigetfile( ...
                    VideoReader.getFileFormats().getFilterSpec(), ...
                    'Select a supported video file as the dataset');
                dataSet = fullfile(p, f);
            end

            % Display the dataset selection (and manually trigger evaluation)
            if (ischar(dataSet) || isstr(dataSet)) && exist(dataSet, 'file')
                edit.String = dataSet;
                obj.evaluateDataset(edit.String, status);
            end

            obj.interactivity(true);
        end

        function cbChooseResults(obj, src, event, edit, status)
            obj.interactivity(false);

            % Select the results directory
            resultsDir = uigetdir('', ...
                'Select the directory to store / access results');

            % Display the selected directory (and manually trigger evaluation)
            if (ischar(resultsDir) || isstr(resultsDir)) && ...
                    exist(resultsDir, 'file')
                edit.String = resultsDir;
                obj.cbEvaluateResults(edit, event, status);
            end

            obj.interactivity(true);
        end

        function cbEvaluateDataset(obj, src, event, status)
            obj.evaluateDataset(src.String, status);

            % Update the state of the SeqSLAM settings and start buttons
            obj.updateButtons();
        end

        function cbEvaluateResults(obj, src, event, status)
            obj.evaluateResults(src.String, status);

            % Update the state of the SeqSLAM settings and start buttons
            obj.updateButtons();
        end

        function cbImport(obj, src, event)
            % Prompt the user to select and XML file
            [f, p] = uigetfile('*.xml', 'Select an XML configuration file', ...
                fullfile(toolboxRoot(), '.config'));
            if isnumeric(f) || isnumeric(p)
                uiwait(msgbox( ...
                    ['No file was selected, ' ...
                    'no configuration was loaded'], ...
                    'No file selected', 'modal'));
                return;
            end
            xmlfile = fullfile(p, f);
            [p, f, e] = fileparts(xmlfile);
            if ~strcmpi(e, '.xml')
                uiwait(msgbox( ...
                    ['No *.xml file was selected, ' ...
                    'no configuration was loaded'], ...
                    'No *.xml file selected', 'modal'));
                return;
            end

            % Attempt to load the settings
            s = obj.loadConfigFromXML(xmlfile);
            if ~s
                uiwait(errordlg( ...
                    ['Import failed, ' ...
                    'are you sure this is a valid config file?'], ...
                    'Import from *.xml file failed', 'modal'));
                return;
            end
        end

        function cbExport(obj, src, event)
            % Prompt user to select where they'd like to export
            [f, p] = uiputfile('*.xml', 'Select export location', ...
                fullfile(toolboxRoot(), '.config'));
            if isnumeric(f) || isnumeric(p)
                uiwait(msgbox( ...
                    ['No save location was selected, ' ...
                    'configuration was not exported'], ...
                    'No save location selected', 'modal'));
                return;
            end

            % Save the settings
            obj.dumpConfigToXML(fullfile(p, f));
        end

        function cbOpenResults(obj, src, event)
            obj.interactivity(false);

            % Request a results directory from the user
            resultsDir = uigetdir('', ...
                'Select the directory containing the saved results');

            % Check the directory for existing results (bail if there are none)
            [present, err] = resultsPresent(resultsDir);

            % Attempt to open the results
            [results, config, err] = resultsOpen(resultsDir);
            if ~isempty(err)
                uiwait(errordlg( ...
                    ['Opening of results failed due to an error (' err ')'], ...
                    'Failed to open results', 'modal'));
            else
                % Open up the results in the ResultsGUI, and wait until done
                resultsui = ResultsGUI(results, config);
                uiwait(resultsui.hFig);
                HelpPopup.setDestination(obj.hHelp, 'configuration');
            end

            obj.interactivity(true);
        end

        function cbConfigure(obj, src, event)
            obj.interactivity(false);

            % Open the GUI, populate it, and wait for the user to finish
            obj.strip();
            seqslamgui = ConfigureGUI(obj.config);
            uiwait(seqslamgui.hFig);
            HelpPopup.setDestination(obj.hHelp, 'configuration');

            % Save the parameters returned
            obj.config = seqslamgui.config;

            obj.interactivity(true);
        end

        function cbStart(obj, src, event)
            % Store a temp config for if we fail
            configTemp = obj.config;

            % Extract all of the data from the UI, including the derived data
            obj.strip();

            % Load the ground truth in here (even reload so we can check it
            % still passes size checks...)
            if obj.config.ground_truth.exists
                [gt, err] = SetupGUI.loadGroundTruthMatrix(obj.config);
                if ~isempty(err)
                    errordlg(err, 'Ground Truth opening failed', 'modal');
                    return;
                else
                    obj.config.ground_truth.matrix = gt;
                end
            end

            % Evaluate the validity of the config
            err = evaluateConfig(obj.config);
            if ~isempty(err)
                errordlg({['The requested configuration is not valid due to '...
                    'the following error:'], '', err}, ...
                    'Invalid configuration', 'modal');
                obj.config = configTemp; % Revert to unmodified config
                return;
            end

            % Report that the figure naturally finished, and then close
            obj.done = true;
            delete(obj.hFig);
        end

        function createGUI(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'OpenSeqSLAM2.0 Configuration';
            obj.hFig.Resize = 'off';

            % Title annotation
            obj.hTitle = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hTitle);
            obj.hTitle.HorizontalAlignment = 'center';
            obj.hTitle.String = '\textbf{Welcome to \textit{OpenSeqSLAM2.0}!}';

            obj.hInfo = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hInfo);
            obj.hInfo.String = ['Press \textit{Start} to ' ...
                'begin immediately, or use this window to pick datasets, ' ...
                'configurations, and adjust settings.'];

            % Button for opening previous results
            obj.hPrevResults = uicontrol('Style', 'pushbutton');
            obj.hPrevResults.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hPrevResults);
            obj.hPrevResults.String = 'Open previous results';

            % Buttons for exporting and importing parameters
            obj.hConfigImport = uicontrol('Style', 'pushbutton');
            GUISettings.applyUIControlStyle(obj.hConfigImport);
            obj.hConfigImport.String = 'Import config';

            obj.hConfigExport = uicontrol('Style', 'pushbutton');
            GUISettings.applyUIControlStyle(obj.hConfigExport);
            obj.hConfigExport.String = 'Export config';

            % Reference dataset elements (title, path edit, file select
            % button, selection status)
            obj.hRef = uipanel();
            GUISettings.applyUIPanelStyle(obj.hRef);
            obj.hRef.Title = 'Reference Dataset Location';

            obj.hRefLocation = uicontrol('Style', 'edit');
            obj.hRefLocation.Parent = obj.hRef;
            GUISettings.applyUIControlStyle(obj.hRefLocation);
            obj.hRefLocation.String = '';

            obj.hRefPicker = uicontrol('Style', 'pushbutton');
            obj.hRefPicker.Parent = obj.hRef;
            GUISettings.applyUIControlStyle(obj.hRefPicker);
            obj.hRefPicker.String = '...';

            obj.hRefStatus = uicontrol('Style', 'text');
            obj.hRefStatus.Parent = obj.hRef;
            GUISettings.applyUIControlStyle(obj.hRefStatus);
            obj.hRefStatus.FontAngle = 'italic';
            obj.hRefStatus.HorizontalAlignment = 'right';
            obj.hRefStatus.String = '';

            obj.hRefSample = uicontrol('Style', 'text');
            obj.hRefSample.Parent = obj.hRef;
            GUISettings.applyUIControlStyle(obj.hRefSample);
            obj.hRefSample.String = 'Subsample Factor:';

            obj.hRefSampleValue = uicontrol('Style', 'edit');
            obj.hRefSampleValue.Parent = obj.hRef;
            GUISettings.applyUIControlStyle(obj.hRefSampleValue);
            obj.hRefSampleValue.String = '';

            % Query dataset elements (title, path edit, file select
            % button, selection status)
            obj.hQuery = uipanel();
            GUISettings.applyUIPanelStyle(obj.hQuery);
            obj.hQuery.Title = 'Query Dataset Location';

            obj.hQueryLocation = uicontrol('Style', 'edit');
            obj.hQueryLocation.Parent = obj.hQuery;
            GUISettings.applyUIControlStyle(obj.hQueryLocation);
            obj.hQueryLocation.String = '';

            obj.hQueryPicker = uicontrol('Style', 'pushbutton');
            obj.hQueryPicker.Parent = obj.hQuery;
            GUISettings.applyUIControlStyle(obj.hQueryPicker);
            obj.hQueryPicker.String = '...';

            obj.hQueryStatus = uicontrol('Style', 'text');
            obj.hQueryStatus.Parent = obj.hQuery;
            GUISettings.applyUIControlStyle(obj.hQueryStatus);
            obj.hQueryStatus.FontAngle = 'italic';
            obj.hQueryStatus.HorizontalAlignment = 'right';
            obj.hQueryStatus.String = '';

            obj.hQuerySample = uicontrol('Style', 'text');
            obj.hQuerySample.Parent = obj.hQuery;
            GUISettings.applyUIControlStyle(obj.hQuerySample);
            obj.hQuerySample.String = 'Subsample Factor:';

            obj.hQuerySampleValue = uicontrol('Style', 'edit');
            obj.hQuerySampleValue.Parent = obj.hQuery;
            GUISettings.applyUIControlStyle(obj.hQuerySampleValue);
            obj.hQuerySampleValue.String = '';

            % Results elements (title, path edit, file select button,
            % selection status)
            obj.hResults = uipanel();
            GUISettings.applyUIPanelStyle(obj.hResults);
            obj.hResults.Title = 'Results Save Location';

            obj.hResultsLocation = uicontrol('Style', 'edit');
            obj.hResultsLocation.Parent = obj.hResults;
            GUISettings.applyUIControlStyle(obj.hResultsLocation);
            obj.hResultsLocation.String = '';

            obj.hResultsPicker = uicontrol('Style', 'pushbutton');
            obj.hResultsPicker.Parent = obj.hResults;
            GUISettings.applyUIControlStyle(obj.hResultsPicker);
            obj.hResultsPicker.String = '...';

            obj.hResultsStatus = uicontrol('Style', 'text');
            obj.hResultsStatus.Parent = obj.hResults;
            GUISettings.applyUIControlStyle(obj.hResultsStatus);
            obj.hResultsStatus.FontAngle = 'italic';
            obj.hResultsStatus.HorizontalAlignment = 'right';
            obj.hResultsStatus.String = '';

            % Configure button
            obj.hConfigure = uicontrol('Style', 'pushbutton');
            GUISettings.applyUIControlStyle(obj.hConfigure);
            obj.hConfigure.String = 'Configure';

            % Start button
            obj.hStart = uicontrol('Style', 'pushbutton');
            GUISettings.applyUIControlStyle(obj.hStart);
            obj.hStart.String = 'Start';

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hPrevResults.Callback = {@obj.cbOpenResults};
            obj.hConfigImport.Callback = {@obj.cbImport};
            obj.hConfigExport.Callback = {@obj.cbExport};
            obj.hRefLocation.Callback = {@obj.cbEvaluateDataset, ...
                obj.hRefStatus};
            obj.hRefPicker.Callback = {@obj.cbChooseDataset, ...
                obj.hRefLocation, obj.hRefStatus};
            obj.hQueryLocation.Callback = {@obj.cbEvaluateDataset, ...
                obj.hQueryStatus};
            obj.hQueryPicker.Callback = {@obj.cbChooseDataset, ...
                obj.hQueryLocation, obj.hQueryStatus};
            obj.hResultsLocation.Callback = {@obj.cbEvaluateResults, ...
                obj.hResultsStatus};
            obj.hResultsPicker.Callback = {@obj.cbChooseResults, ...
                obj.hResultsLocation, obj.hResultsStatus};
            obj.hConfigure.Callback = {@obj.cbConfigure};
            obj.hStart.Callback= {@obj.cbStart};
            obj.hFig.CloseRequestFcn = {@obj.cbClose};
        end

        function evaluateDataset(obj, path, status)
            obj.interactivity(false); status.Enable = 'on';

            % Perform validation (and save the extension as a record of what
            % the validation found)
            status.String = 'Validating...';
            status.ForegroundColor = GUISettings.COL_LOADING;
            drawnow();
            [data, str, col] = SetupGUI.deriveDatasetData(path);
            status.String = str;
            status.ForegroundColor = col;
            if status == obj.hRefStatus
                obj.cachedReference = data;
            elseif status == obj.hQueryStatus
                obj.cachedQuery = data;
            end

            obj.interactivity(true);
        end

        function evaluateResults(obj, path, status)
            obj.interactivity(false); status.Enable = 'on';

            % Perform evaluation
            status.String = 'Validating...';
            status.ForegroundColor = GUISettings.COL_LOADING;
            drawnow();
            obj.interactivity(true);
            if isempty(path)
                % Results will not be saved
                status.String = ['No location selected - ' ...
                    'results will not be saved automatically'];
                status.ForegroundColor = GUISettings.COL_WARNING;
            elseif ~exist(path, 'file')
                % Inform that the path does not point to an existing directory
                status.String = 'Directory does not exist!';
                status.ForegroundColor = GUISettings.COL_ERROR;
            elseif resultsPresent(path)
                % Results directory selected with existing results
                status.String = ['Directory contains ' ...
                    'previous results which will be overwritten'];
                status.ForegroundColor = GUISettings.COL_WARNING;
            else
                % A new results directory was selected
                status.String = ['Selected directory contains ' ...
                    'no existing results'];
                status.ForegroundColor = GUISettings.COL_SUCCESS;
            end

            obj.interactivity(true);
        end

        function interactivity(obj, enable)
            if enable
                status = 'on';
            else
                status = 'off';
            end

            obj.hHelp.Enable = status;

            obj.hPrevResults.Enable = status;

            obj.hConfigImport.Enable = status;
            obj.hConfigExport.Enable = status;

            obj.hRefLocation.Enable = status;
            obj.hRefPicker.Enable = status;
            obj.hRefStatus.Enable = status;
            obj.hRefSampleValue.Enable = status;

            obj.hQueryLocation.Enable = status;
            obj.hQueryPicker.Enable = status;
            obj.hQueryStatus.Enable = status;
            obj.hQuerySampleValue.Enable = status;

            obj.hResultsLocation.Enable = status;
            obj.hResultsPicker.Enable = status;
            obj.hResultsStatus.Enable = status;

            obj.hConfigure.Enable = status;
            obj.hStart.Enable = status;
        end

        function populate(obj)
            % Dump all data from the config struct into the UI
            obj.hRefLocation.String = obj.config.reference.path;
            obj.hRefSampleValue.String = num2str( ...
                SafeData.noEmpty(obj.config.reference.subsample_factor, 1));

            obj.hQueryLocation.String = obj.config.query.path;
            obj.hQuerySampleValue.String = num2str( ...
                SafeData.noEmpty(obj.config.query.subsample_factor, 1));

            obj.hResultsLocation.String = obj.config.results.path;

            % Run any data validation methods after populating
            obj.evaluateDataset(obj.hRefLocation.String, obj.hRefStatus);
            obj.evaluateDataset(obj.hQueryLocation.String, obj.hQueryStatus);
            obj.evaluateResults(obj.hResultsLocation.String, obj.hResultsStatus);
        end

        function samplesPopup(obj)
            % Assume the samples have been unzipped if the directory exists,
            % or do not unzip if do not ask again has been clicked
            if exist(fullfile(toolboxRoot(), 'datasets', 'samples')) == 7 || ...
                    exist(fullfile(toolboxRoot(), '.persistent', 'nounzip'))
                return;
            end

            % Ask the user if they would like to unzip the samples
            resp = questdlg(['It appears the samples provided with the ' ...
                'toolbox have not yet been unzipped. ' ...
                'Would you like to unzip them?'], 'Unzip samples?', ...
                'Yes', 'No', 'Do not ask again', 'Yes');
            if strcmpi(resp, 'yes')
                h = waitbar(0.5, 'Unzipping...');
                unzip( ...
                    fullfile(toolboxRoot(), 'datasets', 'samples.zip'), ...
                    fullfile(toolboxRoot(), 'datasets'));
                close(h);
            elseif strcmpi(resp, 'do not ask again')
                fclose(fopen( ...
                    fullfile(toolboxRoot(), '.persistent', 'nounzip'), 'w+'));
            end
        end

        function sizeGUI(obj)
            % Get some reference dimensions (max width of 3 buttons, and
            % default height of a button)
            widthUnit = obj.hConfigure.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hStart.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * SetupGUI.FIG_WIDTH_FACTOR, ...
                heightUnit * SetupGUI.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hTitle, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 1.0);
            SpecSize.size(obj.hInfo, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 1.0);

            SpecSize.size(obj.hPrevResults, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.35);
            SpecSize.size(obj.hConfigImport, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hConfigExport, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);

            SpecSize.size(obj.hRef, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hRef, SpecSize.HEIGHT, ...
                SpecSize.ABSOLUTE, 6*heightUnit);
            SpecSize.size(obj.hRefLocation, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hRef, 0.9, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hRefPicker, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hRef, 0.1, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hRefStatus, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hRef, GUISettings.PAD_MED);
            SpecSize.size(obj.hRefSample, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_SMALL);

            SpecSize.size(obj.hQuery, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hQuery, SpecSize.HEIGHT, ...
                SpecSize.ABSOLUTE, 6*heightUnit);
            SpecSize.size(obj.hQueryLocation, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hQuery, 0.9, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hQueryPicker, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hQuery, 0.1, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hQueryStatus, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hQuery, GUISettings.PAD_MED);
            SpecSize.size(obj.hQuerySample, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_SMALL);

            SpecSize.size(obj.hResults, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hResults, SpecSize.HEIGHT, ...
                SpecSize.ABSOLUTE, 4*heightUnit);
            SpecSize.size(obj.hResultsLocation, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hResults, 0.9, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hResultsPicker, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hResults, 0.1, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hResultsStatus, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hResults, GUISettings.PAD_MED);

            SpecSize.size(obj.hConfigure, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hStart, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.2);

            % Then, systematically place
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.TOP);
            SpecPosition.positionIn(obj.hInfo, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hInfo, obj.hTitle, ...
                SpecPosition.BELOW, GUISettings.PAD_SMALL);

            SpecPosition.positionIn(obj.hConfigImport, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hConfigImport, obj.hInfo, ...
                SpecPosition.BELOW, 1.5*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hConfigExport, ...
                obj.hConfigImport, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hConfigExport, ...
                obj.hConfigImport, SpecPosition.CENTER_Y);

            SpecPosition.positionIn(obj.hRef, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hRef, obj.hConfigImport, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hRefLocation, obj.hRef, ...
                SpecPosition.TOP, 1.5*heightUnit);
            SpecPosition.positionIn(obj.hRefLocation, obj.hRef, ...
                SpecPosition.LEFT, GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hRefPicker, ...
                obj.hRefLocation, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hRefPicker, obj.hRef, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hRefStatus, obj.hRef, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hRefStatus, ...
                obj.hRefPicker, SpecPosition.BELOW, 0.5*heightUnit);
            SpecPosition.positionRelative(obj.hRefSample, ...
                obj.hRefStatus, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hRefSample, obj.hRef, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hRefSampleValue, ...
                obj.hRefSample, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hRefSampleValue, ...
                obj.hRefSample, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hQuery, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hQuery, obj.hRef, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hQueryLocation, obj.hQuery, ...
                SpecPosition.TOP, 1.5*heightUnit);
            SpecPosition.positionIn(obj.hQueryLocation, obj.hQuery, ...
                SpecPosition.LEFT, GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hQueryPicker, ...
                obj.hQueryLocation, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hQueryPicker, obj.hQuery, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hQueryStatus, obj.hQuery, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hQueryStatus, ...
                obj.hQueryPicker, SpecPosition.BELOW, 0.5*heightUnit);
            SpecPosition.positionRelative(obj.hQuerySample, ...
                obj.hQueryStatus, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hQuerySample, obj.hQuery, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hQuerySampleValue, ...
                obj.hQuerySample, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hQuerySampleValue, ...
                obj.hQuerySample, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hResults, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hResults, obj.hQuery, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hResultsLocation, obj.hResults, ...
                SpecPosition.TOP, 1.5*heightUnit);
            SpecPosition.positionIn(obj.hResultsLocation, obj.hResults, ...
                SpecPosition.LEFT, GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hResultsPicker, ...
                obj.hResultsLocation, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hResultsPicker, obj.hResults, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hResultsStatus, obj.hResults, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hResultsStatus, ...
                obj.hResultsPicker, SpecPosition.BELOW, 0.5*heightUnit);

            SpecPosition.positionIn(obj.hConfigure, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hConfigure, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hPrevResults, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hPrevResults, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hStart, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStart, ...
                obj.hConfigure, SpecPosition.CENTER_Y);
        end

        function strip(obj)
            % Strip data from the UI, and store it over the top
            obj.config.reference.path = obj.hRefLocation.String;
            obj.config.reference.subsample_factor = ...
                str2num(obj.hRefSampleValue.String);

            obj.config.query.path = obj.hQueryLocation.String;
            obj.config.query.subsample_factor = ...
                str2num(obj.hQuerySampleValue.String);

            obj.config.results.path = obj.hResultsLocation.String;

            % Merge the data with the cached data (it is assumed that the
            % deriving operation has already been performed...)
            obj.config = SetupGUI.mergeWithDerived(obj.config, ...
                obj.cachedReference, obj.cachedQuery);
        end

        function updateButtons(obj)
            if isequal(obj.hRefStatus.ForegroundColor, ...
                    GUISettings.COL_ERROR) || isequal( ...
                    obj.hQueryStatus.ForegroundColor, GUISettings.COL_ERROR) ...
                    || isequal(obj.hResultsStatus.ForegroundColor, ...
                    GUISettings.COL_ERROR)
                obj.hConfigure.Enable = 'off';
                obj.hStart.Enable = 'off';
            else
                obj.hConfigure.Enable = 'on';
                obj.hStart.Enable = 'on';
            end
        end
    end
end
