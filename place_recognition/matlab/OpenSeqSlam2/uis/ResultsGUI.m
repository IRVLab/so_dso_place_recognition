classdef ResultsGUI < handle

    properties (Constant)
        SCREENS = { ...
            'Image preprocessing', ...
            'Difference Matrix', ...
            'Sequence Matches', ...
            'Precision-Recall Plotting', ...
            'Matches Video'};

        FIG_WIDTH_FACTOR = 5.5;
        FIG_HEIGHT_FACTOR = 20;
    end

    properties
        hFig;
        hHelp;

        hSaveResults;
        hScreen;

        hTitle;

        hAxA;
        hAxB;
        hAxC;
        hAxD;
        hAxMain;
        hAxPR;
        hAxVideo;

        hOpts;

        hOptsPreDataset;
        hOptsPreDatasetValue;
        hOptsPreImage;
        hOptsPreImageValue;

        hOptsDiffContr;
        hOptsDiffCol;
        hOptsDiffColValue;

        hOptsMatchDiff;
        hOptsMatchSeqs;
        hOptsMatchMatches;
        hOptsMatchSelect;
        hOptsMatchSelectValue;
        hOptsMatchTweak;

        hOptsPRGroundTruth;
        hOptsPRGroundTruthDetails;
        hOptsPRSweepVar;
        hOptsPRSweepVarValue;
        hOptsPRSweepNum;
        hOptsPRSweepNumValue;
        hOptsPRError;

        hOptsVidRate;
        hOptsVidRateValue;
        hOptsVidPlay;
        hOptsVidSlider;
        hOptsVidExport;

        hFocus;
        hFocusAx;
        hFocusButton;
        hFocusRef;
        hFocusRefAx;
        hFocusQuery;
        hFocusQueryAx;

        hFocusPR;
        hFocusPRVisualise;
        hFocusPRSlider;
        hFocusPRVal;
        hFocusPRValValue;
        hFocusPRPrecision;
        hFocusPRPrecisionValue;
        hFocusPRRecall;
        hFocusPRRecallValue;

        results = emptyResults();
        config = emptyConfig();

        listReference = {};
        listQuery = {};
        listMatches = {};

        selectedDiff = [];
        selectedMatch = [];

        currentVideoMatch = [];
        videoTimer = [];

        closeHelp = true;
    end

    methods
        function obj = ResultsGUI(results, config)
            % Create and size the GUI
            obj.createGUI();
            obj.sizeGUI();

            % Save the config and results
            obj.config = config;
            obj.results = results;

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);

            % Populate the static lists
            obj.populateDatasetLists();

            % Start on the matches screen
            obj.hScreen.Value = 3;
            obj.openScreen(obj.hScreen.Value);

            % Finally, show the figure when we are done configuring
            obj.hFig.Visible = 'on';
        end
    end

    methods (Access = private, Static)
        function next = nextMatch(current, matches)
            % Get index of matches
            inds = find(~isnan(matches));

            % Get all that are above current
            nexts = inds(inds > current(1));

            % Return the next
            if ~isempty(nexts)
                next = [nexts(1) matches(nexts(1))];
            else
                next = [];
            end
        end
    end

    methods (Access = private)
        function cbClose(obj, src, event)
            if obj.closeHelp
                HelpPopup.requestClose();
            end
            delete(obj.hFig);
        end

        function cbChangeDataset(obj, src, event)
            % Grey out all of the plots because we have a change
            obj.greyAxes();

            % Update the displayed list
            if (obj.hOptsPreDatasetValue.Value > 1)
                obj.hOptsPreImageValue.String = obj.listQuery;
            else
                obj.hOptsPreImageValue.String = obj.listReference;
            end

            % Reset the image selection to the first one
            obj.hOptsPreImageValue.Value = 1;

            % Redraw the updated value
            obj.drawPreprocessed();
        end

        function cbChangeImage(obj, src, event)
            % Grey out all of the plots because we have a change
            obj.greyAxes();

            % Redraw the updated value
            obj.drawPreprocessed();
        end

        function cbConfigureGroundTruth(obj, src, event)
            % Launch the ground truth popup (and wait until done)
            obj.interactivity(false);
            gtui = GroundTruthPopup(obj.config.ground_truth, ...
                size(obj.results.diff_matrix.enhanced));
            uiwait(gtui.hFig);
            obj.interactivity(true);

            % Update the ground truth section of the results (only if a valid
            % selection was made
            if ~isempty(gtui.gt)
                obj.config.ground_truth = gtui.gt;
            end

            % Update the precision recall plot
            obj.openScreen(obj.hScreen.Value);
        end

        function cbDiffClicked(obj, src, event)
            % Figure out which diff was clicked
            obj.selectedDiff = round(obj.hAxMain.CurrentPoint(1,1:2));

            % Redraw the diff matrix screen
            obj.drawDiffMatrix();
        end

        function cbDiffOptionChange(obj, src, event)
            obj.drawDiffMatrix();
        end

        function cbExportVideo(obj, src, event)
            % Request a save location, exiting if none is provided
            [f, p] = uiputfile('*', 'Select export location');
            if isnumeric(f) || isnumeric(p)
                uiwait(msgbox(['No save location was selected, ' ...
                    'video was not exported'], 'No save location selected', ...
                    'modal'));
                return;
            end

            % Start a waitmsg to inform the user of the progress
            numMatches = sum(~isnan(obj.results.matching.selected.mask));
            h = waitbar(0, 'Exporting... (0%');

            % Save the current state of the playback UI, and disable all
            uiMatch = obj.currentVideoMatch;
            obj.toggleVideoScreenLock(true);
            obj.hOptsVidExport.String = 'Exporting...';

            % Setup the video output file, and figure out frame sizing
            v = VideoWriter(fullfile(p, f), 'Uncompressed AVI');
            v.FrameRate = str2num(obj.hOptsVidRateValue.String);
            f = getframe(obj.hAxVideo);
            fSz = size(f.cdata);
            imSz = size(obj.hAxVideo.Children(end).CData);
            boxIm = [0.5 -0.5+obj.hAxVideo.YLim(2)-imSz(1) imSz(2) imSz(1)];
            scales = [fSz(2) fSz(1)] ./ ...
                [range(obj.hAxVideo.XLim) range(obj.hAxVideo.YLim)];
            boxFr = boxIm .* repmat(scales, 1, 2);

            % Loop through each of the matches, writing the frame to the video
            open(v);
            currentMatch = ResultsGUI.nextMatch([0 0], ...
                obj.results.matching.selected.matches);
            matchNum = 0;
            while ~isempty(currentMatch)
                obj.currentVideoMatch = currentMatch;
                obj.drawVideo();
                v.writeVideo(getframe(obj.hAxVideo, boxFr));
                currentMatch = ResultsGUI.nextMatch(currentMatch, ...
                    obj.results.matching.selected.matches);
                waitbar(matchNum / numMatches, h, ['Exporting... (' ...
                    num2str(round(matchNum/ numMatches * 100)) '%)']);
                matchNum = matchNum + 1;
            end
            close(v);
            close(h);

            % Restore the state of the playback UI, and re-enable all
            obj.currentVideoMatch = uiMatch;
            obj.toggleVideoScreenLock(false);
            obj.hOptsVidExport.String = 'Export';

            obj.drawVideo();
        end

        function cbMatchesOptionChange(obj, src, event)
            obj.drawMatches();
        end

        function cbMatchClicked(obj, src, event)
            % Figure out which match was clicked
            cs = matches2coords(obj.results.matching.selected.matches);
            vs = cs - ones(size(cs))*diag(obj.hAxMain.CurrentPoint(1,1:2));
            [x, mI] = min(sum(vs.^2, 2)); % Index for match with min distance^2
            obj.selectedMatch = cs(mI,:);

            % Update the UI selector to reflect the match
            obj.hOptsMatchSelectValue.Value = mI + 1;

            % Redraw the matches screen
            obj.drawMatches();
        end

        function cbMatchSelected(obj, src, event)
            % Update the selected match
            obj.updateSelectedMatch();

            % Redraw the matches screen
            obj.drawMatches();
        end

        function cbNextFrame(obj, src, event)
            % Get the next frame
            obj.currentVideoMatch = ResultsGUI.nextMatch( ...
                obj.currentVideoMatch, ...
                obj.results.matching.selected.matches);
            if isempty(obj.currentVideoMatch)
                obj.currentVideoMatch = ResultsGUI.nextMatch([0 0], ...
                    obj.results.matching.selected.matches);
            end

            % Redraw the axes on screen
            obj.drawVideo();
        end

        function cbPlayVideo(obj, src, event)
            % Go down two possible branches, depending on if video is playing
            if strcmpi(obj.hOptsVidPlay.String, 'play')
                % Update the UI to reflect that the video is playing
                obj.toggleVideoScreenLock(true);
                obj.hOptsVidPlay.Enable = 'on';
                obj.hOptsVidPlay.String = 'Pause';

                % Start the timer
                obj.videoTimer = timer();
                obj.videoTimer.BusyMode = 'queue';
                obj.videoTimer.ExecutionMode = 'fixedrate';
                obj.videoTimer.ObjectVisibility = 'off';
                obj.videoTimer.Period =  ...
                    1 / str2num(obj.hOptsVidRateValue.String);
                obj.videoTimer.TimerFcn = {@obj.cbNextFrame};
                start(obj.videoTimer);
            else
                % Update the UI to reflect that the video is now paused
                obj.toggleVideoScreenLock(false);
                obj.hOptsVidPlay.String = 'Play';

                % Update the slider value
                ms = obj.results.matching.selected.matches(...
                    ~isnan(obj.results.matching.selected.matches));
                obj.hOptsVidSlider.Value = find(ms == obj.currentVideoMatch(2));

                % Stop the timer, and delete it
                stop(obj.videoTimer);
                delete(obj.videoTimer);
                obj.videoTimer = [];
            end
        end

        function cbPRClicked(obj, src, event)
            % Figure out which PR point was clicked
            cs = [obj.results.pr.values.precisions' ...
                obj.results.pr.values.recalls'];
            vs = cs - ones(size(cs))*diag(obj.hAxPR.CurrentPoint(1,1:2));
            [x, mI] = min(sum(vs.^2, 2)); % Index for PR with min distance^2

            % Update the UI to reflect the click
            obj.hFocusPRSlider.Value = (mI-1) / size(cs, 1);
            obj.cbUpdatePRFocusValues(obj.hFocusPRSlider, []);

            % Redraw the PR screen
            obj.drawPrecisionRecall();
        end

        function cbSelectScreen(obj, src, event)
            obj.openScreen(obj.hScreen.Value);
        end

        function cbSaveResults(obj, src, event)
            % Prompt the user for a save directory
            resultsDir = uigetdir('', ...
                'Select the directory where the results will be saved');
            if isnumeric(resultsDir)
                uiwait(msgbox(['No save location was selected, ' ...
                    'results were not saved'], 'No save location selected', ...
                    'modal'));
                return;
            end

            % Save the results
            resultsSave(resultsDir, obj.config, 'config.xml');
            resultsSave(resultsDir, obj.results.preprocessed, ...
                'preprocessed.mat');
            resultsSave(resultsDir, obj.results.diff_matrix, ...
                'diff_matrix.mat');
            resultsSave(resultsDir, obj.results.matching, ...
                'matching.mat');
            resultsSave(resultsDir, obj.results.pr, ...
                'precision_recall.mat');

            uiwait(msgbox({'Results were saved to:' resultsDir}, ...
                'Save Successful', 'modal'));
        end

        function cbShowSequence(obj, src, event)
            obj.hFocusButton.Enable = 'off';

            % Figure out the rs and qs
            qs = squeeze( ...
                obj.results.matching.selected.trajectories( ...
                obj.selectedMatch(1),1,:));
            rs = squeeze( ...
                obj.results.matching.selected.trajectories( ...
                obj.selectedMatch(1),2,:));

            % Call the sequence popup (it should block until closed)
            obj.interactivity(false);
            sequi = SequencePopup(qs, rs, obj.config, obj.results);
            uiwait(sequi.hFig);
            obj.interactivity(true);
            obj.openScreen(obj.hScreen.Value);

            obj.hFocusButton.Enable = 'on';
        end

        function cbTogglePRValueHighlight(obj, src, event)
            obj.drawPrecisionRecall();
        end

        function cbTweakMatches(obj, src, event)
            % Launch the tweaking popup (and wait until done)
            obj.interactivity(false);
            tweakui = TweakMatchesPopup(obj.config, obj.results);
            uiwait(tweakui.hFig);
            obj.interactivity(true);

            % Update the config, and results (changes should only have been
            % made if apply was clicked, and not close)
            obj.results = tweakui.results;
            obj.config = tweakui.config;

            % Update the results, and update the GUI
            obj.selectedMatch = [];
            obj.hOptsMatchSelectValue.Value = 1;
            obj.updateMatches();
            obj.openScreen(obj.hScreen.Value);
        end

        function cbUpdatePRFocusValues(obj, src, event)
            % Don't do this unless precision-recall has already been
            % calculated
            if ~isequal(obj.hOptsPRGroundTruthDetails.ForegroundColor, ...
                    GUISettings.COL_ERROR)
                % Figure out which point the slider position corresponds to
                n = obj.selectedPRFocus();

                % Update all values
                vs = linspace(obj.results.pr.sweep_var.start, ...
                    obj.results.pr.sweep_var.end, ...
                    obj.results.pr.sweep_var.num_steps);
                obj.hFocusPRValValue.String = num2str(vs(n));
                obj.hFocusPRPrecisionValue.String = num2str( ...
                    obj.results.pr.values.precisions(n));
                obj.hFocusPRRecallValue.String = num2str( ...
                    obj.results.pr.values.recalls(n));
            end

            % Force a redraw
            obj.drawPrecisionRecall();
        end

        function cbUpdateNumSweepPoint(obj, src, event)
            obj.refreshPrecisionRecallScreen();
        end

        function cbVideoSliderAdjust(obj, src, event)
            % TODO this is a silly / lazy way to do this.... fix
            % TODO actually this whole approach to getting the frame
            % corresponding to a given match is poor...
            idxs = find(~isnan(obj.results.matching.selected.matches));
            if obj.hOptsVidSlider.Value > 1
                dummyMatch = [idxs(ceil(obj.hOptsVidSlider.Value - 1)) 0];
            else
                dummyMatch = [0 0];
            end
            obj.currentVideoMatch = ResultsGUI.nextMatch(dummyMatch, ...
                obj.results.matching.selected.matches);
            obj.drawVideo();
        end

        function clearScreen(obj)
            % Hide all options
            obj.hOptsPreDataset.Visible = 'off';
            obj.hOptsPreDatasetValue.Visible = 'off';
            obj.hOptsPreImage.Visible = 'off';
            obj.hOptsPreImageValue.Visible = 'off';
            obj.hOptsDiffContr.Visible = 'off';
            obj.hOptsDiffCol.Visible = 'off';
            obj.hOptsDiffColValue.Visible = 'off';
            obj.hOptsMatchDiff.Visible = 'off';
            obj.hOptsMatchSeqs.Visible = 'off';
            obj.hOptsMatchMatches.Visible = 'off';
            obj.hOptsMatchSelect.Visible = 'off';
            obj.hOptsMatchSelectValue.Visible = 'off';
            obj.hOptsMatchTweak.Visible = 'off';
            obj.hOptsPRGroundTruth.Visible = 'off';
            obj.hOptsPRGroundTruthDetails.Visible = 'off';
            obj.hOptsPRSweepVar.Visible = 'off';
            obj.hOptsPRSweepVarValue.Visible = 'off';
            obj.hOptsPRSweepNum.Visible = 'off';
            obj.hOptsPRSweepNumValue.Visible = 'off';
            obj.hOptsPRError.Visible = 'off';
            obj.hOptsVidRate.Visible = 'off';
            obj.hOptsVidRateValue.Visible = 'off';
            obj.hOptsVidPlay.Visible = 'off';
            obj.hOptsVidSlider.Visible = 'off';
            obj.hOptsVidExport.Visible = 'off';

            % Hide all on screen content
            obj.hAxA.Visible = 'off';
            obj.hAxA.Title.Visible = 'off';
            obj.hAxB.Visible = 'off';
            obj.hAxB.Title.Visible = 'off';
            obj.hAxC.Visible = 'off';
            obj.hAxC.Title.Visible = 'off';
            obj.hAxD.Visible = 'off';
            obj.hAxD.Title.Visible = 'off';
            obj.hAxMain.Visible = 'off';
            obj.hAxMain.Title.Visible = 'off';
            obj.hAxPR.Visible = 'off';
            obj.hAxPR.Title.Visible = 'off';
            obj.hAxVideo.Visible = 'off';
            obj.hAxVideo.Title.Visible = 'off';

            % Hide all of the focus panes
            obj.hFocus.Visible = 'off';
            obj.hFocusPR.Visible = 'off';

            % Clear the axes
            cla(obj.hAxA);
            cla(obj.hAxB);
            cla(obj.hAxC);
            cla(obj.hAxD);
            cla(obj.hAxMain);
            cla(obj.hAxPR);
            cla(obj.hAxVideo);

            % Remove any screen dependent callbacks
            obj.hAxMain.ButtonDownFcn = {};
            obj.hAxPR.ButtonDownFcn = {};
        end

        function createGUI(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'OpenSeqSLAM2.0 Results';

            % Generic elements
            obj.hSaveResults = uicontrol('Style', 'pushbutton');
            obj.hSaveResults.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hSaveResults);
            obj.hSaveResults.String = 'Save results';
            obj.hSaveResults.FontWeight = 'bold';

            obj.hScreen = uicontrol('Style', 'popupmenu');
            GUISettings.applyUIControlStyle(obj.hScreen);
            obj.hScreen.String = ResultsGUI.SCREENS;

            obj.hTitle = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hTitle);
            GUISettings.setFontScale(obj.hTitle, 2.5);
            obj.hTitle.FontWeight = 'bold';
            obj.hTitle.String = 'Testing 1 2 3';

            % Axes
            obj.hAxA = axes();
            GUISettings.applyUIAxesStyle(obj.hAxA);
            obj.hAxA.Visible = 'off';

            obj.hAxB = axes();
            GUISettings.applyUIAxesStyle(obj.hAxB);
            obj.hAxB.Visible = 'off';

            obj.hAxC = axes();
            GUISettings.applyUIAxesStyle(obj.hAxC);
            obj.hAxC.Visible = 'off';

            obj.hAxD = axes();
            GUISettings.applyUIAxesStyle(obj.hAxD);
            obj.hAxD.Visible = 'off';

            obj.hAxMain = axes();
            GUISettings.applyUIAxesStyle(obj.hAxMain);
            obj.hAxMain.Visible = 'off';

            obj.hAxPR = axes();
            GUISettings.applyUIAxesStyle(obj.hAxPR);
            obj.hAxPR.Visible = 'off';

            obj.hAxVideo = axes();
            GUISettings.applyUIAxesStyle(obj.hAxVideo);
            obj.hAxVideo.Visible = 'off';

            % Options area for each screen
            obj.hOpts = uipanel();
            GUISettings.applyUIPanelStyle(obj.hOpts);
            obj.hOpts.Title = 'Options';

            obj.hOptsPreDataset = uicontrol('Style', 'text');
            obj.hOptsPreDataset.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPreDataset);
            obj.hOptsPreDataset.String = 'Dataset:';

            obj.hOptsPreDatasetValue = uicontrol('Style', 'popupmenu');
            obj.hOptsPreDatasetValue.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPreDatasetValue);
            obj.hOptsPreDatasetValue.String = {'Reference', 'Query'};

            obj.hOptsPreImage = uicontrol('Style', 'text');
            obj.hOptsPreImage.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPreImage);
            obj.hOptsPreImage.String = 'Image:';

            obj.hOptsPreImageValue = uicontrol('Style', 'popupmenu');
            obj.hOptsPreImageValue.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPreImageValue);
            obj.hOptsPreImageValue.String = '';

            obj.hOptsDiffContr = uicontrol('Style', 'checkbox');
            obj.hOptsDiffContr.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsDiffContr);
            obj.hOptsDiffContr.String = 'Contrast Enhanced';

            obj.hOptsDiffCol = uicontrol('Style', 'text');
            obj.hOptsDiffCol.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsDiffCol);
            obj.hOptsDiffCol.String = 'Outlier Fading:';

            obj.hOptsDiffColValue = uicontrol('Style', 'slider');
            obj.hOptsDiffColValue.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsDiffColValue);
            obj.hOptsDiffColValue.Min = 0.01;
            obj.hOptsDiffColValue.Value = obj.hOptsDiffColValue.Min;

            obj.hOptsMatchDiff = uicontrol('Style', 'checkbox');
            obj.hOptsMatchDiff.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsMatchDiff);
            obj.hOptsMatchDiff.String = 'Show Difference Matrix';
            obj.hOptsMatchDiff.Value = 1;

            obj.hOptsMatchSeqs = uicontrol('Style', 'checkbox');
            obj.hOptsMatchSeqs.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsMatchSeqs);
            obj.hOptsMatchSeqs.String = 'Plot Sequences';

            obj.hOptsMatchMatches = uicontrol('Style', 'checkbox');
            obj.hOptsMatchMatches.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsMatchMatches);
            obj.hOptsMatchMatches.String = 'Plot Matches';
            obj.hOptsMatchMatches.Value = 1;

            obj.hOptsMatchSelect = uicontrol('Style', 'text');
            obj.hOptsMatchSelect.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsMatchSelect);
            obj.hOptsMatchSelect.String = 'Selected Match:';

            obj.hOptsMatchSelectValue = uicontrol('Style', 'popupmenu');
            obj.hOptsMatchSelectValue.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsMatchSelectValue);
            obj.hOptsMatchSelectValue.String = '';

            obj.hOptsMatchTweak = uicontrol('Style', 'pushbutton');
            obj.hOptsMatchTweak.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsMatchTweak);
            obj.hOptsMatchTweak.String = 'Tweak matching';

            obj.hOptsPRGroundTruth = uicontrol('Style', 'pushbutton');
            obj.hOptsPRGroundTruth.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPRGroundTruth);
            obj.hOptsPRGroundTruth.String = 'Configure ground truth';

            obj.hOptsPRGroundTruthDetails = uicontrol('Style', 'text');
            obj.hOptsPRGroundTruthDetails.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPRGroundTruthDetails);
            obj.hOptsPRGroundTruthDetails.String = {'' ''};
            obj.hOptsPRGroundTruthDetails.HorizontalAlignment = 'left';
            obj.hOptsPRGroundTruthDetails.FontAngle = 'italic';

            obj.hOptsPRSweepVar = uicontrol('Style', 'text');
            obj.hOptsPRSweepVar.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPRSweepVar);
            obj.hOptsPRSweepVar.String = 'Sweep variable:';

            obj.hOptsPRSweepVarValue = annotation(obj.hOpts, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hOptsPRSweepVarValue);
            obj.hOptsPRSweepVarValue.String = '';

            obj.hOptsPRSweepNum = uicontrol('Style', 'text');
            obj.hOptsPRSweepNum.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPRSweepNum);
            obj.hOptsPRSweepNum.String = '# of sweep points:';

            obj.hOptsPRSweepNumValue = uicontrol('Style', 'edit');
            obj.hOptsPRSweepNumValue.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsPRSweepNumValue);
            obj.hOptsPRSweepNumValue.String = '';

            obj.hOptsPRError = uicontrol('Style', 'text');
            obj.hOptsPRError.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hOptsPRError);
            GUISettings.setFontScale(obj.hOptsPRError, 1.75);
            obj.hOptsPRError.String = ...
                {'Please first correctly configure the' ...
                'ground truth matrix via the button above'};
            obj.hOptsPRError.FontAngle = 'italic';

            obj.hOptsVidRate = uicontrol('Style', 'text');
            obj.hOptsVidRate.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsVidRate);
            obj.hOptsVidRate.String = 'Frame rate (Hz):';

            obj.hOptsVidRateValue = uicontrol('Style', 'edit');
            obj.hOptsVidRateValue.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsVidRateValue);
            obj.hOptsVidRateValue.String = '1';

            obj.hOptsVidPlay = uicontrol('Style', 'pushbutton');
            obj.hOptsVidPlay.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsVidPlay);
            obj.hOptsVidPlay.String = 'Play';

            obj.hOptsVidSlider = uicontrol('Style', 'slider');
            obj.hOptsVidSlider.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsVidSlider);

            obj.hOptsVidExport = uicontrol('Style', 'pushbutton');
            obj.hOptsVidExport.Parent = obj.hOpts;
            GUISettings.applyUIControlStyle(obj.hOptsVidExport);
            obj.hOptsVidExport.String = 'Export...';

            % Focus Pane
            obj.hFocus = uipanel();
            GUISettings.applyUIPanelStyle(obj.hFocus);
            obj.hFocus.Title = 'Focus: Off';

            obj.hFocusAx = axes();
            obj.hFocusAx.Parent = obj.hFocus;
            GUISettings.applyUIAxesStyle(obj.hFocusAx);
            obj.hFocusAx.Visible = 'off';

            obj.hFocusButton = uicontrol('Style', 'pushbutton');
            obj.hFocusButton.Parent = obj.hFocus;
            GUISettings.applyUIControlStyle(obj.hFocusButton);
            obj.hFocusButton.String = 'Show sequence';

            obj.hFocusRef = uicontrol('Style', 'text');
            obj.hFocusRef.Parent = obj.hFocus;
            GUISettings.applyUIControlStyle(obj.hFocusRef);
            obj.hFocusRef.String = 'Reference Image:';

            obj.hFocusRefAx = axes();
            obj.hFocusRefAx.Parent = obj.hFocus;
            GUISettings.applyUIAxesStyle(obj.hFocusRefAx);
            obj.hFocusRefAx.Visible = 'off';

            obj.hFocusQuery = uicontrol('Style', 'text');
            obj.hFocusQuery.Parent = obj.hFocus;
            GUISettings.applyUIControlStyle(obj.hFocusQuery);
            obj.hFocusQuery.String = 'Query Image:';

            obj.hFocusQueryAx = axes();
            obj.hFocusQueryAx.Parent = obj.hFocus;
            GUISettings.applyUIAxesStyle(obj.hFocusQueryAx);
            obj.hFocusQueryAx.Visible = 'off';

            % Focus Pane for Precision Recall
            obj.hFocusPR = uipanel();
            GUISettings.applyUIPanelStyle(obj.hFocusPR);
            obj.hFocusPR.Title = 'Variable Value Explorer';

            obj.hFocusPRVisualise = uicontrol('Style', 'checkbox');
            obj.hFocusPRVisualise.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRVisualise);
            obj.hFocusPRVisualise.String = 'Highlight value in plot';
            obj.hFocusPRVisualise.Value = 1;

            obj.hFocusPRSlider = uicontrol('Style', 'slider');
            obj.hFocusPRSlider.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRSlider);

            obj.hFocusPRVal = annotation(obj.hFocusPR, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hFocusPRVal);
            obj.hFocusPRVal.String = '';
            obj.hFocusPRVal.FontWeight = 'bold';

            obj.hFocusPRValValue = uicontrol('Style', 'text');
            obj.hFocusPRValValue.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRValValue);
            obj.hFocusPRValValue.FontAngle = 'italic';

            obj.hFocusPRPrecision = uicontrol('Style', 'text');
            obj.hFocusPRPrecision.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRPrecision);
            obj.hFocusPRPrecision.String = 'Precision:';
            obj.hFocusPRPrecision.FontWeight = 'bold';
            obj.hFocusPRPrecision.HorizontalAlignment = 'left';

            obj.hFocusPRPrecisionValue = uicontrol('Style', 'text');
            obj.hFocusPRPrecisionValue.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRPrecisionValue);
            obj.hFocusPRPrecisionValue.FontAngle = 'italic';

            obj.hFocusPRRecall = uicontrol('Style', 'text');
            obj.hFocusPRRecall.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRRecall);
            obj.hFocusPRRecall.String = 'Recall:';
            obj.hFocusPRRecall.FontWeight = 'bold';
            obj.hFocusPRRecall.HorizontalAlignment = 'left';

            obj.hFocusPRRecallValue = uicontrol('Style', 'text');
            obj.hFocusPRRecallValue.Parent = obj.hFocusPR;
            GUISettings.applyUIControlStyle(obj.hFocusPRRecallValue);
            obj.hFocusPRRecallValue.FontAngle = 'italic';

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hSaveResults.Callback = {@obj.cbSaveResults};
            obj.hScreen.Callback = {@obj.cbSelectScreen};
            obj.hOptsPreDatasetValue.Callback = {@obj.cbChangeDataset};
            obj.hOptsPreImageValue.Callback = {@obj.cbChangeImage};
            obj.hOptsDiffContr.Callback = {@obj.cbDiffOptionChange};
            obj.hOptsDiffColValue.Callback = {@obj.cbDiffOptionChange};
            obj.hOptsMatchDiff.Callback = {@obj.cbMatchesOptionChange};
            obj.hOptsMatchSeqs.Callback = {@obj.cbMatchesOptionChange};
            obj.hOptsMatchMatches.Callback = {@obj.cbMatchesOptionChange};
            obj.hOptsMatchSelectValue.Callback = {@obj.cbMatchSelected};
            obj.hOptsMatchTweak.Callback = {@obj.cbTweakMatches};
            obj.hOptsPRGroundTruth.Callback = {@obj.cbConfigureGroundTruth};
            obj.hOptsPRSweepNumValue.Callback = {@obj.cbUpdateNumSweepPoint};
            obj.hOptsVidPlay.Callback = {@obj.cbPlayVideo};
            addlistener(obj.hOptsVidSlider, 'Value', 'PostSet', ...
                @obj.cbVideoSliderAdjust);
            obj.hOptsVidExport.Callback = {@obj.cbExportVideo};
            obj.hFocusButton.Callback = {@obj.cbShowSequence};
            obj.hFocusPRVisualise.Callback = {@obj.cbTogglePRValueHighlight};
            addlistener(obj.hFocusPRSlider, 'Value', 'PostSet', ...
                @obj.cbUpdatePRFocusValues);
            obj.hFig.CloseRequestFcn = {@obj.cbClose};
        end

        function drawPreprocessed(obj)
            % Get all of the requested parameters
            ds = lower(obj.hOptsPreDatasetValue.String{ ...
                obj.hOptsPreDatasetValue.Value});
            img = datasetOpenImage(obj.config.(ds), ...
                obj.hOptsPreImageValue.Value, ...
                obj.results.preprocessed.([ds '_numbers']));

            % Grab the images from each of the steps
            [img_out, imgs] = SeqSLAMInstance.preprocessSingle(img, ...
                obj.config.seqslam.image_processing, ...
                lower(obj.hOptsPreDatasetValue.String{ ...
                obj.hOptsPreDatasetValue.Value}), 1);

            % Plot the 4 images
            cla(obj.hAxA); imshow(img, 'Parent', obj.hAxA);
            cla(obj.hAxB); imshow(imgs{1}, 'Parent', obj.hAxB);
            cla(obj.hAxC); imshow(imgs{2}, 'Parent', obj.hAxC);
            cla(obj.hAxD); imshow(img_out, 'Parent', obj.hAxD);

            % Style the plots
            obj.hAxA.Title.String = ['Original (' ...
                num2str(size(img, 2)) 'x' ...
                num2str(size(img, 1)) ')'];
            obj.hAxB.Title.String = ['Greyscale (' ...
                num2str(size(imgs{1}, 2)) 'x' ...
                num2str(size(imgs{1}, 1)) ')'];
            obj.hAxC.Title.String = ['Resized and cropped (' ...
                num2str(size(imgs{2}, 2)) 'x' ...
                num2str(size(imgs{2}, 1)) ')'];
            obj.hAxD.Title.String = ['Patch normalised (' ...
                num2str(size(img_out, 2)) 'x' ...
                num2str(size(img_out, 1)) ')'];
            GUISettings.axesHide(obj.hAxA);
            GUISettings.axesHide(obj.hAxB);
            GUISettings.axesHide(obj.hAxC);
            GUISettings.axesHide(obj.hAxD);

            % Do the prettying up (don't know why I have to do this after
            % every plot call Matlab...)
            obj.hAxMain.Visible = 'off';
        end

        function drawDiffMatrix(obj)
            % Useful temporaries
            szDiff = size(obj.results.diff_matrix.enhanced);
            d = obj.selectedDiff;
            ds = obj.config.seqslam.search.d_s;

            % Clear the axis to start
            cla(obj.hAxMain);
            hold(obj.hAxMain, 'on');

            % Draw the requested difference matrix
            if obj.hOptsDiffContr.Value
                dataNorm = obj.results.diff_matrix.enhanced ./ ...
                    max(max(obj.results.diff_matrix.enhanced));
            else
                dataNorm = obj.results.diff_matrix.base ./ ...
                    max(max(obj.results.diff_matrix.base));
            end
            dataNorm = 1 - exp(-10*obj.hOptsDiffColValue.Value*dataNorm);
            imagesc(obj.hAxMain, dataNorm);
            hold(obj.hAxMain, 'off');

            % Style the plot
            GUISettings.axesDiffMatrixStyle(obj.hAxMain, szDiff);
            arrayfun(@(x) set(x, 'HitTest', 'off'), obj.hAxMain.Children);

            % Update the focus area and plots
            if ~isempty(d)
                % Display, and update the title with the details
                obj.hFocus.Visible = 'on';
                obj.hFocus.Title = ['Focus: centred on (query #' ...
                    num2str(d(1)) ', ref #' num2str(d(2)) ')'];

                % Get the limits of the cutout, and the cutout data
                rLimits = d(2) - floor(ds/2) + [0 ds-1];
                qLimits = d(1) - floor(ds/2) + [0 ds-1];
                rDataLimits = max(rLimits(1),1) : min(rLimits(2), szDiff(1));
                qDataLimits = max(qLimits(1),1) : min(qLimits(2), szDiff(1));

                % Draw the focus box in the main axis
                hold(obj.hAxMain, 'on');
                h = rectangle(obj.hAxMain, 'Position', ...
                    [qDataLimits(1), rDataLimits(1), ...
                    range(qDataLimits), range(rDataLimits)]);
                hold(obj.hAxMain, 'off');

                % Draw the elements of the focus cutout
                cla(obj.hFocusAx);
                hold(obj.hFocusAx, 'on');
                if obj.hOptsDiffContr.Value
                    imagesc(obj.hFocusAx, qDataLimits, rDataLimits, ...
                        obj.results.diff_matrix.enhanced(rDataLimits, ...
                        qDataLimits));
                else
                    imagesc(obj.hFocusAx, qDataLimits, rDataLimits, ...
                        obj.results.diff_matrix.base(rDataLimits, ...
                        qDataLimits));
                end
                h = plot(obj.hFocusAx, d(1), d(2), 'k.');
                h.MarkerSize = h.MarkerSize * 6;
                hold(obj.hFocusAx, 'off');
                GUISettings.axesDiffMatrixFocusStyle(obj.hFocusAx, ...
                    qLimits, rLimits);

                % Draw the reference and query images
                imshow(datasetOpenImage(obj.config.('reference'), d(2), ...
                    obj.results.preprocessed.('reference_numbers')), ...
                    'Parent', obj.hFocusRefAx);
                imshow(datasetOpenImage(obj.config.('query'), d(1), ...
                    obj.results.preprocessed.('query_numbers')), ...
                    'Parent', obj.hFocusQueryAx);
            else
                obj.hFocus.Visible = 'off';
                obj.hFocus.Title = ['Focus: Off'];
            end
        end

        function drawMatches(obj)
            % Useful temporaries
            m = obj.selectedMatch;
            szDiff = size(obj.results.diff_matrix.enhanced);
            szTrajs = size(obj.results.matching.selected.trajectories);

            % Fill in the difference matrix plot, with any requested overlaying
            % features
            cla(obj.hAxMain);
            hold(obj.hAxMain, 'on');
            if obj.hOptsMatchDiff.Value
                h = imagesc(obj.hAxMain, obj.results.diff_matrix.enhanced);
                h.AlphaData = 0.25;
            end
            if obj.hOptsMatchSeqs.Value
                arrayfun(@(x) plot(obj.hAxMain, ...
                    squeeze(obj.results.matching.selected.trajectories( ...
                    x,1,:)), ...
                    squeeze(obj.results.matching.selected.trajectories( ...
                    x,2,:)), ...
                    '-'), 1:szTrajs(1));
            end
            if obj.hOptsMatchMatches.Value
                plot(obj.hAxMain, ...
                    obj.results.matching.selected.matches, '.');
            end
            if ~isempty(m)
                plot(obj.hAxMain, [m(1) m(1)], [1 szDiff(1)], 'k--', ...
                    [1 szDiff(2)], [m(2) m(2)], 'k--');
            end
            hold(obj.hAxMain, 'off');

            % Style the difference matrix plot
            GUISettings.axesDiffMatrixStyle(obj.hAxMain, szDiff);
            arrayfun(@(x) set(x, 'HitTest', 'off'), obj.hAxMain.Children);

            % Update the focus area and plots
            if ~isempty(m)
                % Get the trajectory, and update the title with its details
                t = obj.results.matching.selected.trajectories(m(1),:,:);
                obj.hFocus.Visible = 'on';
                obj.hFocus.Title = ['Focus: (matched #' num2str(m(1)) ...
                    ' with #' num2str(m(2)) ')'];

                % Get maximum "distance", and limits for the focus cutout
                ds = max(obj.config.seqslam.search.d_s, ...
                    1 + range(t(:,2,:)));
                if ds == obj.config.seqslam.search.d_s
                    rLimits = m(2) - floor(ds/2) + [0 ds-1];
                else
                    rLimits = [min(t(:,2,:)) max(t(:,2,:))];
                end
                qLimits = m(1) - floor(ds/2) + [0 ds-1];

                % Draw the elements of the focus cutout
                cla(obj.hFocusAx);
                hold(obj.hFocusAx, 'on');
                if obj.hOptsMatchDiff.Value
                    qDataLimits = max(qLimits(1), 1) : ...
                        min(qLimits(2), szDiff(2));
                    rDataLimits = max(rLimits(1), 1) : ...
                        min(rLimits(2), szDiff(1));
                    h = imagesc(obj.hFocusAx, qDataLimits, rDataLimits, ...
                        obj.results.diff_matrix.enhanced(rDataLimits, ...
                        qDataLimits));
                end
                if obj.hOptsMatchSeqs.Value
                    h = plot(obj.hFocusAx, squeeze(t(:,1,:)), ...
                        squeeze(t(:,2,:)), 'k-');
                    h.LineWidth = h.LineWidth * 5;
                end
                if obj.hOptsMatchMatches.Value
                    h = plot(obj.hFocusAx, m(1), ...
                        obj.results.matching.selected.matches(m(1)), 'k.');
                    h.MarkerSize = h.MarkerSize * 6;
                end
                hold(obj.hFocusAx, 'off');
                GUISettings.axesDiffMatrixFocusStyle(obj.hFocusAx, ...
                    qLimits, rLimits);

                % Draw the reference and query images
                imshow(datasetOpenImage(obj.config.('reference'), m(2), ...
                    obj.results.preprocessed.('reference_numbers')), ...
                    'Parent', obj.hFocusRefAx);
                imshow(datasetOpenImage(obj.config.('query'), m(1), ...
                    obj.results.preprocessed.('query_numbers')), ...
                    'Parent', obj.hFocusQueryAx);
            else
                obj.hFocus.Visible = 'off';
                obj.hFocus.Title = ['Focus: Off'];
            end
        end

        function drawPrecisionRecall(obj)
            % Don't do anything if there isn't a valid configuration present
            if isequal(obj.hOptsPRGroundTruthDetails.ForegroundColor, ...
                    GUISettings.COL_ERROR)
                obj.hOptsPRError.Visible = 'on';
                obj.hAxPR.Visible = 'off';
                obj.hFocusPR.Visible = 'off';
                return;
            end
            obj.hOptsPRError.Visible = 'off';
            obj.hFocusPR.Visible = 'on';

            % Draw the precision recall plot (use saved values)
            cla(obj.hAxPR);
            hold(obj.hAxPR, 'on');
            h = plot(obj.hAxPR, obj.results.pr.values.recalls, ...
                obj.results.pr.values.precisions, 'bo-');
            h.MarkerFaceColor = 'b';
            h.MarkerSize = h.MarkerSize * 0.5;

            % Draw the highlighting if required
            if obj.hFocusPRVisualise.Value == 1
                n = obj.selectedPRFocus();
                p = obj.results.pr.values.precisions(n);
                r = obj.results.pr.values.recalls(n);
                h = plot(obj.hAxPR, r, p, 'o');
                h.MarkerEdgeColor = GUISettings.COL_WARNING;
                h.MarkerFaceColor = GUISettings.COL_WARNING;
                h = plot(obj.hAxPR, [0 r r], [p p 0], '--');
                h.Color = GUISettings.COL_WARNING;
            end
            hold(obj.hAxPR, 'off');

            % Style the plot
            GUISettings.axesPrecisionRecallStyle(obj.hAxPR);
        end

        function drawVideo(obj)
            hold(obj.hAxVideo, 'on');

            % Build the frame, then display it
            qIm = datasetOpenImage( ...
                obj.config.query, ...
                obj.currentVideoMatch(1), ...
                obj.results.preprocessed.query_numbers);
            rIm = datasetOpenImage( ...
                obj.config.reference, ...
                obj.currentVideoMatch(2), ...
                obj.results.preprocessed.reference_numbers);
            frame = [qIm; rIm];
            imshow(frame, 'Parent', obj.hAxVideo);

            % Draw the text over the top
            inset = 0.05 * size(qIm,1);
            t = text(obj.hAxVideo, inset, inset, ...
                ['Query #' num2str(obj.currentVideoMatch(1))]);
            t.FontSize = 16;
            t.Color = 'r';
            t = text(obj.hAxVideo, inset, size(frame,1) - inset, ...
                ['Reference #' num2str(obj.currentVideoMatch(2))]);
            t.FontSize = 16;
            t.Color = 'r';

            hold(obj.hAxVideo, 'off');
        end

        function greyAxes(obj)
            % Apply alpha to axes
            alpha(obj.hAxA, 0.2);
            alpha(obj.hAxB, 0.2);
            alpha(obj.hAxC, 0.2);
            alpha(obj.hAxD, 0.2);
            alpha(obj.hAxMain, 0.2);

            % Hide the titles
            obj.hAxA.Title.Visible = 'off';
            obj.hAxB.Title.Visible = 'off';
            obj.hAxC.Title.Visible = 'off';
            obj.hAxD.Title.Visible = 'off';
            obj.hAxMain.Title.Visible = 'off';
        end

        function interactivity(obj, enable)
            if enable
                status = 'on';
            else
                status = 'off';
            end

            obj.hHelp.Enable = status;
            obj.hSaveResults.Enable = status;
            obj.hScreen.Enable = status;
            obj.hOptsPreDatasetValue.Enable = status;
            obj.hOptsPreImageValue.Enable = status;
            obj.hOptsDiffContr.Enable = status;
            obj.hOptsDiffColValue.Enable = status;
            obj.hOptsMatchDiff.Enable = status;
            obj.hOptsMatchSeqs.Enable = status;
            obj.hOptsMatchMatches.Enable = status;
            obj.hOptsMatchSelectValue.Enable = status;
            obj.hOptsMatchTweak.Enable = status;
            obj.hOptsVidRateValue.Enable = status;
            obj.hOptsVidPlay.Enable = status;
            obj.hOptsVidSlider.Enable = status;
            obj.hOptsVidExport.Enable = status;
            obj.hFocusButton.Enable = status;
        end

        function openScreen(obj, screen)
            % Clear everything off the screen
            obj.clearScreen();

            % Update the title
            obj.hTitle.String = ResultsGUI.SCREENS{screen};

            % Add the appropriate elements for the screen
            if (screen == 1)
                % Image preprocessing screen
                HelpPopup.setDestination(obj.hHelp, ...
                    'results/image_preprocessing');

                % Show the appropriate options
                obj.hOptsPreDataset.Visible = 'on';
                obj.hOptsPreDatasetValue.Visible = 'on';
                obj.hOptsPreImage.Visible = 'on';
                obj.hOptsPreImageValue.Visible = 'on';

                % Turn on the required axes
                obj.hAxMain.Visible = 'on';

                % Select the dataset, and manually trigger the refresh
                obj.cbChangeDataset();
                obj.drawPreprocessed();
            elseif (screen == 2)
                % Difference matrix screen
                HelpPopup.setDestination(obj.hHelp, ...
                    'results/diff_matrix');

                % Show the appropriate options
                obj.hOptsDiffContr.Visible = 'on';
                obj.hOptsDiffCol.Visible = 'on';
                obj.hOptsDiffColValue.Visible = 'on';

                % Turn on the required axes
                obj.hAxMain.Visible = 'on';

                % Turn on the focus box
                obj.hFocus.Visible = 'on';
                obj.hFocusButton.Visible = 'off';

                % Register the callback for the main axis
                obj.hAxMain.ButtonDownFcn = {@obj.cbDiffClicked};

                % Draw the content
                obj.drawDiffMatrix();
            elseif(screen == 3)
                % Sequence matches screen
                HelpPopup.setDestination(obj.hHelp, ...
                    'results/matches');

                % Show the appropriate options
                obj.hOptsMatchDiff.Visible = 'on';
                obj.hOptsMatchSeqs.Visible = 'on';
                obj.hOptsMatchMatches.Visible = 'on';
                obj.hOptsMatchSelect.Visible = 'on';
                obj.hOptsMatchSelectValue.Visible = 'on';
                obj.hOptsMatchTweak.Visible = 'on';

                % Turn on the required axes
                obj.hAxMain.Visible = 'on';

                % Turn on the focus box
                obj.hFocus.Visible = 'on';
                obj.hFocusButton.Visible = 'on';

                % Register the callback for the main axis
                obj.hAxMain.ButtonDownFcn = {@obj.cbMatchClicked};

                % Create and draw the content
                obj.updateMatches();
                obj.drawMatches();
            elseif (screen == 4)
                % Precision-recall plotting
                HelpPopup.setDestination(obj.hHelp, ...
                    'results/precision_recall');

                % Show the appropriate options
                obj.hOptsPRGroundTruth.Visible = 'on';
                obj.hOptsPRGroundTruthDetails.Visible = 'on';
                obj.hOptsPRSweepVar.Visible = 'on';
                obj.hOptsPRSweepVarValue.Visible = 'on';
                obj.hOptsPRSweepNum.Visible = 'on';
                obj.hOptsPRSweepNumValue.Visible = 'on';

                % Turn on the required axes
                obj.hAxPR.Visible = 'on';

                % Fill in values here (have to do here in case another screen
                % has changed these values - i.e. threshold method changed)
                if strcmp(obj.config.seqslam.matching.method, 'thresh')
                    obj.hOptsPRSweepVarValue.String = '$\lambda$';
                    obj.hFocusPRVal.String = '$\lambda$:';
                else
                    obj.hOptsPRSweepVarValue.String = '$\mu$';
                    obj.hFocusPRVal.String = '$\mu$:';
                end
                obj.updateGroundTruthDescription();

                % TODO probably should do this elsewhere only once... meh
                obj.hOptsPRSweepNumValue.String = ...
                    SafeData.noEmpty(obj.results.pr.sweep_var.num_steps, 25);

                % Register the callback for the main axis
                obj.hAxPR.ButtonDownFcn = {@obj.cbPRClicked};

                % Update and draw the content
                obj.updatePrecisionRecall();
                obj.drawPrecisionRecall();
            elseif (screen == 5)
                % Matches video screen
                HelpPopup.setDestination(obj.hHelp, ...
                    'results/video');

                % Show the appropriate options
                obj.hOptsVidRate.Visible = 'on';
                obj.hOptsVidRateValue.Visible = 'on';
                obj.hOptsVidPlay.Visible = 'on';
                obj.hOptsVidSlider.Visible = 'on';
                obj.hOptsVidExport.Visible = 'on';

                % Turn on the required axes
                obj.hAxVideo.Visible = 'on';

                % Always start at the first frame
                obj.currentVideoMatch = ResultsGUI.nextMatch( ...
                    [0 0], obj.results.matching.selected.matches);

                % Set the correct limits and intervals on the slider
                numMatches = sum(~isnan(obj.results.matching.selected.mask));
                if numMatches <= 1
                    obj.hOptsVidSlider.Enable = 'off';
                else
                    obj.hOptsVidSlider.Max = numMatches;
                    obj.hOptsVidSlider.Min = 1;
                    if (1/(numMatches - 1)) < 0.1
                        obj.hOptsVidSlider.SliderStep = [1/(numMatches-1) 0.1];
                    else
                        obj.hOptsVidSlider.SliderStep = [0.1 0.1];
                    end
                    obj.hOptsVidSlider.Value = 1;
                end

                % Draw the content
                obj.drawVideo();
            end

            % Force a draw at the end
            drawnow();
        end

        function populateDatasetLists(obj)
            % Get the lists for each of the datasets
            obj.listReference = datasetImageList(obj.config.reference, ...
                obj.results.preprocessed.reference_numbers);
            obj.listQuery = datasetImageList(obj.config.query, ...
                obj.results.preprocessed.query_numbers);
        end

        function populateMatchList(obj)
            % transforming the query dataset list
            obj.listMatches = ['All' ...
                obj.listQuery(~isnan(obj.results.matching.selected.mask))];
        end

        function refreshPrecisionRecallScreen(obj)
            obj.updateGroundTruthDescription();
            obj.updatePrecisionRecall();
            obj.updateValueExplorerSlider();
            obj.drawPrecisionRecall();
        end

        function n = selectedPRFocus(obj)
            n = round(obj.hFocusPRSlider.Value * ...
                (str2num(obj.hOptsPRSweepNumValue.String)-1) + 1);
        end

        function sizeGUI(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            widthUnit = obj.hTitle.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hTitle.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * ResultsGUI.FIG_WIDTH_FACTOR, ...
                heightUnit * ResultsGUI.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hSaveResults, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);
            SpecSize.size(obj.hScreen, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.8);

            SpecSize.size(obj.hTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            SpecSize.size(obj.hOpts, SpecSize.HEIGHT, SpecSize.ABSOLUTE, ...
                1.5*heightUnit);
            SpecSize.size(obj.hOpts, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            SpecSize.size(obj.hOptsPreDataset, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hOptsPreDatasetValue, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hOptsPreImage, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hOptsPreImageValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.6);

            SpecSize.size(obj.hOptsDiffContr, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hOptsDiffCol, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hOptsDiffColValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.5);

            SpecSize.size(obj.hOptsMatchDiff, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hOptsMatchSeqs, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hOptsMatchMatches, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hOptsMatchSelect, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hOptsMatchSelectValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.4);
            SpecSize.size(obj.hOptsMatchTweak, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.1);

            SpecSize.size(obj.hOptsPRGroundTruth, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hOptsPRGroundTruthDetails, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.425);
            SpecSize.size(obj.hOptsPRGroundTruthDetails, SpecSize.HEIGHT, ...
                SpecSize.WRAP);
            SpecSize.size(obj.hOptsPRSweepVar, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hOptsPRSweepVarValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.1);
            SpecSize.size(obj.hOptsPRSweepNum, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hOptsPRSweepNumValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.1);
            SpecSize.size(obj.hOptsPRError, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.5);
            SpecSize.size(obj.hOptsPRError, SpecSize.HEIGHT, ...
                SpecSize.PERCENT, obj.hFig, 0.5);

            SpecSize.size(obj.hOptsVidRate, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_SMALL);
            SpecSize.size(obj.hOptsVidRateValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.1);
            SpecSize.size(obj.hOptsVidPlay, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.15);
            SpecSize.size(obj.hOptsVidSlider, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.4);
            SpecSize.size(obj.hOptsVidExport, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hOpts, 0.15);

            SpecSize.size(obj.hAxA, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.25);
            SpecSize.size(obj.hAxA, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);
            SpecSize.size(obj.hAxB, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.25);
            SpecSize.size(obj.hAxB, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);
            SpecSize.size(obj.hAxC, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.25);
            SpecSize.size(obj.hAxC, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);
            SpecSize.size(obj.hAxD, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.25);
            SpecSize.size(obj.hAxD, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);

            SpecSize.size(obj.hAxMain, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.6);
            SpecSize.size(obj.hAxMain, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);

            SpecSize.size(obj.hAxPR, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.6);
            SpecSize.size(obj.hAxPR, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);

            SpecSize.size(obj.hAxVideo, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.9);
            SpecSize.size(obj.hAxVideo, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.7);

            SpecSize.size(obj.hFocus, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.3);
            SpecSize.size(obj.hFocus, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.775);
            SpecSize.size(obj.hFocusAx, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFocus, 0.6);
            SpecSize.size(obj.hFocusAx, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);
            SpecSize.size(obj.hFocusButton, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocus, 0.5)
            SpecSize.size(obj.hFocusRef, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hFocusRefAx, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFocus, 0.5);
            SpecSize.size(obj.hFocusRefAx, SpecSize.HEIGHT, SpecSize.RATIO, ...
                3/4);
            SpecSize.size(obj.hFocusQuery, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hFocusQueryAx, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocus, 0.5);
            SpecSize.size(obj.hFocusQueryAx, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 3/4);

            SpecSize.size(obj.hFocusPR, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.3);
            SpecSize.size(obj.hFocusPR, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.775);
            SpecSize.size(obj.hFocusPRVisualise, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFocusPR, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hFocusPRSlider, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.05);
            SpecSize.size(obj.hFocusPRSlider, SpecSize.HEIGHT, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.6);
            SpecSize.size(obj.hFocusPRVal, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFocusPR, 0.5);
            SpecSize.size(obj.hFocusPRValValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.4);
            SpecSize.size(obj.hFocusPRPrecision, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.5);
            SpecSize.size(obj.hFocusPRPrecisionValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.4);
            SpecSize.size(obj.hFocusPRRecall, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.5);
            SpecSize.size(obj.hFocusPRRecallValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFocusPR, 0.4);

            % Then, systematically place
            SpecPosition.positionIn(obj.hSaveResults, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hSaveResults, obj.hFig, ...
                SpecPosition.RIGHT, 3.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hScreen, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_SMALL);
            SpecPosition.positionIn(obj.hScreen, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hTitle, obj.hScreen, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hOpts, obj.hTitle, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hOpts, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hOptsPreDataset, obj.hOpts, ...
                SpecPosition.TOP, 1.75*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hOptsPreDataset, obj.hOpts, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsPreDatasetValue, ...
                obj.hOptsPreDataset, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPreDatasetValue, ...
                obj.hOptsPreDataset, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsPreImage, ...
                obj.hOptsPreDataset, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPreImage, ...
                obj.hOptsPreDatasetValue, SpecPosition.RIGHT_OF, ...
                2*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsPreImageValue, ...
                obj.hOptsPreDataset, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPreImageValue, ...
                obj.hOptsPreImage, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hOptsDiffContr, obj.hOpts, ...
                SpecPosition.TOP, 1.75*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hOptsDiffContr, obj.hOpts, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsDiffCol, ...
                obj.hOptsDiffContr, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsDiffCol, ...
                obj.hOptsDiffContr, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsDiffColValue, ...
                obj.hOptsDiffCol, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsDiffColValue, ...
                obj.hOptsDiffCol, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);

            SpecPosition.positionIn(obj.hOptsMatchDiff, obj.hOpts, ...
                SpecPosition.TOP, 1.75*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hOptsMatchDiff, obj.hOpts, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsMatchSeqs, ...
                obj.hOptsMatchDiff, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsMatchSeqs, ...
                obj.hOptsMatchDiff, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsMatchMatches, ...
                obj.hOptsMatchSeqs, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsMatchMatches, ...
                obj.hOptsMatchSeqs, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsMatchSelect, ...
                obj.hOptsMatchMatches, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsMatchSelect, ...
                obj.hOptsMatchMatches, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsMatchSelectValue, ...
                obj.hOptsMatchSelect, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsMatchSelectValue, ...
                obj.hOptsMatchSelect, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsMatchTweak, ...
                obj.hOptsMatchSelect, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hOptsMatchTweak, ...
                obj.hOpts, SpecPosition.RIGHT, GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hOptsPRGroundTruth, obj.hOpts, ...
                SpecPosition.TOP, 1.75*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hOptsPRGroundTruth, obj.hOpts, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsPRGroundTruthDetails, ...
                obj.hOptsPRGroundTruth, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPRGroundTruthDetails, ...
                obj.hOptsPRGroundTruth, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsPRSweepNumValue, ...
                obj.hOptsPRGroundTruth, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hOptsPRSweepNumValue, obj.hOpts, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsPRSweepNum, ...
                obj.hOptsPRGroundTruth, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPRSweepNum, ...
                obj.hOptsPRSweepNumValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsPRSweepVarValue, ...
                obj.hOptsPRGroundTruth, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPRSweepVarValue, ...
                obj.hOptsPRSweepNum, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsPRSweepVar, ...
                obj.hOptsPRGroundTruth, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsPRSweepVar, ...
                obj.hOptsPRSweepVarValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hOptsPRError, obj.hFig, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hOptsPRError, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hOptsVidRate, obj.hOpts, ...
                SpecPosition.TOP, 1.75*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hOptsVidRate, obj.hOpts, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsVidRateValue, ...
                obj.hOptsVidRate, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsVidRateValue, ...
                obj.hOptsVidRate, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hOptsVidPlay, ...
                obj.hOptsVidRate, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsVidPlay, ...
                obj.hOptsVidRateValue, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsVidSlider, ...
                obj.hOptsVidRate, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hOptsVidSlider, ...
                obj.hOptsVidPlay, SpecPosition.RIGHT_OF, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hOptsVidExport, ...
                obj.hOptsVidRate, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hOptsVidExport, obj.hOpts, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hAxA, obj.hOpts, ...
                SpecPosition.BELOW, 8*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxA, obj.hOpts, ...
                SpecPosition.LEFT, 12*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxB, obj.hOpts, ...
                SpecPosition.BELOW, 8*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxB, obj.hOpts, ...
                SpecPosition.RIGHT, 12*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxC, obj.hAxA, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxC, obj.hOpts, ...
                SpecPosition.LEFT, 12*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxD, obj.hAxB, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxD, obj.hOpts, ...
                SpecPosition.RIGHT, 12*GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hAxMain, obj.hOpts, ...
                SpecPosition.BELOW, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxMain, obj.hFig, ...
                SpecPosition.LEFT, 4*GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hAxPR, obj.hOpts, ...
                SpecPosition.BELOW, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxPR, obj.hOpts, ...
                SpecPosition.LEFT, 4*GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hAxVideo, obj.hOpts, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxVideo, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hFocus, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hFocus, obj.hOpts, ...
                SpecPosition.BELOW, 1.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusAx, obj.hFocus, ...
                SpecPosition.TOP, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusAx, obj.hFocus, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hFocusButton, obj.hFocusAx, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusButton, obj.hFocus, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hFocusQueryAx, ...
                obj.hFocus, SpecPosition.BOTTOM);
            SpecPosition.positionIn(obj.hFocusQuery, obj.hFocus, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hFocusQuery, ...
                obj.hFocusQueryAx, SpecPosition.ABOVE);
            SpecPosition.positionIn(obj.hFocusQuery, obj.hFocus, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hFocusRefAx, obj.hFocusQuery, ...
                SpecPosition.ABOVE);
            SpecPosition.positionIn(obj.hFocusRef, obj.hFocus, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hFocusRef, obj.hFocusRefAx, ...
                SpecPosition.ABOVE);
            SpecPosition.positionIn(obj.hFocusRef, obj.hFocus, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hFocusPR, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hFocusPR, obj.hOpts, ...
                SpecPosition.BELOW, 1.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusPRVisualise, obj.hFocusPR, ...
                SpecPosition.TOP, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusPRVisualise, obj.hFocusPR, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hFocusPRSlider, ...
                obj.hFocusPRVisualise, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusPRSlider, obj.hFocusPR, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hFocusPRRecall, obj.hFocusPR, ...
                SpecPosition.BOTTOM, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFocusPRRecall, obj.hFocusPR, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hFocusPRRecallValue, ...
                obj.hFocusPRRecall, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hFocusPRRecallValue, ...
                obj.hFocusPRRecall, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hFocusPRPrecision, ...
                obj.hFocusPRRecall, SpecPosition.ABOVE, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hFocusPRPrecision, ...
                obj.hFocusPRRecall, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hFocusPRPrecisionValue, ...
                obj.hFocusPRPrecision, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hFocusPRPrecisionValue, ...
                obj.hFocusPRPrecision, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hFocusPRVal, ...
                obj.hFocusPRPrecision, SpecPosition.ABOVE);
            SpecPosition.positionRelative(obj.hFocusPRVal, ...
                obj.hFocusPRRecall, SpecPosition.LEFT, ...
                -1*GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hFocusPRValValue, ...
                obj.hFocusPRVal, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hFocusPRValValue, ...
                obj.hFocusPRVal, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
        end

        function toggleVideoScreenLock(obj, locked)
            if locked
                state = 'off';
            else
                state = 'on';
            end
            obj.hSaveResults.Enable = state;
            obj.hScreen.Enable = state;
            obj.hOptsVidRateValue.Enable = state;
            obj.hOptsVidPlay.Enable = state;
            obj.hOptsVidExport.Enable = state;

            % Handle slider special case (if only 1 match... never enable)
            if sum(~isnan(obj.results.matching.selected.mask)) <= 1
                obj.hOptsVidSlider.Enable = 'off';
            else
                obj.hOptsVidSlider.Enable = state;
            end
        end

        function updateGroundTruthDescription(obj)
            [obj.hOptsPRGroundTruthDetails.String, ...
                obj.hOptsPRGroundTruthDetails.ForegroundColor] = ...
                GroundTruthPopup.gtDescription(obj.config.ground_truth);
        end

        function updateMatches(obj)
            obj.populateMatchList();
            obj.hOptsMatchSelectValue.String = obj.listMatches;
        end

        function updatePrecisionRecall(obj)
            % Bail if we do are not in a valid configuration
            if isequal(obj.hOptsPRGroundTruthDetails.ForegroundColor, ...
                    GUISettings.COL_ERROR)
                return;
            end

            % Get all configuration parameters (method, start, end, # steps)
            method = obj.config.seqslam.matching.method;
            us = [];
            if strcmp(method, 'thresh')
                bestScores = min(obj.results.matching.all.min_scores);
                sweepStart = max(bestScores);
                sweepEnd = min(bestScores);
            else
                us = SeqSLAMInstance.usFromMatches( ...
                    obj.results.matching.all.min_scores, ...
                    obj.config.seqslam.matching.method_window.r_window);
                sweepStart = 1;
                sweepEnd = max(us);
            end
            numSteps = str2num(obj.hOptsPRSweepNumValue.String);

            % Get each of the variable sweep values
            varVals = linspace(sweepStart, sweepEnd, numSteps);

            % Get the matches for each of the variable sweep values
            matches = cell(size(varVals));
            for k = 1:length(varVals)
                if strcmp(method, 'thresh')
                    thresholded = SeqSLAMInstance.thresholdBasic( ...
                        obj.results.matching.all, varVals(k));
                else
                    thresholded = SeqSLAMInstance.thresholdWindowed( ...
                        obj.results.matching.all, ...
                        obj.config.seqslam.matching.method_window.r_window, ...
                        varVals(k));
                end
                matches{k} = thresholded.matches;
            end

            % Calculate precision / recall for each variable value
            ps = zeros(size(matches));
            rs = zeros(size(matches));
            for k = 1:length(matches)
                [ps(k), rs(k)] = calcPR(matches{k}, ...
                    obj.config.ground_truth.matrix);
            end

            % Save the results
            obj.results.pr.sweep_var.start = sweepStart;
            obj.results.pr.sweep_var.end = sweepEnd;
            obj.results.pr.sweep_var.num_steps = numSteps;
            obj.results.pr.values.precisions = ps;
            obj.results.pr.values.recalls = rs;
        end

        function updateSelectedMatch(obj)
            v = obj.hOptsMatchSelectValue.Value;
            if v == 1
                % Store an empty matrix if all are selected
                m = [];
            else
                % Get the match indices
                mIs = find(~isnan(obj.results.matching.selected.mask));

                % Stor the image # for query and reference ([mQ, mR])
                obj.selectedMatch = [mIs(v-1) ...
                    obj.results.matching.selected.matches(mIs(v-1))];
            end
        end

        function updateValueExplorerSlider(obj)
            % Set the intervals to reflect the number of variable values in the
            % plot
            smallStep = 1/(str2num(obj.hOptsPRSweepNumValue.String)-1);
            if smallStep < 0.1
                obj.hFocusPRSlider.SliderStep = [smallStep 0.1];
            else
                obj.hFocusPRSlider.SliderStep = [0.1 0.1];
            end

            % Set it back to the start value, and call the callback
            obj.hFocusPRSlider.Value = 0;
            obj.cbUpdatePRFocusValues(obj.hFocusPRSlider, []);
        end
    end
end
