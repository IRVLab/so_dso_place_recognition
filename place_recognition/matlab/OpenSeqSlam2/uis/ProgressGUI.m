classdef ProgressGUI < handle

    properties (Constant)
        FIG_WIDTH_FACTOR = 4.5;
        FIG_HEIGHT_FACTOR = 20;
    end

    properties
        hFig;
        hHelp;

        hStatus1;
        hStatus2;
        hStatus3;
        hStatus4;
        hStatus5;
        hStatus6;
        hStatus12;
        hStatus23;
        hStatus34;
        hStatus45;
        hStatus56;

        hTitle;
        hSubtitle;

        hAxA;
        hAxB;
        hAxC;
        hAxD;
        hAxMain;
        hAxOverlay;

        hPercent;

        config = emptyConfig();
        progress;

        lastPercentRefresh = 0;
        lastMainRefresh = 0;

        instance;

        results = emptyResults();
    end

    methods
        function obj = ProgressGUI(config)
            % Create and size the GUI
            obj.createGUI();
            obj.sizeGUI();

            % Save the config
            obj.config = config;

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);
            HelpPopup.setDestination(obj.hHelp, 'progress');

            % Create an initial progress state
            progress = [];
            progress.state = SeqSLAMInstance.STATE_START;
            progress.percent = 0;
            obj.refreshMain(progress);

            % Create, and attach to, the SeqSLAM instance
            obj.instance = SeqSLAMInstance(config);
            obj.instance.attachUI(obj);

            % Finally, show the figure when we are done configuring
            obj.hFig.Visible = 'on';
        end

        function due = refreshPercentDue(obj, state, perc)
            rate = obj.config.ui.progress.percent_freq;
            due = state ~= obj.progress.state || ...
                floor(perc / rate) > floor(obj.lastPercentRefresh / rate);
        end

        function due = refreshMainDue(obj, state, perc)
            if (state == SeqSLAMInstance.STATE_PREPROCESS_REF || ...
                    state == SeqSLAMInstance.STATE_PREPROCESS_QUERY)
                rate = obj.config.ui.progress.preprocess_freq;
            elseif (state == SeqSLAMInstance.STATE_DIFF_MATRIX)
                rate = obj.config.ui.progress.diff_matrix_freq;
            elseif (state == SeqSLAMInstance.STATE_DIFF_MATRIX_CONTRAST)
                rate = obj.config.ui.progress.enhance_freq;
            elseif (state == SeqSLAMInstance.STATE_MATCHING)
                rate = obj.config.ui.progress.match_freq;
            else
                rate = 1;
            end
            due = state ~= obj.progress.state || ...
                floor(perc / rate) > floor(obj.lastMainRefresh / rate);
        end

        function refreshPercent(obj, percent)
            % Update the percent internally and visually
            obj.progress.percent = percent;
            obj.updatePercent();

            % Force a draw
            drawnow();
            obj.lastPercentRefresh = percent;
        end

        function refreshMain(obj, progress)
            % Refresh the last percent if we are in a new state
            if ~isempty(obj.progress) && progress.state ~= obj.progress.state
                obj.lastPercentRefresh = 0;
            end

            % Save the new progress
            obj.progress = progress;

            % Update each of the elements
            obj.updateStatusBar();
            obj.updateTitle();
            obj.updatePlots();
            obj.updatePercent();

            % Force a draw
            drawnow();
            obj.lastMainRefresh = progress.percent;
        end

        function run(obj)
            obj.instance.run();
            obj.results = obj.instance.results;

            % Close when done
            % TODO maybe there is a more appropriate place for this?
            delete(obj.hFig);
        end
    end

    methods (Access = private)
        function cbClose(obj, src, event)

        end

        function clearScreen(obj)
            cla(obj.hAxA);
            cla(obj.hAxB);
            cla(obj.hAxC);
            cla(obj.hAxD);
            cla(obj.hAxMain);
            cla(obj.hAxOverlay);

            obj.hAxA.Visible = 'off';
            obj.hAxB.Visible = 'off';
            obj.hAxC.Visible = 'off';
            obj.hAxD.Visible = 'off';
            obj.hAxMain.Visible = 'off';
            obj.hAxOverlay.Visible = 'off';

            obj.hAxA.Title.Visible = 'off';
            obj.hAxB.Title.Visible = 'off';
            obj.hAxC.Title.Visible = 'off';
            obj.hAxD.Title.Visible = 'off';
            obj.hAxMain.Title.Visible = 'off';
            obj.hAxOverlay.Title.Visible = 'off';
        end

        function createGUI(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'OpenSeqSLAM2.0 Progress';

            % Status bar elements
            obj.hStatus1 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus1);
            GUISettings.setFontScale(obj.hStatus1, 1.5);
            obj.hStatus1.FontWeight = 'bold';
            obj.hStatus1.String = '1';

            obj.hStatus2 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus2);
            GUISettings.setFontScale(obj.hStatus2, 1.5);
            obj.hStatus2.FontWeight = 'bold';
            obj.hStatus2.String = '2';

            obj.hStatus3 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus3);
            GUISettings.setFontScale(obj.hStatus3, 1.5);
            obj.hStatus3.FontWeight = 'bold';
            obj.hStatus3.String = '3';

            obj.hStatus4 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus4);
            GUISettings.setFontScale(obj.hStatus4, 1.5);
            obj.hStatus4.FontWeight = 'bold';
            obj.hStatus4.String = '4';

            obj.hStatus5 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus5);
            GUISettings.setFontScale(obj.hStatus5, 1.5);
            obj.hStatus5.FontWeight = 'bold';
            obj.hStatus5.String = '5';

            obj.hStatus6 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus6);
            GUISettings.setFontScale(obj.hStatus6, 1.5);
            obj.hStatus6.FontWeight = 'bold';
            obj.hStatus6.String = '6';

            obj.hStatus12 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus12);
            GUISettings.setFontScale(obj.hStatus12, 1.5);
            obj.hStatus12.FontWeight = 'bold';
            obj.hStatus12.String = ' - ';

            obj.hStatus23 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus23);
            GUISettings.setFontScale(obj.hStatus23, 1.5);
            obj.hStatus23.FontWeight = 'bold';
            obj.hStatus23.String = ' - ';

            obj.hStatus34 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus34);
            GUISettings.setFontScale(obj.hStatus34, 1.5);
            obj.hStatus34.FontWeight = 'bold';
            obj.hStatus34.String = ' - ';

            obj.hStatus45 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus45);
            GUISettings.setFontScale(obj.hStatus45, 1.5);
            obj.hStatus45.FontWeight = 'bold';
            obj.hStatus45.String = ' - ';

            obj.hStatus56 = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hStatus56);
            GUISettings.setFontScale(obj.hStatus56, 1.5);
            obj.hStatus56.FontWeight = 'bold';
            obj.hStatus56.String = ' - ';

            % Title and subtitle
            obj.hTitle = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hTitle);
            GUISettings.setFontScale(obj.hTitle, 2.5);
            obj.hTitle.FontWeight = 'bold';
            obj.hTitle.String = 'Testing 1 2 3';

            obj.hSubtitle = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hSubtitle);
            GUISettings.setFontScale(obj.hSubtitle, 1.5);
            obj.hSubtitle.String = 'Testing 1 2 3';

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

            obj.hAxOverlay = axes();
            GUISettings.applyUIAxesStyle(obj.hAxOverlay);
            obj.hAxOverlay.Visible = 'off';

            % Percent
            obj.hPercent = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hPercent);
            GUISettings.setFontScale(obj.hPercent,2);
            obj.hPercent.FontWeight = 'bold';
            obj.hPercent.HorizontalAlignment = 'right';
            obj.hPercent.String = '50%';

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hFig.CloseRequestFcn = {@obj.cbClose};
        end

        function sizeGUI(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            widthUnit = obj.hTitle.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hTitle.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * ProgressGUI.FIG_WIDTH_FACTOR, ...
                heightUnit * ProgressGUI.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hStatus1, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus1, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus2, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus2, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus3, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus3, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus4, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus4, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus5, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus5, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus6, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus6, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus12, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus12, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus23, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus23, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus34, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus34, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus45, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus45, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);
            SpecSize.size(obj.hStatus56, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hStatus56, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.08);

            SpecSize.size(obj.hTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hSubtitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hSubtitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_SMALL);

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
                obj.hFig, 0.7);
            SpecSize.size(obj.hAxMain, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);
            SpecSize.size(obj.hAxOverlay, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.7);
            SpecSize.size(obj.hAxOverlay, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);

            SpecSize.size(obj.hPercent, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hPercent, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.15, GUISettings.PAD_MED);

            % Then, systematically place
            SpecPosition.positionIn(obj.hStatus34, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hStatus34, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionRelative(obj.hStatus3, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus3, obj.hStatus34, ...
                SpecPosition.LEFT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus23, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus23, obj.hStatus3, ...
                SpecPosition.LEFT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus2, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus2, obj.hStatus23, ...
                SpecPosition.LEFT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus12, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus12, obj.hStatus2, ...
                SpecPosition.LEFT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus1, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus1, obj.hStatus12, ...
                SpecPosition.LEFT_OF, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hStatus4, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus4, obj.hStatus34, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus45, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus45, obj.hStatus4, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus5, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus5, obj.hStatus45, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus56, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus56, obj.hStatus5, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hStatus6, obj.hStatus34, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hStatus6, obj.hStatus56, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hTitle, obj.hStatus34, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hSubtitle, obj.hTitle, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hSubtitle, obj.hFig, ...
                SpecPosition.LEFT, 1.5*GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hAxA, obj.hSubtitle, ...
                SpecPosition.BELOW, 8*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxA, obj.hSubtitle, ...
                SpecPosition.LEFT, 12*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxB, obj.hSubtitle, ...
                SpecPosition.BELOW, 8*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxB, obj.hSubtitle, ...
                SpecPosition.RIGHT, 12*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxC, obj.hAxA, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxC, obj.hSubtitle, ...
                SpecPosition.LEFT, 12*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxD, obj.hAxB, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hAxD, obj.hSubtitle, ...
                SpecPosition.RIGHT, 12*GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hAxMain, obj.hSubtitle, ...
                SpecPosition.BELOW, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxMain, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hAxOverlay, obj.hSubtitle, ...
                SpecPosition.BELOW, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxOverlay, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hPercent, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hPercent, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
        end

        function updatePercent(obj)
            if ~isfield(obj.progress, 'percent') || isempty(obj.progress.percent)
                obj.hPercent.String = '';
            else
                obj.hPercent.String = [num2str(round(obj.progress.percent)) '%'];
            end
        end

        function updatePlots(obj)
            % Clear the screen
            obj.clearScreen();

            % Draw the new screen
            if obj.progress.state == SeqSLAMInstance.STATE_PREPROCESS_REF || ...
                    obj.progress.state == SeqSLAMInstance.STATE_PREPROCESS_QUERY
                % Plot the 4 images
                imshow(obj.progress.image_init, 'Parent', obj.hAxA);
                imshow(obj.progress.image_grey, 'Parent', obj.hAxB);
                imshow(obj.progress.image_crop_resized, 'Parent', obj.hAxC);
                imshow(obj.progress.image_out, 'Parent', obj.hAxD);

                % Style the plots
                obj.hAxA.Title.String = ['Original (' ...
                    num2str(size(obj.progress.image_init, 2)) 'x' ...
                    num2str(size(obj.progress.image_init, 1)) ')'];
                obj.hAxB.Title.String = ['Greyscale (' ...
                    num2str(size(obj.progress.image_grey, 2)) 'x' ...
                    num2str(size(obj.progress.image_grey, 1)) ')'];
                obj.hAxC.Title.String = ['Resized and cropped (' ...
                    num2str(size(obj.progress.image_crop_resized, 2)) 'x' ...
                    num2str(size(obj.progress.image_crop_resized, 1)) ')'];
                obj.hAxD.Title.String = ['Patch normalised (' ...
                    num2str(size(obj.progress.image_out, 2)) 'x' ...
                    num2str(size(obj.progress.image_out, 1)) ')'];
                GUISettings.axesHide(obj.hAxA);
                GUISettings.axesHide(obj.hAxB);
                GUISettings.axesHide(obj.hAxC);
                GUISettings.axesHide(obj.hAxD);
            elseif obj.progress.state == SeqSLAMInstance.STATE_DIFF_MATRIX
                % Draw the difference matrix
                pcolor(obj.hAxMain, obj.progress.diff_matrix);
                shading(obj.hAxMain, 'flat');

                % Style the plot
                GUISettings.axesDiffMatrixStyle(obj.hAxMain, ...
                    size(obj.progress.diff_matrix));
            elseif obj.progress.state == SeqSLAMInstance.STATE_DIFF_MATRIX_CONTRAST
                % Draw the difference matrices
                pcolor(obj.hAxMain, obj.progress.diff_matrix);
                shading(obj.hAxMain, 'flat');
                pcolor(obj.hAxOverlay, obj.progress.diff_matrix_enhanced);
                shading(obj.hAxOverlay, 'flat');
                obj.hAxOverlay.Color = 'none';

                % Style the plot
                GUISettings.axesDiffMatrixStyle(obj.hAxMain, ...
                    size(obj.progress.diff_matrix));
                GUISettings.axesDiffMatrixStyle(obj.hAxOverlay, ...
                    size(obj.progress.diff_matrix));
            elseif obj.progress.state == SeqSLAMInstance.STATE_MATCHING
                % Draw the background difference matrix
                imagesc(obj.hAxMain, obj.progress.diff_matrix);
                hold(obj.hAxMain, 'on');
                plot(obj.hAxMain, obj.progress.qs, obj.progress.rs, ...
                    'Color', GUISettings.COL_ERROR);
                plot(obj.hAxMain, [obj.progress.q obj.progress.q], ...
                    [1 size(obj.progress.diff_matrix,1)], 'Color', ...
                    GUISettings.COL_ERROR, 'LineStyle', '--');
                h = plot(obj.hAxMain, obj.progress.best_scores, '.', ...
                    'Color', GUISettings.COL_ERROR);
                h.MarkerSize = h.MarkerSize * 1.5;
                hold(obj.hAxMain, 'off');

                % Style the plot
                GUISettings.axesDiffMatrixStyle(obj.hAxMain, ...
                    size(obj.progress.diff_matrix));
            elseif obj.progress.state == SeqSLAMInstance.STATE_MATCHING_FILTERING
                % TODO can't see any reason to do anything in here (too quick)
            elseif obj.progress.state == SeqSLAMInstance.STATE_DONE
                % Should never be plotting here...
            end
        end

        function updateStatusBar(obj)
            % Default everything to grey
            obj.hStatus1.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus2.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus3.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus4.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus5.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus6.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus12.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus23.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus34.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus45.ForegroundColor = GUISettings.COL_LOADING;
            obj.hStatus56.ForegroundColor = GUISettings.COL_LOADING;

            % Change colors based on progression through states
            if obj.progress.state == SeqSLAMInstance.STATE_PREPROCESS_REF
                obj.hStatus1.ForegroundColor = GUISettings.COL_DEFAULT;
            end
            if obj.progress.state > SeqSLAMInstance.STATE_PREPROCESS_REF
                obj.hStatus1.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus12.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus2.ForegroundColor = GUISettings.COL_DEFAULT;
            end
            if obj.progress.state > SeqSLAMInstance.STATE_PREPROCESS_QUERY
                obj.hStatus2.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus23.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus3.ForegroundColor = GUISettings.COL_DEFAULT;
            end
            if obj.progress.state > SeqSLAMInstance.STATE_DIFF_MATRIX
                obj.hStatus3.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus34.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus4.ForegroundColor = GUISettings.COL_DEFAULT;
            end
            if obj.progress.state > SeqSLAMInstance.STATE_DIFF_MATRIX_CONTRAST
                obj.hStatus4.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus45.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus5.ForegroundColor = GUISettings.COL_DEFAULT;
            end
            if obj.progress.state > SeqSLAMInstance.STATE_MATCHING
                obj.hStatus5.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus56.ForegroundColor = GUISettings.COL_SUCCESS;
                obj.hStatus6.ForegroundColor = GUISettings.COL_DEFAULT;
            end
            if obj.progress.state > SeqSLAMInstance.STATE_MATCHING_FILTERING
                obj.hStatus6.ForegroundColor = GUISettings.COL_SUCCESS;
            end
        end

        function updateTitle(obj)
            if obj.progress.state == SeqSLAMInstance.STATE_PREPROCESS_REF
                obj.hTitle.Visible = 'on';
                obj.hTitle.String = '1 - Preprocessing Reference Images';
                obj.hSubtitle.Visible = 'on';
                obj.hSubtitle.String = obj.progress.image_details;
            elseif obj.progress.state == SeqSLAMInstance.STATE_PREPROCESS_QUERY
                obj.hTitle.Visible = 'on';
                obj.hTitle.String = '2 - Preprocessing Query Images';
                obj.hSubtitle.Visible = 'on';
                obj.hSubtitle.String = obj.progress.image_details;
            elseif obj.progress.state == SeqSLAMInstance.STATE_DIFF_MATRIX
                obj.hTitle.Visible = 'on';
                obj.hTitle.String = '3 - Constructing Difference Matrix';
                obj.hSubtitle.Visible = 'off';
            elseif obj.progress.state == SeqSLAMInstance.STATE_DIFF_MATRIX_CONTRAST
                obj.hTitle.Visible = 'on';
                obj.hTitle.String = '4 - Enhancing Difference Matrix Contrast';
                obj.hSubtitle.Visible = 'off';
            elseif obj.progress.state == SeqSLAMInstance.STATE_MATCHING
                obj.hTitle.Visible = 'on';
                obj.hTitle.String = '5 - Searching for Matches';
                obj.hSubtitle.Visible = 'off';
            elseif obj.progress.state == SeqSLAMInstance.STATE_MATCHING_FILTERING
                obj.hTitle.Visible = 'on';
                obj.hTitle.String = '6 - Filtering Best Matches';
                obj.hSubtitle.Visible = 'off';
            elseif obj.progress.state == SeqSLAMInstance.STATE_DONE
                obj.hTitle.String = 'Done!';
                obj.hSubtitle.Visible = 'off';
            else
                obj.hTitle.Visible = 'off';
                obj.hSubtitle.Visible = 'off';
            end
        end
    end
end
