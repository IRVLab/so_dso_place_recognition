classdef TweakMatchesPopup < handle

    properties (Constant)
        FIG_WIDTH_FACTOR = 5;
        FIG_HEIGHT_FACTOR = 25;
    end

    properties
        hFig;
        hHelp;

        hTitle;

        hMethod;
        hMethodValue;

        hWindowWidth;
        hWindowWidthValue;
        hWindowU;
        hWindowUValue;

        hThresh;
        hThreshValue;

        hAxis;
        hLine;

        hApply;

        config = emptyConfig();
        results = emptyResults();
    end

    methods
        function obj = TweakMatchesPopup(config, results)
            % Save the provided data
            obj.config = config;
            obj.results = results;

            % Create and size the popup
            obj.createPopup();
            obj.sizePopup();

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);
            HelpPopup.setDestination(obj.hHelp, ...
                'tweaking');

            % Populate the UI (drawing happens after poulating selects a method)
            obj.populate();

            % Finally, show the figure once done configuring
            obj.hFig.Visible = 'on';
        end
    end

    methods (Access = private)
        function cbApplyThreshold(obj, src, data)
            % Get the new thresholded matches
            if obj.hMethodValue.Value == 2
                obj.results.matching.selected = ...
                    SeqSLAMInstance.thresholdBasic(obj.results.matching.all, ...
                    str2num(obj.hThreshValue.String));
            else
                obj.results.matching.selected = ...
                    SeqSLAMInstance.thresholdWindowed(...
                    obj.results.matching.all, ...
                    str2num(obj.hWindowWidthValue.String), ...
                    str2num(obj.hWindowUValue.String));
            end

            % Strip the provided config out into the saved config
            obj.strip();

            % Close the figure
            close(obj.hFig)
        end

        function cbSelectMethod(obj, src, data)
            % Choose the UI elements to display
            if obj.hMethodValue.Value == 2
                obj.hWindowWidth.Visible = 'off';
                obj.hWindowWidthValue.Visible = 'off';
                obj.hWindowU.Visible = 'off';
                obj.hWindowUValue.Visible = 'off';
                obj.hThresh.Visible = 'on';
                obj.hThreshValue.Visible = 'on';
            else
                obj.hWindowWidth.Visible = 'on';
                obj.hWindowWidthValue.Visible = 'on';
                obj.hWindowU.Visible = 'on';
                obj.hWindowUValue.Visible = 'on';
                obj.hThresh.Visible = 'off';
                obj.hThreshValue.Visible = 'off';
            end

            % Refresh the graphs
            obj.drawScreen();
        end

        function cbUpdateThresh(obj, src, data)
            % Validate and update
            obj.validate(obj.hThreshValue, obj.hAxis.YLim(1), ...
                obj.hAxis.YLim(2));
            obj.drawScreen();
        end

        function cbUpdateWindowU(obj, src, data)
            % Validate and update
            us = SeqSLAMInstance.usFromMatches( ...
                obj.results.matching.all.min_scores, ...
                str2num(obj.hWindowWidthValue.String));
            obj.validate(obj.hWindowUValue, 1, max(us));
            obj.drawScreen();
        end

        function cbUpdateWindowWidth(obj, src, data)
            % Validate and update
            obj.validate(obj.hWindowWidthValue, 1, range(obj.hAxis.XLim));
            obj.drawScreen();
        end

        function posOut = constrainedPosition(obj, posIn)
            posOut = posIn;

            % Allow no changes if resizing
            if obj.hMethodValue.Value == 2
                v = str2num(obj.hThreshValue.String);
            else
                v = str2num(obj.hWindowUValue.String);
            end
            c = obj.hLine.getPosition();
            if abs(range(c(:,2))-range(posOut(:,2))) > 0.001 || ...
                    abs(range(c(:,1))-range(obj.hAxis.XLim)) > 0.001
                posOut(:,1) = obj.hAxis.XLim;
                posOut(:,2) = v;
                return;
            end

            % Force to be within y axis limits
            if obj.hMethodValue.Value == 2
                yMin = 0;
            else
                yMin = 1;
            end
            posOut(1,2) = max(posOut(1,2), yMin);
            posOut(1,2) = min(posOut(1,2), obj.hAxis.YLim(2));
            posOut(2,2) = posOut(1,2);

            % Force to remain at x axis limits
            posOut(:,1) = obj.hAxis.XLim;

            % Set the new value and call a redraw TODO maybe a little heavy...
            if obj.hMethodValue.Value == 2
                obj.hThreshValue.String = num2str(posOut(1,2));
            else
                obj.hWindowUValue.String = num2str(posOut(1,2));
            end
            obj.drawScreen();
        end

        function createPopup(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'Match Selection Tweaker';

            % Create the title
            obj.hTitle = uicontrol('Style', 'text');
            obj.hTitle.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hTitle);
            GUISettings.setFontScale(obj.hTitle, 1.5);
            obj.hTitle.String = 'Tweak Match Selection Method';

            % Create the configuration UI elements
            obj.hMethod = uicontrol('Style', 'text');
            obj.hMethod.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMethod);
            obj.hMethod.String = 'Selection Method:';

            obj.hMethodValue = uicontrol('Style', 'popupmenu');
            obj.hMethodValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMethodValue);
            obj.hMethodValue.String = ...
                {'Windowed Uniqueness' 'Basic Thresholding'};

            obj.hWindowWidth = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hWindowWidth);
            obj.hWindowWidth.String = 'Window width ($r_{window}$):';

            obj.hWindowWidthValue = uicontrol('Style', 'edit');
            obj.hWindowWidthValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hWindowWidthValue);
            obj.hWindowWidthValue.String = '';

            obj.hWindowU = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hWindowU);
            obj.hWindowU.String = 'Uniqueness factor ($\mu$):';

            obj.hWindowUValue = uicontrol('Style', 'edit');
            obj.hWindowUValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hWindowUValue);
            obj.hWindowUValue.String = '';

            obj.hThresh = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hThresh);
            obj.hThresh.String = 'Threshold ($\lambda$):';

            obj.hThreshValue = uicontrol('Style', 'edit');
            obj.hThreshValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hThreshValue);
            obj.hThreshValue.String = '';

            % Create the axis
            obj.hAxis = axes();
            GUISettings.applyUIAxesStyle(obj.hAxis);
            obj.hAxis.Visible = 'off';

            % Create the apply button
            obj.hApply = uicontrol('Style', 'pushbutton');
            obj.hApply.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hApply);
            obj.hApply.String = 'Apply new threshold';

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hMethodValue.Callback = {@obj.cbSelectMethod};
            obj.hWindowWidthValue.Callback = {@obj.cbUpdateWindowWidth};
            obj.hWindowUValue.Callback = {@obj.cbUpdateWindowU};
            obj.hThreshValue.Callback = {@obj.cbUpdateThresh};
            obj.hApply.Callback = {@obj.cbApplyThreshold};
        end

        function drawScreen(obj)
            % Get the data that we will be plotting
            if obj.hMethodValue.Value == 2
                data = min(obj.results.matching.all.min_scores);
                v = str2num(obj.hThreshValue.String);
                ylim = [min([data v]) max(data)];
            else
                data = SeqSLAMInstance.usFromMatches( ...
                    obj.results.matching.all.min_scores, ...
                    str2num(obj.hWindowWidthValue.String));
                v = str2num(obj.hWindowUValue.String);
                ylim = [1 max(data)];
            end
            if any(isnan(ylim))
                return;
            end
            xlim = [1 length(data)];
            ylim = ylim + [-1 1] * range(ylim) * 0.05;

            % Plot the data, and draw the overlay components
            cla(obj.hAxis);
            hold(obj.hAxis, 'on');
            plot(obj.hAxis, data, '.');
            if obj.hMethodValue.Value == 2
                coords = [xlim(1) ylim(1) range(xlim) v - ylim(1)];
            else
                coords = [xlim(1) v range(xlim) abs(ylim(2) - v)];
            end
            if coords(3) > 0 && coords(4) > 0
                r = rectangle(obj.hAxis, 'Position', coords);
                r.LineStyle = 'none';
                r.FaceColor = [GUISettings.COL_SUCCESS 0.25];
            end
            obj.hLine = imline(obj.hAxis, xlim, [v v]);
            obj.hLine.setColor(GUISettings.COL_SUCCESS);
            obj.hLine.setPositionConstraintFcn(@obj.constrainedPosition);
            hold(obj.hAxis, 'off');

            % Configure the axis
            obj.hAxis.Visible = 'on';
            obj.hAxis.Box = 'off';
            obj.hAxis.XLim = xlim;
            obj.hAxis.YLim = ylim;
        end

        function populate(obj)
            % Load the values in from the results
            obj.hWindowWidthValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.matching.method_window.r_window, 10);
            obj.hWindowUValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.matching.method_window.u, 1.111);
            obj.hThreshValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.matching.method_thresh.threshold, 2);
            method = SafeData.noEmpty(obj.config.seqslam.matching.method, ...
                'window');
            if strcmpi(method, 'thresh')
                obj.hMethodValue.Value = 2;
            else
                obj.hMethodValue.Value = 1;
            end

            % Execute any required callbacks manually
            obj.cbSelectMethod(obj.hMethodValue, []);
        end

        function sizePopup(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            widthUnit = obj.hTitle.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hTitle.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * TweakMatchesPopup.FIG_WIDTH_FACTOR, ...
                heightUnit * TweakMatchesPopup.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            SpecSize.size(obj.hMethod, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hMethodValue, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_LARGE);

            SpecSize.size(obj.hWindowWidth, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.15);
            SpecSize.size(obj.hWindowWidthValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);
            SpecSize.size(obj.hWindowU, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.15);
            SpecSize.size(obj.hWindowUValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);

            SpecSize.size(obj.hThresh, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.15);
            SpecSize.size(obj.hThreshValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);

            SpecSize.size(obj.hAxis, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.9);
            SpecSize.size(obj.hAxis, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.75);

            SpecSize.size(obj.hApply, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.2);

            % Then, systematically place
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionRelative(obj.hMethod, obj.hTitle, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMethod, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMethodValue, obj.hMethod, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMethodValue, obj.hMethod, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hWindowWidth, obj.hMethod, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hWindowWidth, ...
                obj.hMethodValue, SpecPosition.RIGHT_OF, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hWindowWidthValue, ...
                obj.hMethod, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hWindowWidthValue, ...
                obj.hWindowWidth, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hWindowU, obj.hMethod, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hWindowU, ...
                obj.hWindowWidthValue, SpecPosition.RIGHT_OF, ...
                4*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hWindowUValue, obj.hMethod, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hWindowUValue, obj.hWindowU, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hThresh, obj.hMethod, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hThresh, obj.hMethodValue, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hThreshValue, obj.hMethod, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hThreshValue, obj.hThresh, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hAxis, obj.hMethod, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxis, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hApply, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hApply, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
        end

        function strip(obj)
            % ONLY strip out the values for the currently selected method
            if obj.hMethodValue.Value == 2
                obj.config.seqslam.matching.method = 'thresh';
                obj.config.seqslam.matching.method_thresh.threshold = ...
                    str2num(obj.hThreshValue.String);
            else
                obj.config.seqslam.matching.method = 'window';
                obj.config.seqslam.matching.method_window.r_window = ...
                    str2num(obj.hWindowWidthValue.String);
                obj.config.seqslam.matching.method_window.u = ...
                    str2num(obj.hWindowUValue.String);
            end
        end

        function validate(obj, edit, a, b)
            % Do some lazy validity checking (end enforce in UI)
            v = str2num(edit.String);
            if length(v) ~= 1 || v < a
                v = a;
            elseif v > b
                v = b;
            end
            edit.String = num2str(v);
        end
    end
end
