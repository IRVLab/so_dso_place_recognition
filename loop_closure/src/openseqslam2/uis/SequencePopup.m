classdef SequencePopup < handle

    properties (Constant)
        FIG_WIDTH_FACTOR = 3.5;
        FIG_HEIGHT_FACTOR = 25;
    end

    properties
        hFig;
        hHelp;

        hTitle;

        hQueryTitle;
        hQueryAxes = [];
        hRefTitle;
        hRefAxes = [];
        hScrollbar;

        config = emptyConfig();
        results = emptyResults();

        listRs = [];
        listQs = [];
        middleInd;

        axesWidth;
        figureWidth;
    end

    methods
        function obj = SequencePopup(qs, rs, config, results)
            % Save the provided data
            obj.config = config;
            obj.results = results;
            obj.listRs = rs;
            obj.listQs = qs;
            obj.middleInd = floor(length(rs) / 2) + 1;

            % Create and size the popup
            obj.createPopup();
            obj.sizePopup();

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);
            HelpPopup.setDestination(obj.hHelp, ...
                'sequence');

            % Draw the screen content (images)
            obj.drawScreen();

            % Scroll to the current position (middle)
            obj.cbScroll(obj.hScrollbar, []);

            % Finally, show the figure once done configuring
            obj.hFig.Visible = 'on';
        end
    end

    methods (Access = private, Static)
        function focusAxis(ax)
            uistack(ax, 'top');
            ax.XColor = GUISettings.COL_SUCCESS;
            ax.YColor = GUISettings.COL_SUCCESS;
            ax.XTick = [];
            ax.YTick = [];
            ax.LineWidth = 5;
            ax.Visible = 'on';
        end
    end

    methods (Access = private)
        function cbScroll(obj, src, event)
            % Get max scroll offset, and percent of the scroll on the bar
            maxScroll = obj.axesWidth - obj.figureWidth;
            percScroll = obj.hScrollbar.Value / ...
                (obj.hScrollbar.Max - obj.hScrollbar.Min);

            % Set the appropriate offset
            obj.setScollOffset(-1 * maxScroll * percScroll);
        end

        function createPopup(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'Matched Sequence Viewer';

            % Generic elements
            obj.hTitle = uicontrol('Style', 'text');
            GUISettings.applyUIControlStyle(obj.hTitle);
            GUISettings.setFontScale(obj.hTitle, 1.5);
            obj.hTitle.FontWeight = 'bold';
            obj.hTitle.String = ['Matched Sequence for Query Image #' ...
                num2str(obj.listQs(floor(end/2)+1))];

            % Create query elements
            obj.hQueryTitle = uicontrol('Style', 'text');
            obj.hQueryTitle.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hQueryTitle);
            GUISettings.setFontScale(obj.hQueryTitle, 1.25);
            obj.hQueryTitle.FontWeight = 'bold';
            obj.hQueryTitle.String = 'Query images';

            obj.hQueryAxes = gobjects(size(obj.listQs));
            for k = 1:length(obj.listQs)
                obj.hQueryAxes(k) = axes();
                GUISettings.applyUIAxesStyle(obj.hQueryAxes(k));
                obj.hQueryAxes(k).Visible = 'off';
            end

            % Create reference elments
            obj.hRefTitle = uicontrol('Style', 'text');
            obj.hRefTitle.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hRefTitle);
            GUISettings.setFontScale(obj.hRefTitle, 1.25);
            obj.hRefTitle.FontWeight = 'bold';
            obj.hRefTitle.String = 'Reference images';

            obj.hRefAxes = gobjects(size(obj.listRs));
            for k = 1:length(obj.listRs)
                obj.hRefAxes(k) = axes();
                GUISettings.applyUIAxesStyle(obj.hRefAxes(k));
                obj.hRefAxes(k).Visible = 'off';
            end

            % Create the bottom scrollbar
            obj.hScrollbar = uicontrol('Style', 'slider');
            obj.hScrollbar.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hScrollbar);
            obj.hScrollbar.Value = 0.5 + 0.05 * mod(length(obj.listQs) + 1, 2);

            % Callbacks (must be last, otherwise empty objects passed...)
            addlistener(obj.hScrollbar, 'Value', 'PostSet', @obj.cbScroll);
        end

        function drawScreen(obj)
            % Draw each of the query images in each of the axis
            for k = 1:length(obj.hQueryAxes)
                cla(obj.hQueryAxes(k));
                imshow(datasetOpenImage(obj.config.('query'), obj.listQs(k), ...
                    obj.results.preprocessed.('query_numbers')), ...
                    'Parent', obj.hQueryAxes(k));
            end

            % Draw each of the reference images in each of the axis
            for k = 1:length(obj.hRefAxes)
                cla(obj.hRefAxes(k));
                imshow(datasetOpenImage(obj.config.('reference'), ...
                    obj.listRs(k), ...
                    obj.results.preprocessed.('reference_numbers')), ...
                    'Parent', obj.hRefAxes(k));
            end

            % Draw visual focus for middle axis
            SequencePopup.focusAxis(obj.hQueryAxes(obj.middleInd));
            SequencePopup.focusAxis(obj.hRefAxes(obj.middleInd));
        end

        function sizePopup(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            widthUnit = obj.hTitle.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hTitle.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * SequencePopup.FIG_WIDTH_FACTOR, ...
                heightUnit * SequencePopup.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');
            obj.figureWidth = obj.hFig.Position(3);

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            SpecSize.size(obj.hQueryTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hQueryTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            for k = 1:length(obj.hQueryAxes)
                SpecSize.size(obj.hQueryAxes(k), SpecSize.HEIGHT, ...
                    SpecSize.PERCENT, obj.hFig, 0.35);
            end

            SpecSize.size(obj.hRefTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hRefTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            for k = 1:length(obj.hRefAxes)
                SpecSize.size(obj.hRefAxes(k), SpecSize.HEIGHT, ...
                    SpecSize.PERCENT, obj.hFig, 0.35);
            end

            SpecSize.size(obj.hScrollbar, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_LARGE);

            % Then, systematically place
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionRelative(obj.hQueryTitle, obj.hTitle, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hQueryTitle, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hQueryAxes(1), ...
                obj.hQueryTitle, SpecPosition.BELOW);
            SpecPosition.positionIn(obj.hQueryAxes(1), obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            for k = 2:length(obj.hQueryAxes)
                SpecPosition.positionRelative(obj.hQueryAxes(k), ...
                    obj.hQueryAxes(1), SpecPosition.CENTER_Y);
                SpecPosition.positionRelative(obj.hQueryAxes(k), ...
                    obj.hQueryAxes(k-1), SpecPosition.RIGHT_OF, ...
                    0.5*GUISettings.PAD_SMALL);
            end

            SpecPosition.positionRelative(obj.hRefTitle, obj.hQueryAxes(1), ...
                SpecPosition.BELOW, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hRefTitle, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);

            SpecPosition.positionRelative(obj.hRefAxes(1), ...
                obj.hRefTitle, SpecPosition.BELOW);
            SpecPosition.positionIn(obj.hRefAxes(1), obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            for k = 2:length(obj.hRefAxes)
                SpecPosition.positionRelative(obj.hRefAxes(k), ...
                    obj.hRefAxes(1), SpecPosition.CENTER_Y);
                SpecPosition.positionRelative(obj.hRefAxes(k), ...
                    obj.hRefAxes(k-1), SpecPosition.RIGHT_OF, ...
                    0.5*GUISettings.PAD_SMALL);
            end

            SpecPosition.positionIn(obj.hScrollbar, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hScrollbar, obj.hFig, ...
                SpecPosition.CENTER_X);

            % Update the total width of the image axes
            obj.axesWidth = ...
                sum(arrayfun(@(x) x.Position(3), obj.hQueryAxes)) + ...
                2 * GUISettings.PAD_MED;
            if obj.axesWidth < obj.figureWidth
                obj.hScrollbar.Enable = 'off';
            else
                obj.hScrollbar.Enable = 'on';
            end
        end

        function setScollOffset(obj, offset)
            obj.hQueryAxes(1).Position(1) = GUISettings.PAD_MED + offset;
            obj.hRefAxes(1).Position(1) = GUISettings.PAD_MED + offset;
            for k = 2:length(obj.hQueryAxes)
                obj.hQueryAxes(k).Position(1) = ...
                    obj.hQueryAxes(k-1).Position(1) + ...
                    obj.hQueryAxes(k-1).Position(3);
                obj.hRefAxes(k).Position(1) = ...
                    obj.hRefAxes(k-1).Position(1) + ...
                    obj.hRefAxes(k-1).Position(3);
            end
        end
    end
end

