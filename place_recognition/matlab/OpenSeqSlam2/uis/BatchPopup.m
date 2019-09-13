classdef BatchPopup < handle

    properties (Constant)
        FIG_WIDTH_FACTOR = 3;
        FIG_HEIGHT_FACTOR = 6;
    end

    properties
        hFig;

        hCurrent;
        hCurrentDetails;

        hRemaining;
        hRemainingDetails;

        jobCount;
    end

    methods
        function obj = BatchPopup(jobCount)
            % Store any persistent data
            obj.jobCount = jobCount;

            % Create and size the popup
            obj.createPopup();
            obj.sizePopup();

            % Finally, show the figure once done configuring
            obj.hFig.Visible = 'on';
        end

        function updateJob(obj, jobDetails, jobNum)
            obj.hCurrentDetails.String = [jobDetails{1} ' = ' ...
                num2str(jobDetails{2})];
            obj.hRemainingDetails.String = [num2str(obj.jobCount - jobNum+1) ...
                ' jobs remaining (of ' num2str(obj.jobCount) ' total)'];
            drawnow();
        end
    end

    methods (Access = private)
        function createPopup(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'Batch Parameter Sweep Progress';

            % Generic elements
            obj.hCurrent = uicontrol('Style', 'text');
            obj.hCurrent.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hCurrent);
            obj.hCurrent.FontWeight = 'bold';
            obj.hCurrent.String = 'Current job details:';

            obj.hCurrentDetails = uicontrol('Style', 'text');
            obj.hCurrentDetails.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hCurrentDetails);
            obj.hCurrentDetails.String = '';

            obj.hRemaining = uicontrol('Style', 'text');
            obj.hRemaining.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hRemaining);
            obj.hRemaining.FontWeight = 'bold';
            obj.hRemaining.String = 'Jobs remaining:';

            obj.hRemainingDetails = uicontrol('Style', 'text');
            obj.hRemainingDetails.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hRemainingDetails);
            obj.hRemainingDetails.String = '';
        end

        function sizePopup(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            widthUnit = obj.hCurrent.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hCurrent.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * BatchPopup.FIG_WIDTH_FACTOR, ...
                heightUnit * BatchPopup.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'northeast');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hCurrent, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hCurrentDetails, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig, GUISettings.PAD_LARGE);

            SpecSize.size(obj.hRemaining, SpecSize.WIDTH, SpecSize.WRAP);
            SpecSize.size(obj.hRemainingDetails, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig, GUISettings.PAD_LARGE);

            % Then, systematically place
            SpecPosition.positionIn(obj.hCurrent, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hCurrent, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_SMALL);
            SpecPosition.positionIn(obj.hCurrentDetails, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hCurrentDetails, obj.hCurrent, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hRemaining, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hRemaining, ...
                obj.hCurrentDetails, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hRemainingDetails, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hRemainingDetails, ...
                obj.hRemaining, SpecPosition.BELOW, GUISettings.PAD_MED);
        end
    end
end
