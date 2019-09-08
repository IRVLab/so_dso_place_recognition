classdef HelpPopup < handle

    properties (Constant)
        TAG = 'openseqslam2_help';
        POPUP_FIELD = 'Manager';
        BUTTON_FIELD = 'DocTarget';

        FIG_WIDTH_FACTOR = 1.01;
        FIG_HEIGHT_FACTOR = 30;
    end

    properties
        hFig;
        hHelp;
        hDbg;

        hHelpTexts = gobjects(0);
        hWelcomeText;
        hWelcomeClose;
        hScrollbar;

        docSource;

        figureHeight;
        helpHeight;
    end

    methods
        function obj = HelpPopup()
            % Create and size the popup
            obj.createPopup();
            obj.sizePopup();

            % Add the help button
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);
            obj.hHelp.Callback = {@obj.cbHelp};
            SpecPosition.positionIn(obj.hHelp, obj.hFig, ...
                SpecPosition.TOP);
            SpecPosition.positionRelative(obj.hHelp, obj.hScrollbar, ...
                SpecPosition.LEFT_OF);
            obj.hHelp.Visible = 'off';

            % Finally, show the figure once done configuring
            obj.hFig.Visible = 'on';
        end

        function setHelpScreen(obj, docDir)
            % Get the full path for the corresponding help directory
            docSource = fullfile(toolboxRoot(), 'doc', docDir);
            if exist(docSource) ~= 7
                error(['No documentation was found in: ' docSource]);
                return;
            end

            % Get all tex files in the directory
            files = dir(docSource);
            texFiles = files(arrayfun(@(x) endsWith(x.name, '.tex'), files));
            if isempty(texFiles)
                error(['No documentation files were found in: ' docSource]);
                return;
            end
            obj.docSource = docSource;

            % Update all of the helper text objects
            delete(obj.hHelpTexts);
            obj.hHelpTexts = gobjects(size(texFiles));
            for k = 1:length(texFiles)
                % Create the annotation, and add its text
                obj.hHelpTexts(k) = annotation(obj.hFig, 'textbox');
                GUISettings.applyAnnotationStyle(obj.hHelpTexts(k));
                obj.hHelpTexts(k).String = fileread(fullfile(obj.docSource, ...
                    texFiles(k).name));
                drawnow(); % TODO find a non gross way to do this...

                % Size and position the annotation
                if k == 1
                    SpecPosition.positionIn(obj.hHelpTexts(k), obj.hFig, ...
                        SpecPosition.TOP, GUISettings.PAD_MED);
                else
                    SpecPosition.positionRelative(obj.hHelpTexts(k), ...
                        obj.hHelpTexts(k-1), SpecPosition.BELOW);
                end
                SpecPosition.positionIn(obj.hHelpTexts(k), obj.hFig, ...
                    SpecPosition.LEFT);

                % Move to the back
                uistack(obj.hHelpTexts(k), 'bottom');
            end

            % Upate the height of the help
            obj.helpHeight = sum(arrayfun(@(x) x.Position(4), obj.hHelpTexts));
            if obj.helpHeight < obj.figureHeight
                obj.hScrollbar.Enable = 'off';
            else
                obj.hScrollbar.Enable = 'on';
            end
            obj.setHeightOffset(0);
        end
    end

    methods (Static)
        function helpButton = addHelpButton(figHandle)
            % Create and add the help button to the figure
            helpButton = uicontrol('Style', 'pushbutton');
            helpButton.Parent = figHandle;
            GUISettings.applyUIControlStyle(helpButton);
            helpButton.String = '?';

            SpecSize.size(helpButton, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_MED);
            SpecPosition.positionIn(helpButton, figHandle, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(helpButton, figHandle, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);

            % Add in the interactivity
            helpButton.Callback = {@HelpPopup.cbOpenHelp};
        end

        function setDestination(helpButton, docName)
            % Add the dynamic property if it doesn't exist
            if ~isprop(helpButton, HelpPopup.BUTTON_FIELD)
                addprop(helpButton, HelpPopup.BUTTON_FIELD);
            end

            % Set the docname
            helpButton.(HelpPopup.BUTTON_FIELD) = docName;

            % Now try and update the help popup if it exists
            popup = HelpPopup.getPopup();
            if ~isempty(popup)
                popup.setHelpScreen(docName);
            end
        end

        function requestClose()
            popup = HelpPopup.getPopup();
            if ~isempty(popup)
                delete(popup.hFig);
            end
        end
    end

    methods (Access = private, Static)
        function cbOpenHelp(src, event)
            % Get a valid instance of the Help Popup
            popup = HelpPopup.getPopup();
            if isempty(popup)
                popup = HelpPopup();
            end

            % Set the documentation to the target stored in the button
            % TODO handling if for whatever reason the button has no target...
            popup.setHelpScreen(src.(HelpPopup.BUTTON_FIELD));
        end

        function popup = getPopup()
            hs = findobj('Tag', HelpPopup.TAG);
            popup = [];
            if length(hs) > 1
                % TODO handle properly
                error('Found more than 1 HelpPopup.... TODO WHY?');
                return;
            elseif ~isempty(hs)
                popup = hs(1).(HelpPopup.POPUP_FIELD);
            end
        end

        function string = welcomeText()
            string = fileread(fullfile(toolboxRoot(), 'doc', 'welcome.tex'));
        end
    end

    methods (Access = private)
        function cbCloseWelcome(obj, src, event)
            % Hide the welcome area
            obj.hWelcomeText.Visible = 'off';
            obj.hWelcomeClose.Visible = 'off';

            % Display the help button
            obj.hHelp.Visible = 'on';
        end

        function cbDebug(obj, src, event)
            d = obj.docSource;
            nFull = [];
            while ~strcmp(d, fullfile(toolboxRoot(), 'doc'))
                [d, n, e] = fileparts(d);
                nFull = fullfile(n, nFull);
            end
            obj.setHelpScreen(nFull);
        end

        function cbHelp(obj, src, event)
            % Display the welcome area
            obj.hWelcomeText.Visible = 'on';
            obj.hWelcomeClose.Visible = 'on';

            % Hide the help button
            obj.hHelp.Visible = 'off';
        end

        function cbScroll(obj, src, event)
            % Get max scroll offset, and percent of the scroll on the bar
            maxScroll = obj.helpHeight - obj.figureHeight;
            percScroll = (obj.hScrollbar.Max - obj.hScrollbar.Value) / ...
                (obj.hScrollbar.Max - obj.hScrollbar.Min);

            % Set the appropriate offset
            obj.setHeightOffset(maxScroll * percScroll);
        end

        function createPopup(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Tag = HelpPopup.TAG;

            % Make the figure and its manager easily accessible
            obj.hFig.Tag = HelpPopup.TAG;
            addprop(obj.hFig, HelpPopup.POPUP_FIELD);
            obj.hFig.Manager = obj;

            % Create the content
            obj.hDbg = uicontrol('Style', 'pushbutton');
            obj.hDbg.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hDbg);
            obj.hDbg.String = 'R';
            obj.hDbg.Visible = 'off';

            obj.hHelpTexts(1) = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hHelpTexts(1));
            obj.hHelpTexts(1).String = '\noindent\rule{\textwidth}{2pt}';

            obj.hWelcomeText = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hWelcomeText);
            obj.hWelcomeText.String = HelpPopup.welcomeText();
            obj.hWelcomeText.BackgroundColor = GUISettings.COL_WARNING;

            obj.hWelcomeClose = uicontrol('Style', 'text');
            obj.hWelcomeClose.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hWelcomeClose);
            obj.hWelcomeClose.String = 'x';
            obj.hWelcomeClose.FontSize = 1.2 * obj.hWelcomeClose.FontSize;
            obj.hWelcomeClose.FontWeight = 'bold';
            obj.hWelcomeClose.Enable = 'inactive';
            obj.hWelcomeClose.BackgroundColor = GUISettings.COL_WARNING;

            obj.hScrollbar = uicontrol('Style', 'slider');
            obj.hScrollbar.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hScrollbar);
            obj.hScrollbar.Value = 1;

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hDbg.Callback = {@obj.cbDebug};
            addlistener(obj.hScrollbar, 'Value', 'PostSet', @obj.cbScroll);
            obj.hWelcomeClose.ButtonDownFcn = {@obj.cbCloseWelcome};
        end

        function sizePopup(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            drawnow(); % TODO find a non gross way to do this...
            widthUnit = obj.hHelpTexts(1).Position(3);
            heightUnit = obj.hHelpTexts(1).Position(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * HelpPopup.FIG_WIDTH_FACTOR, ...
                heightUnit * HelpPopup.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');
            obj.figureHeight = obj.hFig.Position(4);

            % Size and place the elements (grossly...)
            SpecSize.size(obj.hDbg, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hDbg, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hDbg, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hHelpTexts(1), obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hHelpTexts(1), obj.hFig, ...
                SpecPosition.LEFT);
            SpecPosition.positionIn(obj.hWelcomeText, obj.hFig, ...
                SpecPosition.BOTTOM);
            SpecPosition.positionIn(obj.hWelcomeText, obj.hFig, ...
                SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hWelcomeClose, ...
                obj.hWelcomeText, SpecPosition.TOP, GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hWelcomeClose, ...
                obj.hWelcomeText, SpecPosition.RIGHT);
            SpecSize.size(obj.hScrollbar, SpecSize.HEIGHT, SpecSize.MATCH, ...
                obj.hFig);
            SpecSize.size(obj.hScrollbar, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.02);
            SpecPosition.positionIn(obj.hScrollbar, obj.hFig, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hScrollbar, obj.hFig, ...
                SpecPosition.RIGHT);
            %SpecPosition.RIGHT, -1 * GUISettings.PAD_MED);
        end

        function setHeightOffset(obj, offset)
            for k = 1:length(obj.hHelpTexts)
                if k == 1
                    obj.hHelpTexts(k).Position(2) = obj.figureHeight - ...
                        GUISettings.PAD_MED - obj.hHelpTexts(k).Position(4) ...
                        + offset;
                else
                    obj.hHelpTexts(k).Position(2) = ...
                        obj.hHelpTexts(k-1).Position(2) - ...
                        obj.hHelpTexts(k).Position(4);
                end
            end
        end
    end
end
