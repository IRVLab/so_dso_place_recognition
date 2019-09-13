classdef GroundTruthPopup < handle

    properties (Constant)
        FIG_WIDTH_FACTOR = 4;
        FIG_HEIGHT_FACTOR = 31;

        SOURCE_VEL = 1;
        SOURCE_CSV = 2;
        SOURCE_MAT = 3;
    end

    properties
        hFig;
        hHelp;

        hTitle;

        hType;
        hTypeValue;

        hVelocityVel;
        hVelocityVelValue;
        hVelocityTol;
        hVelocityTolValue;

        hFilePathSelect;
        hFilePathDetails;

        hAxis;
        hError;

        hApply;

        targetSize;
        gt = emptyGroundTruth();

        selectedCSV = [];
        selectedMAT = [];
        selectedVar = [];
        selectedMatrix = [];
    end

    methods
        function obj = GroundTruthPopup(gt, targetSize)
            % Save the provided data
            obj.gt = gt;
            obj.targetSize = targetSize;

            % Create and size the popup
            obj.createPopup();
            obj.sizePopup();

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);
            HelpPopup.setDestination(obj.hHelp, ...
                'ground_truth');

            % Populate the UI (drawing happens after poulating selects a method)
            obj.populate();

            % Finally, show the figure once done configuring
            obj.hFig.Visible = 'on';
        end
    end

    methods (Static)
        function [gt, err] = getGtMatrix(gtdata, diffSize)
            gt = [];
            if ~gtdata.exists
                err = 'Ground truth existance configured as false';
            elseif strcmp(gtdata.type, 'mat')
                [gt, err] = GroundTruthPopup.gtFromMAT(gtdata.file.path, ...
                    gtdata.file.var, diffSize);
            elseif strcmp(gtdata.type, 'csv')
                [gt, err] = GroundTruthPopup.gtFromCSV(gtdata.file.path, ...
                    diffSize);
            else
                [gt, err] = GroundTruthPopup.gtFromVel(gtdata.velocity.vel, ...
                    gtdata.velocity.tol, diffSize);
            end
        end

        function [gt, err] = gtFromVel(vel, tol, diffSize)
            gt = []; err = [];

            % Note: there are no errors to detect here
            % Construct and return the ground truth matrix
            qs = 1:diffSize(2);
            rs = round(linspace(1, 1 + (length(qs)-1)*vel, length(qs)));
            gt = zeros(length(rs), length(qs));
            for k = 1:length(qs)
                gt = GroundTruthPopup.addGroundTruthValue(gt, qs(k), rs(k), ...
                    tol);
            end
        end

        function [gt, err] = gtFromCSV(filename, diffSize)
            gt = []; err = [];

            % Read in the csv
            v = csvread(filename);

            % Perform validity checks
            if size(v,2) > 3
                err = ['Data has too many values (' ...
                    num2str(size(v,2)) ') per row (3 expected)'];
            elseif min(min(v)) < 0
                err = 'Data is invalid (negative values were detected)';
            elseif max(v(:,1)) > diffSize(2)
                err = ['Data found with query image number (' ...
                    num2str(max(v(:,1))) ') greater than number of ' ...
                    'query images in the difference matrix (' ...
                    num2str(diffSize(2)) ')'];
            elseif max(v(:,2)) > diffSize(1)
                err = ['Data found with reference image number (' ...
                    num2str(max(v(:,2))) ') greater than number of ' ...
                    'reference images in the difference matrix (' ...
                    num2str(diffSize(1)) ')'];
            end

            % Construct and return the ground truth matrix
            gt = zeros(diffSize);
            for k = 1:size(v, 1)
                gt = GroundTruthPopup.addGroundTruthValue(gt, v(k,1), ...
                    v(k,2), v(k,3));
            end
        end

        function [gt, err] = gtFromMAT(filename, varname, diffSize)
            gt = []; err = [];

            % Get the list of variables, checking the variable is availabe
            vars = whos('-file', filename);
            v = vars(find(arrayfun(@(x) strcmp(x.name, varname), vars)));
            if isempty(v)
                err = ['Could not find variable ' varname 'in MAT-file at ' ...
                    filename];
                return;
            end

            % Open the variable
            data = load(filename, v.name);
            v = getfield(data, v.name);

            % Perform validity checks
            if ~isequal(size(v), diffSize)
                err = ['The size of the ground truth data (' ...
                    num2str(size(v, 1)) 'x' num2str(size(v,2)) ...
                    ') does not match the size of the difference matrix (' ...
                    num2str(diffSize(1)) 'x' num2str(diffSize(2)) ')'];
                return;
            elseif ~isempty(v(v(v(v ~= 0) ~= 1) ~=0.5))
                err = ['Invalid values were detected in the ground truth ' ...
                    'data (only 0, 0.5, and 1 are supported values)'];
                return;
            end

            % Construct and return the ground truth matrix
            gt = v;
        end

        function [str, col] = gtDescription(gtdata)
            if isempty(gtdata) || isempty(gtdata.exists) || ...
                    ~gtdata.exists %|| isempty(gtdata.matrix)
                str = ['No ground truth matrix available. Please ' ...
                    'configure to select one...'];
                col = GUISettings.COL_ERROR;
            elseif any(strcmp(gtdata.type, {'csv', 'mat'}))
                if isempty(gtdata.file.path)
                    str = 'No data for ground truth file found';
                    col = GUISettings.COL_WARNING;
                elseif ~exist(gtdata.file.path)
                    str = ['File @ ' gtdata.file.path 'could not be found'];
                    col = GUISettings.COL_WARNING;
                else
                    str = ['Loaded from ' gtdata.file.path];
                    if strcmp(gtdata.type, 'mat')
                        str = [str '; and using variable ' gtdata.file.var];
                    end
                    col = GUISettings.COL_SUCCESS;
                end
            elseif strcmp(gtdata.type, 'velocity')
                if isempty(gtdata.velocity.vel) || ...
                        isempty(gtdata.velocity.tol)
                    str = ['Failed to load data for velocity based ground truth'];
                    col = GUISettings.COL_WARNING;
                else
                    str = ['Ground truth with vel = ' ...
                        num2str(gtdata.velocity.vel) ' & tol = ' ...
                        num2str(gtdata.velocity.tol)];
                    col = GUISettings.COL_SUCCESS;
                end
            end
        end
    end

    methods (Access = private, Static)
        function matrix = addGroundTruthValue(matrix, q, r, t)
            rRange = r-t:r+t;
            rRange = rRange(rRange >= 1 & rRange <= size(matrix, 1));
            matrix(rRange, q) = 0.5;
            matrix(r, q) = 1.0;
        end
    end

    methods (Access = private)
        function cbApply(obj, src, event)
            % Handle error cases
            if isempty(obj.selectedMatrix)
                uiwait(errordlg( ...
                    'No valid ground truth data has been selected', ...
                    'Cannot apply empty ground truth data', 'modal'));
                return;
            end

            % Strip out the chosen data, and construct the ground truth matrix
            obj.strip();
            obj.gt.matrix = obj.selectedMatrix > 0;

            % Close the figure
            delete(obj.hFig)
        end

        function cbClose(obj, src, event)
            obj.selectedMatrix = [];
            obj.gt = [];
            delete(obj.hFig);
        end

        function cbSelectSource(obj, src, event)
            if obj.hTypeValue.Value > 1
                obj.hVelocityVel.Visible = 'off';
                obj.hVelocityVelValue.Visible = 'off';
                obj.hVelocityTol.Visible = 'off';
                obj.hVelocityTolValue.Visible = 'off';

                obj.hFilePathSelect.Visible = 'on';
                obj.hFilePathDetails.Visible = 'on';
            else
                obj.hVelocityVel.Visible = 'on';
                obj.hVelocityVelValue.Visible = 'on';
                obj.hVelocityTol.Visible = 'on';
                obj.hVelocityTolValue.Visible = 'on';

                obj.hFilePathSelect.Visible = 'off';
                obj.hFilePathDetails.Visible = 'off';
            end
            obj.cbUpdateGroundTruth(obj.hTypeValue, []);
        end

        function cbSelectSourceFile(obj, src, event)
            % Attempt to open the source file, then update all ground truth data
            obj.openSourceFile();
            obj.drawScreen();
        end

        function cbUpdateGroundTruth(obj, src, event)
            % Update ground truth and redraw
            obj.updateGroundTruth();
            obj.drawScreen();
        end

        function createPopup(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'Ground Truth Configuration';

            % Create the title
            obj.hTitle = uicontrol('Style', 'text');
            obj.hTitle.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hTitle);
            GUISettings.setFontScale(obj.hTitle, 1.5);
            obj.hTitle.String = 'Ground Truth Configuration';

            % Create the configuration UI elements
            obj.hType = uicontrol('Style', 'text');
            obj.hType.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hType);
            obj.hType.String = 'Ground Truth Source:';

            obj.hTypeValue = uicontrol('Style', 'popupmenu');
            obj.hTypeValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hTypeValue);
            obj.hTypeValue.String = { 'velocity model', '*.csv file', ...
                '*.mat matrix'};

            obj.hVelocityVel = uicontrol('Style', 'text');
            obj.hVelocityVel.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hVelocityVel);
            obj.hVelocityVel.String = 'Ground truth velocity:';

            obj.hVelocityVelValue = uicontrol('Style', 'edit');
            obj.hVelocityVelValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hVelocityVelValue);
            obj.hVelocityVelValue.String = '';

            obj.hVelocityTol = uicontrol('Style', 'text');
            obj.hVelocityTol.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hVelocityTol);
            obj.hVelocityTol.String = 'Ground truth tolerance:';

            obj.hVelocityTolValue = uicontrol('Style', 'edit');
            obj.hVelocityTolValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hVelocityTolValue);
            obj.hVelocityTolValue.String = '';

            obj.hFilePathSelect = uicontrol('Style', 'pushbutton');
            obj.hFilePathSelect.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hFilePathSelect);
            obj.hFilePathSelect.String = 'Select file';

            obj.hFilePathDetails = uicontrol('Style', 'text');
            obj.hFilePathDetails.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hFilePathDetails);
            obj.hFilePathDetails.String = {' ', ' '};
            obj.hFilePathDetails.FontAngle = 'italic';
            obj.hFilePathDetails.HorizontalAlignment = 'left';

            % Create the axis
            obj.hAxis = axes();
            GUISettings.applyUIAxesStyle(obj.hAxis);
            obj.hAxis.Visible = 'off';
            colormap(obj.hAxis, 'gray');

            obj.hError = uicontrol('Style', 'text');
            obj.hError.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hError);
            GUISettings.setFontScale(obj.hError, 1.25);
            obj.hError.String = 'Please select a valid ground truth file above';
            obj.hError.FontAngle = 'italic';

            % Create the apply button
            obj.hApply = uicontrol('Style', 'pushbutton');
            obj.hApply.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hApply);
            obj.hApply.String = 'Apply ground truth settings';

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hTypeValue.Callback = {@obj.cbSelectSource};
            obj.hVelocityVelValue.Callback = {@obj.cbUpdateGroundTruth};
            obj.hVelocityTolValue.Callback = {@obj.cbUpdateGroundTruth};
            obj.hFilePathSelect.Callback = {@obj.cbSelectSourceFile};
            obj.hApply.Callback = {@obj.cbApply};
            obj.hFig.CloseRequestFcn = {@obj.cbClose};
        end

        function drawScreen(obj)
            cla(obj.hAxis);

            % Either draw the matrix or show the error
            if (obj.hTypeValue.Value == GroundTruthPopup.SOURCE_CSV && ...
                    isempty(obj.selectedCSV)) || ...
                    (obj.hTypeValue.Value == GroundTruthPopup.SOURCE_MAT && ...
                    isempty(obj.selectedMAT))
                obj.hAxis.Visible = 'off';
                obj.hError.Visible = 'on';
            else
                obj.hAxis.Visible = 'on';
                obj.hError.Visible = 'off';

                % Draw the currently selected ground truth matrix
                imagesc(obj.selectedMatrix, 'Parent', obj.hAxis);
                GUISettings.axesDiffMatrixStyle(obj.hAxis, obj.targetSize);
            end

            % Update the details text if necessary
            if obj.hTypeValue.Value == GroundTruthPopup.SOURCE_CSV && ...
                    ~isempty(obj.selectedCSV)
                obj.hFilePathDetails.String = {['Ground truth matrix loaded ' ...
                    'from:'], obj.selectedCSV};
            elseif obj.hTypeValue.Value == GroundTruthPopup.SOURCE_MAT && ...
                    ~isempty(obj.selectedMAT)
                obj.hFilePathDetails.String = {['Ground truth matrix loaded ' ...
                    'from ''' obj.selectedVar ''' in:'], obj.selectedMAT};
            else
                obj.hFilePathDetails.String = {' ' ' '};
            end
        end

        function openSourceFile(obj)
            % Attempt to get a file of the appropriate type from the user
            f = 0; p = 0;
            if obj.hTypeValue.Value == GroundTruthPopup.SOURCE_MAT
                [f, p] = uigetfile('*.mat', ...
                    'Select a *.mat file containing a ground truth matrix');
            elseif obj.hTypeValue.Value == GroundTruthPopup.SOURCE_CSV
                [f, p] = uigetfile('*.csv', ...
                    'Select a *.csv file containing ground truth data');
            end

            % Bail if there was no valid file selected
            if isnumeric(f) || isnumeric (p)
                return;
            end
            f = fullfile(p, f);

            % Prompt variable selection if choosing a MAT file
            if obj.hTypeValue.Value == GroundTruthPopup.SOURCE_MAT
                % Get the user to select variable to use
                vars = whos('-file', f);
                v = listdlg('PromptString', ...
                    'Which variable represents the ground truth matrix:', ...
                    'SelectionMode', 'Single', 'ListString', {vars.name});
                if isempty(v)
                    return;
                end
                v = vars(v).name;

                % Attempt to open the ground truth matrix
                [gt, err] = GroundTruthPopup.gtFromMAT(f, v, obj.targetSize);

                % Exit if there was an error
                if ~isempty(err)
                    uiwait(errordlg(err, 'Ground Truth Data Read Failed', ...
                        'modal'));
                    return;
                end

                % Valid, save the selection
                obj.selectedMAT = f;
                obj.selectedVar = v;
                obj.selectedMatrix = gt;
            elseif obj.hTypeValue.Value == GroundTruthPopup.SOURCE_CSV
                % Attempt to opent he ground truth matrix
                [gt, err] = GroundTruthPopup.gtFromCSV(f, obj.targetSize);

                % Exit if there was an error
                if ~isempty(err)
                    uiwait(errordlg(err, 'Ground Truth Data Read Failed', ...
                        'modal'));
                    return;
                end

                % Valid, save the selection
                obj.selectedCSV = f;
                obj.selectedMatrix = gt;
            end
        end

        function populate(obj)
            % Load the values in from the results
            source = SafeData.noEmpty(obj.gt.type, 'velocity');
            if strcmpi(source, 'file')
                file = SafeData.noEmpty(obj.gt.file.path, '*.csv');
                if endswith(file, 'mat')
                    obj.hTypeValue.Value = GroundTruthPopup.SOURCE_MAT;
                else
                    obj.hTypeValue.Value = GroundTruthPopup.SOURCE_CSV;
                end
            else
                obj.hTypeValue.Value = GroundTruthPopup.SOURCE_VEL;
            end
            obj.hVelocityVelValue.String = SafeData.noEmpty( ...
                obj.gt.velocity.vel, 1);
            obj.hVelocityTolValue.String = SafeData.noEmpty( ...
                obj.gt.velocity.tol, 5);

            % Execute any required callbacks manually
            obj.cbSelectSource(obj.hTypeValue, []);
        end

        function sizePopup(obj)
            % Statically size for now
            % TODO handle potential resizing gracefully
            widthUnit = obj.hTitle.Extent(3) * toolboxWidthFactor();
            heightUnit = obj.hTitle.Extent(4);

            % Size and position the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * GroundTruthPopup.FIG_WIDTH_FACTOR, ...
                heightUnit * GroundTruthPopup.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hTitle, SpecSize.HEIGHT, SpecSize.WRAP);
            SpecSize.size(obj.hTitle, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);

            SpecSize.size(obj.hType, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_MED);
            SpecSize.size(obj.hTypeValue, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.125);

            SpecSize.size(obj.hVelocityVel, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_MED);
            SpecSize.size(obj.hVelocityVelValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);
            SpecSize.size(obj.hVelocityTol, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_MED);
            SpecSize.size(obj.hVelocityTolValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);

            SpecSize.size(obj.hFilePathSelect, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.1);
            SpecSize.size(obj.hFilePathDetails, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.6);
            SpecSize.size(obj.hFilePathDetails, SpecSize.HEIGHT, ...
                SpecSize.WRAP);

            SpecSize.size(obj.hAxis, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.8);
            SpecSize.size(obj.hAxis, SpecSize.HEIGHT, SpecSize.RATIO, 3/4);
            SpecSize.size(obj.hError, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.5);
            SpecSize.size(obj.hError, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.5);

            SpecSize.size(obj.hApply, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.2);

            % Then, systematically place
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hTitle, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionRelative(obj.hType, obj.hTitle, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hType, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hTypeValue, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hTypeValue, obj.hType, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hVelocityVel, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hVelocityVel, obj.hTypeValue, ...
                SpecPosition.RIGHT_OF, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hVelocityVelValue, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hVelocityVelValue, ...
                obj.hVelocityVel, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hVelocityTol, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hVelocityTol, ...
                obj.hVelocityVelValue, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hVelocityTolValue, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hVelocityTolValue, ...
                obj.hVelocityTol, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hFilePathSelect, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hFilePathSelect, ...
                obj.hTypeValue, SpecPosition.RIGHT_OF, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hFilePathDetails, obj.hType, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hFilePathDetails, ...
                obj.hFilePathSelect, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hAxis, obj.hType, ...
                SpecPosition.BELOW, 4*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hAxis, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hError, obj.hFig, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hError, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hApply, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hApply, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
        end

        function strip(obj)
            % ONLY strip out the values for the currently selected method
            obj.gt.exists = true;
            if obj.hTypeValue.Value == 1
                obj.gt.type = 'velocity';
                obj.gt.file.path = [];
                obj.gt.velocity.vel = str2num(obj.hVelocityVelValue.String);
                obj.gt.velocity.tol = str2num(obj.hVelocityTolValue.String);
            elseif obj.hTypeValue.Value == 2
                obj.gt.type = 'csv';
                obj.gt.file.path = obj.selectedCSV;
                obj.gt.velocity.vel = [];
                obj.gt.velocity.tol = [];
            elseif obj.hTypeValue.Value == 3
                obj.gt.type = 'mat';
                obj.gt.file.path = obj.selectedMAT;
                obj.gt.file.var = obj.selectedVar;
                obj.gt.velocity.vel = [];
                obj.gt.velocity.tol = [];
            else
                obj.gt.exists = false;
            end
        end

        function updateGroundTruth(obj)
            % Reconstruct the ground truth matrix based on selected parameters
            if obj.hTypeValue.Value == GroundTruthPopup.SOURCE_VEL
                obj.selectedMatrix = GroundTruthPopup.gtFromVel( ...
                    str2num(obj.hVelocityVelValue.String), ...
                    str2num(obj.hVelocityTolValue.String), obj.targetSize);
            elseif obj.hTypeValue.Value == GroundTruthPopup.SOURCE_CSV && ...
                    ~isempty(obj.selectedCSV)
                obj.selectedMatrix = GroundTruthPopup.gtFromCSV( ...
                    obj.selectedCSV, obj.targetSize);
            elseif obj.hTypeValue.Value == GroundTruthPopup.SOURCE_MAT && ...
                    ~isempty(obj.selectedMAT)
                obj.selectedMatrix = GroundTruthPopup.gtFromMAT( ...
                    obj.selectedMAT, obj.selectedVar, obj.targetSize);
            else
                obj.selectedMatrix = [];
            end
        end
    end
end
