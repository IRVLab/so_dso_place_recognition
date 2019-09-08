classdef ConfigureGUI < handle
    % TODO on the image preprocessing settings screen selecting the same value
    % should not cause the disabling, but currently does

    properties (Access = private, Constant)
        SCREENS = { ...
            'Image preprocessing', ...
            'Sequence Matching', ...
            'Miscellaneous'};

        % Sizing parameters
        FIG_WIDTH_FACTOR = 10;  % Times longest internal heading
        FIG_HEIGHT_FACTOR = 40; % Times height of buttons at font size

        % Visual constants
        IMAGE_FADE = 0.2;
    end

    properties
        hFig;
        hHelp;
        hScreen;

        hImPrRef;
        hImPrRefSample;
        hImPrRefAxCrop;
        hImPrRefCropBox;
        hImPrRefAxResize;
        hImPrRefAxNorm;
        hImPrQuery;
        hImPrQuerySample;
        hImPrQueryAxCrop;
        hImPrQueryCropBox;
        hImPrQueryAxResize;
        hImPrQueryAxNorm;
        hImPrRefresh;
        hImPrCropRef;
        hImPrCropRefValue;
        hImPrCropQuery;
        hImPrCropQueryValue;
        hImPrResize;
        hImPrResizeW;
        hImPrResizeX;
        hImPrResizeH;
        hImPrResizeLock;
        hImPrResizeMethod;
        hImPrResizeMethodValue;
        hImPrNorm;
        hImPrNormThresh;
        hImPrNormThreshValue;
        hImPrNormStrength;
        hImPrNormStrengthValue;

        hMatchSearchTitle;
        hMatchSearchAx;
        hMatchSearchLength;
        hMatchSearchLengthValue;
        hMatchSearchVmin;
        hMatchSearchVminValue;
        hMatchSearchVmax;
        hMatchSearchVmaxValue;
        hMatchSearchMethod;
        hMatchSearchMethodValue;
        hMatchSearchVstep;
        hMatchSearchVstepValue;
        hMatchCriTitle;
        hMatchCriAx;
        hMatchCriMethod;
        hMatchCriMethodValue;
        hMatchCriWindow;
        hMatchCriWindowValue;
        hMatchCriU;
        hMatchCriUValue;
        hMatchCriThreshold;
        hMatchCriThresholdValue;

        hMscDiff;
        hMscDiffEnh;
        hMscDiffEnhValue;
        hMscGt;
        hMscGtStatus;
        hMscGtConfigure;
        hMscGtRemove;
        hMscGtOptimise;
        hMscGtOptimiseValue;
        hMscBatch;
        hMscBatchEnabled;
        hMscBatchParam;
        hMscBatchParamValue;
        hMscBatchValues;
        hMscBatchValuesValue;
        hMscBatchParallelise;
        hMscBatchTrim;
        hMscUI;
        hMscUIProgressType;
        hMscUIProgressTypeValue;
        hMscUIResults;
        hMscUIWarn;
        hMscUIPerc;
        hMscUIPercValue;
        hMscUIPrepro;
        hMscUIPreproValue;
        hMscUIDiff;
        hMscUIDiffValue;
        hMscUIContr;
        hMscUIContrValue;
        hMscUIMatch;
        hMscUIMatchValue;

        hDone;

        config = emptyConfig();

        numbersRef = [];
        numbersQuery = [];

        listImagesRef = [];
        listImagesQuery = [];

        dimRef = [];
        dimQuery = [];

        dataTraj = [];
        dataWindow = [];
        dataThresh = [];

        cachedGroundTruth = [];
    end

    methods
        function obj = ConfigureGUI(config)
            % Build all required data
            obj.config = config;
            obj.generateImageLists();

            % Create and size the GUI
            obj.createGUI();
            obj.sizeGUI();

            % Add the help button to the figure
            obj.hHelp = HelpPopup.addHelpButton(obj.hFig);

            % Populate the UI, and open the default screen (first)
            obj.populate();
            obj.openScreen(obj.hScreen.Value);

            % Finally, show the figure when we are done configuring
            obj.hFig.Visible = 'on';
        end
    end

    methods (Access = private, Static)
        function out = constrainInLimits(in, xlims, ylims)
            % Force the start positions to remain inside left and top limits
            out(1:2) = max(in(1:2), [xlims(1) ylims(1)]);

            % Adapt width and height so other box edges remain at same place
            %out(3:4) = in(3:4) - (out(1:2) - in(1:2));
            out(3:4) = in(3:4);

            % Force width and height to keep box inside right and bottom limits
            out(3:4) = min(out(3:4), [xlims(2) ylims(2)]-out(1:2));

            % Handle the rounding due to limits being at X.5
            if out(1) + out(3) == xlims(2)
                out(3) = out(3) - 0.01;
            end
            if out(2) + out(4) == ylims(2)
                out(4) = out(4) - 0.01;
            end
        end
    end

    methods (Access = private)
        function cbBatchToggle(obj, src, event)
            if obj.hMscBatchEnabled.Value
                status = 'on';
            else
                status = 'off';
            end
            obj.hMscBatchParam.Enable = status;
            obj.hMscBatchParamValue.Enable = status;
            obj.hMscBatchValues.Enable = status;
            obj.hMscBatchValuesValue.Enable = status;
            obj.hMscBatchParallelise.Enable = status;
            obj.hMscBatchTrim.Enable = status;
        end

        function cbChangeCrop(obj, src, event)
            if src == obj.hImPrRefCropBox
                mask = [0 0; 1 0; 1 0];
            elseif src == obj.hImPrQueryCropBox
                mask = [0 0; 0 1; 0 1];
            end
            obj.disableProcessingPreviews(mask);
        end

        function cbChangePreviewImage(obj, src, event)
            if (src == obj.hImPrRefSample)
                mask = [1 0; 1 0; 1 0];
            elseif (src == obj.hImPrQuerySample)
                mask = [0 1; 0 1; 0 1];
            end
            obj.disableProcessingPreviews(mask);
        end

        function cbChangeResize(obj, src, event)
            % Disable the appropriate previews
            obj.disableProcessingPreviews([0 0; 1 1; 1 1]);

            % Force the other resize value to maintain aspect ratio if required
            if obj.hImPrResizeLock.Value == 1
                pos = obj.hImPrRefCropBox.getPosition();
                ar = pos(3) / pos(4);
                if src == obj.hImPrResizeW
                    obj.hImPrResizeH.String = num2str(round( ...
                        str2num(obj.hImPrResizeW.String) / ar));
                elseif src == obj.hImPrResizeH
                    obj.hImPrResizeW.String = num2str(round( ...
                        str2num(obj.hImPrResizeH.String) * ar));
                end
            end
        end

        function cbChangeNorm(obj, src, event)
            obj.disableProcessingPreviews([0 0; 0 0; 1 1]);
        end

        function cbChangeScreen(obj, src, event)
            obj.openScreen(obj.hScreen.Value);
        end

        function cbChooseSearchMethod(obj, src, event)
            if obj.hMatchSearchMethodValue.Value == 1
                status = 'on';
            else
                status = 'off';
            end
            obj.hMatchSearchVstep.Visible = status;
            obj.hMatchSearchVstepValue.Visible = status;
            obj.drawMatchDiagrams();
        end

        function cbConfigureGroundTruth(obj, src, event)
            % Launch the ground truth popup
            if isempty(obj.cachedGroundTruth)
                gt = emptyGroundTruth;
            else
                gt = obj.cachedGroundTruth;
            end
            gtui = GroundTruthPopup(gt, ...
                [length(SeqSLAMInstance.numbers(obj.config.query)), ...
                length(SeqSLAMInstance.numbers(obj.config.reference))]);
            uiwait(gtui.hFig);

            % Update the ground truth options if one was selected
            if ~isempty(gtui.gt)
                obj.cachedGroundTruth = gtui.gt;
                obj.updateGroundTruthOptions();
            end
            obj.openScreen(obj.hScreen.Value);
        end

        function cbDone(obj, src, event)
            % Valid the data before proceeding
            if ~obj.isDataValid([]);
                return;
            end

            % Strip the UI data, save it in the config, and close the GUI
            obj.strip();
            close(obj.hFig);
        end

        function cbLockAspectRatio(obj, src, event)
            if obj.hImPrResizeLock.Value == 1
                % Set height to track aspect ratio
                pos = obj.hImPrRefCropBox.getPosition();
                ar = pos(3)/pos(4);
                obj.hImPrResizeH.String = num2str(round( ...
                    str2num(obj.hImPrResizeW.String) / ar));
            end
        end

        function cbRefreshDiagrams(obj, src, event)
            obj.drawMatchDiagrams();
        end

        function cbRefreshPreprocessed(obj, src, event)
            if ~obj.isDataValid(obj.hScreen.Value);
                return;
            end

            obj.drawProcessingPreviews();
        end

        function cbRemoveGroundTruth(obj, src, event)
            obj.cachedGroundTruth = [];
            obj.updateGroundTruthOptions();
        end

        function cbSelectMatchMethod(obj, src, event)
            % Enable the correct visual elements (1 is the default)
            if obj.hMatchCriMethodValue.Value == 2
                obj.hMatchCriWindow.Visible = 'off';
                obj.hMatchCriWindowValue.Visible = 'off';
                obj.hMatchCriU.Visible = 'off';
                obj.hMatchCriUValue.Visible = 'off';
                obj.hMatchCriThreshold.Visible = 'on';
                obj.hMatchCriThresholdValue.Visible = 'on';
            else
                obj.hMatchCriWindow.Visible = 'on';
                obj.hMatchCriWindowValue.Visible = 'on';
                obj.hMatchCriU.Visible = 'on';
                obj.hMatchCriUValue.Visible = 'on';
                obj.hMatchCriThreshold.Visible = 'off';
                obj.hMatchCriThresholdValue.Visible = 'off';
            end

            % Refresh the diagrams
            obj.cbRefreshDiagrams(src, event);
        end

        function clearScreen(obj)
            % Hide all options
            obj.hImPrRef.Visible = 'off';
            obj.hImPrRefSample.Visible = 'off';
            obj.hImPrRefAxCrop.Visible = 'off';
            obj.hImPrRefAxResize.Visible = 'off';
            obj.hImPrRefAxNorm.Visible = 'off';
            obj.hImPrQuery.Visible = 'off';
            obj.hImPrQuerySample.Visible = 'off';
            obj.hImPrQueryAxCrop.Visible = 'off';
            obj.hImPrQueryAxResize.Visible = 'off';
            obj.hImPrQueryAxNorm.Visible = 'off';
            obj.hImPrRefresh.Visible = 'off';
            obj.hImPrCropRef.Visible = 'off';
            obj.hImPrCropRefValue.Visible = 'off';
            obj.hImPrCropQuery.Visible = 'off';
            obj.hImPrCropQueryValue.Visible = 'off';
            obj.hImPrResize.Visible = 'off';
            obj.hImPrResizeW.Visible = 'off';
            obj.hImPrResizeX.Visible = 'off';
            obj.hImPrResizeH.Visible = 'off';
            obj.hImPrResizeLock.Visible = 'off';
            obj.hImPrResizeMethod.Visible = 'off';
            obj.hImPrResizeMethodValue.Visible = 'off';
            obj.hImPrNorm.Visible = 'off';
            obj.hImPrNormThresh.Visible = 'off';
            obj.hImPrNormThreshValue.Visible = 'off';
            obj.hImPrNormStrength.Visible = 'off';
            obj.hImPrNormStrengthValue.Visible = 'off';

            obj.hMatchSearchTitle.Visible = 'off';
            obj.hMatchSearchAx.Visible = 'off';
            obj.hMatchSearchLength.Visible = 'off';
            obj.hMatchSearchLengthValue.Visible = 'off';
            obj.hMatchSearchVmin.Visible = 'off';
            obj.hMatchSearchVminValue.Visible = 'off';
            obj.hMatchSearchVmax.Visible = 'off';
            obj.hMatchSearchVmaxValue.Visible = 'off';
            obj.hMatchSearchMethod.Visible = 'off';
            obj.hMatchSearchMethodValue.Visible = 'off';
            obj.hMatchSearchVstep.Visible = 'off';
            obj.hMatchSearchVstepValue.Visible = 'off';
            obj.hMatchCriTitle.Visible = 'off';
            obj.hMatchCriAx.Visible = 'off';
            obj.hMatchCriMethod.Visible = 'off';
            obj.hMatchCriMethodValue.Visible = 'off';
            obj.hMatchCriWindow.Visible = 'off';
            obj.hMatchCriWindowValue.Visible = 'off';
            obj.hMatchCriU.Visible = 'off';
            obj.hMatchCriUValue.Visible = 'off';
            obj.hMatchCriThreshold.Visible = 'off';
            obj.hMatchCriThresholdValue.Visible = 'off';

            obj.hMscDiff.Visible = 'off';
            obj.hMscGt.Visible = 'off';
            obj.hMscBatch.Visible = 'off';
            obj.hMscUI.Visible = 'off';

            % Clear the axes
            cla(obj.hImPrRefAxCrop);
            cla(obj.hImPrRefAxResize);
            cla(obj.hImPrRefAxNorm);
            cla(obj.hImPrQueryAxCrop);
            cla(obj.hImPrQueryAxResize);
            cla(obj.hImPrQueryAxNorm);
            cla(obj.hMatchSearchAx);
            cla(obj.hMatchCriAx);

            % Delete any remaining objects
            delete(obj.hImPrRefCropBox);
            delete(obj.hImPrQueryCropBox);
        end

        function posOut = constrainedQueryPosition(obj, posIn)
            % Force to be within axis limits
            posOut = ConfigureGUI.constrainInLimits(posIn, ...
                obj.hImPrQueryAxCrop.XLim, obj.hImPrQueryAxCrop.YLim);

            % Enforce that the aspect ratio must remain the same
            if abs(posOut(3)/posOut(4) - posIn(3)/posIn(4)) > 0.001
                newW = posOut(4) * posIn(3)/posIn(4);
                newH = posOut(3) * posIn(4)/posIn(3);
                if newW < posOut(3)
                    posOut(3) = newW;
                elseif newH < posOut(4)
                    posOut(4) = newH;
                end
            end

            % Update the text boxes
            v = round([posOut(1:2) posOut(1:2)+posOut(3:4)]);
            obj.hImPrCropQueryValue.String = SafeData.vector2str(v);
            obj.hImPrResizeW.String = v(3) - v(1) + 1;
            obj.hImPrResizeH.String = v(4) - v(2) + 1;

            % Fade all other previews
            obj.disableProcessingPreviews([0 0; 1 1; 1 1]);
        end

        function posOut = constrainedRefPosition(obj, posIn)
            % Force to be within axis limits
            posOut = ConfigureGUI.constrainInLimits(posIn, ...
                obj.hImPrRefAxCrop.XLim, obj.hImPrRefAxCrop.YLim);

            % Force the other crop box to have the same position
            % TODO could be more flexible here... but this is easy
            obj.hImPrQueryCropBox.setPosition(posOut);

            % Update the text boxes
            v = round([posOut(1:2) posOut(1:2)+posOut(3:4)]);
            obj.hImPrCropRefValue.String = SafeData.vector2str(v);
            obj.hImPrCropQueryValue.String = SafeData.vector2str(v);
            obj.hImPrResizeW.String = v(3) - v(1) + 1;
            obj.hImPrResizeH.String = v(4) - v(2) + 1;

            % Fade all other previews
            obj.disableProcessingPreviews([0 0; 1 1; 1 1]);
        end

        function createGUI(obj)
            % Create the figure (and hide it)
            obj.hFig = figure('Visible', 'off');
            GUISettings.applyFigureStyle(obj.hFig);
            obj.hFig.Name = 'SeqSLAM Settings';
            obj.hFig.Resize = 'off';

            % Create the dropdown list for toggling the setting screens
            obj.hScreen = uicontrol('Style', 'popupmenu');
            obj.hScreen.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hScreen);
            obj.hScreen.String = ConfigureGUI.SCREENS;

            % Create the image processing panel
            obj.hImPrRef = uicontrol('Style', 'text');
            obj.hImPrRef.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrRef);
            obj.hImPrRef.String = 'Reference Image Preview:';
            obj.hImPrRef.HorizontalAlignment = 'left';

            obj.hImPrRefSample = uicontrol('Style', 'popupmenu');
            obj.hImPrRefSample.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrRefSample);
            obj.hImPrRefSample.String = obj.listImagesRef;

            obj.hImPrRefAxCrop = axes();
            GUISettings.applyUIAxesStyle(obj.hImPrRefAxCrop);
            obj.hImPrRefAxCrop.Visible = 'off';

            obj.hImPrRefAxResize = axes();
            GUISettings.applyUIAxesStyle(obj.hImPrRefAxResize);
            obj.hImPrRefAxResize.Visible = 'off';

            obj.hImPrRefAxNorm = axes();
            GUISettings.applyUIAxesStyle(obj.hImPrRefAxNorm);
            obj.hImPrRefAxNorm.Visible = 'off';

            obj.hImPrQuery = uicontrol('Style', 'text');
            obj.hImPrQuery.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrQuery);
            obj.hImPrQuery.String = 'Query Image Preview:';
            obj.hImPrQuery.HorizontalAlignment = 'left';

            obj.hImPrQuerySample = uicontrol('Style', 'popupmenu');
            obj.hImPrQuerySample.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrQuerySample);
            obj.hImPrQuerySample.String = obj.listImagesQuery;

            obj.hImPrQueryAxCrop = axes();
            GUISettings.applyUIAxesStyle(obj.hImPrQueryAxCrop);
            obj.hImPrQueryAxCrop.Visible = 'off';

            obj.hImPrQueryAxResize = axes();
            GUISettings.applyUIAxesStyle(obj.hImPrQueryAxResize);
            obj.hImPrQueryAxResize.Visible = 'off';

            obj.hImPrQueryAxNorm = axes();
            GUISettings.applyUIAxesStyle(obj.hImPrQueryAxNorm);
            obj.hImPrQueryAxNorm.Visible = 'off';

            obj.hImPrRefresh = uicontrol('Style', 'pushbutton');
            obj.hImPrRefresh.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrRefresh);
            obj.hImPrRefresh.String = 'Refresh previews';

            obj.hImPrCropRef = uicontrol('Style', 'text');
            obj.hImPrCropRef.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrCropRef);
            obj.hImPrCropRef.String = 'Reference image crop:';

            obj.hImPrCropRefValue = uicontrol('Style', 'edit');
            obj.hImPrCropRefValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrCropRefValue);
            obj.hImPrCropRefValue.String = '';
            obj.hImPrCropRefValue.Enable = 'off';

            obj.hImPrCropQuery = uicontrol('Style', 'text');
            obj.hImPrCropQuery.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrCropQuery);
            obj.hImPrCropQuery.String = 'Query image crop:';

            obj.hImPrCropQueryValue = uicontrol('Style', 'edit');
            obj.hImPrCropQueryValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrCropQueryValue);
            obj.hImPrCropQueryValue.String = '';
            obj.hImPrCropQueryValue.Enable = 'off';

            obj.hImPrResize = uicontrol('Style', 'text');
            obj.hImPrResize.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResize);
            obj.hImPrResize.String = 'Resized dimensions:';

            obj.hImPrResizeW = uicontrol('Style', 'edit');
            obj.hImPrResizeW.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResizeW);
            obj.hImPrResizeW.String = '';

            obj.hImPrResizeX = uicontrol('Style', 'text');
            obj.hImPrResizeX.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResizeX);
            obj.hImPrResizeX.String = 'x';

            obj.hImPrResizeH = uicontrol('Style', 'edit');
            obj.hImPrResizeH.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResizeH);
            obj.hImPrResizeH.String = '';

            obj.hImPrResizeLock = uicontrol('Style', 'checkbox');
            obj.hImPrResizeLock.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResizeLock);
            obj.hImPrResizeLock.String = 'Lock aspect ratio';
            obj.hImPrResizeLock.Value = 1;

            obj.hImPrResizeMethod = uicontrol('Style', 'text');
            obj.hImPrResizeMethod.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResizeMethod);
            obj.hImPrResizeMethod.String = 'Resize method:';

            obj.hImPrResizeMethodValue = uicontrol('Style', 'popupmenu');
            obj.hImPrResizeMethodValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrResizeMethodValue);
            obj.hImPrResizeMethodValue.String = {'lanczos3' 'TODO'};

            obj.hImPrNorm = uicontrol('Style', 'text');
            obj.hImPrNorm.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrNorm);
            obj.hImPrNorm.String = 'Normalisation:';

            obj.hImPrNormThresh = uicontrol('Style', 'text');
            obj.hImPrNormThresh.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrNormThresh);
            obj.hImPrNormThresh.String = 'Edge threshold:';
            obj.hImPrNormThresh.HorizontalAlignment = 'left';

            obj.hImPrNormThreshValue = uicontrol('Style', 'edit');
            obj.hImPrNormThreshValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrNormThreshValue);
            obj.hImPrNormThreshValue.String = '';

            obj.hImPrNormStrength = uicontrol('Style', 'text');
            obj.hImPrNormStrength.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrNormStrength);
            obj.hImPrNormStrength.String = 'Enhance strength:';
            obj.hImPrNormStrength.HorizontalAlignment = 'left';

            obj.hImPrNormStrengthValue = uicontrol('Style', 'edit');
            obj.hImPrNormStrengthValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hImPrNormStrengthValue);
            obj.hImPrNormStrengthValue.String = '';

            % Create the matching panel
            obj.hMatchSearchTitle = uicontrol('Style', 'text');
            obj.hMatchSearchTitle.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchSearchTitle);
            obj.hMatchSearchTitle.String = 'Sequence Search Settings';
            obj.hMatchSearchTitle.FontWeight = 'bold';

            obj.hMatchSearchAx = axes();
            GUISettings.applyUIAxesStyle(obj.hMatchSearchAx);
            obj.hMatchSearchAx.YDir = 'reverse';
            obj.hMatchSearchAx.Visible = 'off';

            obj.hMatchSearchLength = annotation(obj.hFig, 'textbox');
            obj.hMatchSearchLength.String = 'Sequence length ($d_s$):';
            GUISettings.applyAnnotationStyle(obj.hMatchSearchLength);

            obj.hMatchSearchLengthValue = uicontrol('Style', 'edit');
            obj.hMatchSearchLengthValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchSearchLengthValue);
            obj.hMatchSearchLengthValue.String = '';

            obj.hMatchSearchVmin = annotation(obj.hFig, 'textbox');
            obj.hMatchSearchVmin.String = ...
                'Minimum search velocity ($v_{min}$):';
            GUISettings.applyAnnotationStyle(obj.hMatchSearchVmin);

            obj.hMatchSearchVminValue = uicontrol('Style', 'edit');
            obj.hMatchSearchVminValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchSearchVminValue);
            obj.hMatchSearchVminValue.String = '';

            obj.hMatchSearchVmax = annotation(obj.hFig, 'textbox');
            obj.hMatchSearchVmax.String = ...
                'Maximum search velocity ($v_{max}$):';
            GUISettings.applyAnnotationStyle(obj.hMatchSearchVmax);

            obj.hMatchSearchVmaxValue = uicontrol('Style', 'edit');
            obj.hMatchSearchVmaxValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchSearchVmaxValue);
            obj.hMatchSearchVmaxValue.String = '';

            obj.hMatchSearchMethod = annotation(obj.hFig, 'textbox');
            GUISettings.applyAnnotationStyle(obj.hMatchSearchMethod);
            obj.hMatchSearchMethod.String = 'Sequence search method:';

            obj.hMatchSearchMethodValue = uicontrol('Style', 'popupmenu');
            obj.hMatchSearchMethodValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchSearchMethodValue);
            obj.hMatchSearchMethodValue.String = {'Trajectory-based', ...
                'Cone-based', 'Hybrid'};

            obj.hMatchSearchVstep = annotation(obj.hFig, 'textbox');
            obj.hMatchSearchVstep.String = 'Trajectory $v_{step}$:';
            GUISettings.applyAnnotationStyle(obj.hMatchSearchVstep);

            obj.hMatchSearchVstepValue = uicontrol('Style', 'edit');
            obj.hMatchSearchVstepValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchSearchVstepValue);
            obj.hMatchSearchVstepValue.String = '';

            obj.hMatchCriTitle = uicontrol('Style', 'text');
            obj.hMatchCriTitle.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchCriTitle);
            obj.hMatchCriTitle.String = 'Match Selection Settings';
            obj.hMatchCriTitle.FontWeight = 'bold';

            obj.hMatchCriAx = axes();
            GUISettings.applyUIAxesStyle(obj.hMatchCriAx);
            obj.hMatchCriAx.XAxisLocation = 'top';
            obj.hMatchCriAx.Visible = 'off';

            obj.hMatchCriMethod = annotation(obj.hFig, 'textbox');
            obj.hMatchCriMethod.String = 'Selection Method:';
            GUISettings.applyAnnotationStyle(obj.hMatchCriMethod);

            obj.hMatchCriMethodValue = uicontrol('Style', 'popupmenu');
            obj.hMatchCriMethodValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchCriMethodValue);
            obj.hMatchCriMethodValue.String = ...
                { 'Windowed Uniqueness' 'Basic Thresholding' };

            obj.hMatchCriWindow = annotation(obj.hFig, 'textbox');
            obj.hMatchCriWindow.String = 'Exclusion window ($r_{window}$):';
            GUISettings.applyAnnotationStyle(obj.hMatchCriWindow);

            obj.hMatchCriWindowValue = uicontrol('Style', 'edit');
            obj.hMatchCriWindowValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchCriWindowValue);
            obj.hMatchCriWindowValue.String = '';

            obj.hMatchCriU = annotation(obj.hFig, 'textbox');
            obj.hMatchCriU.String = ...
                'Uniqueness factor ($\mu = \frac{min_2}{min_1}$):';
            GUISettings.applyAnnotationStyle(obj.hMatchCriU);

            obj.hMatchCriUValue = uicontrol('Style', 'edit');
            obj.hMatchCriUValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchCriUValue);
            obj.hMatchCriUValue.String = '';

            obj.hMatchCriThreshold = annotation(obj.hFig, 'textbox');
            obj.hMatchCriThreshold.String = 'Threshold ($\lambda$):';
            GUISettings.applyAnnotationStyle(obj.hMatchCriThreshold);

            obj.hMatchCriThresholdValue = uicontrol('Style', 'edit');
            obj.hMatchCriThresholdValue.Parent = obj.hFig;
            GUISettings.applyUIControlStyle(obj.hMatchCriThresholdValue);
            obj.hMatchCriThresholdValue.String = '';

            obj.hMscDiff = uipanel();
            GUISettings.applyUIPanelStyle(obj.hMscDiff);
            obj.hMscDiff.Title = 'Difference Matrix Contrast Enhancement';

            obj.hMscDiffEnh = annotation(obj.hMscDiff, 'textbox');
            obj.hMscDiffEnh.String = ...
                'Enhancement Window Size ($R_{window}$):';
            GUISettings.applyAnnotationStyle(obj.hMscDiffEnh);

            obj.hMscDiffEnhValue = uicontrol('Style', 'edit');
            obj.hMscDiffEnhValue.Parent = obj.hMscDiff;
            GUISettings.applyUIControlStyle(obj.hMscDiffEnhValue);
            obj.hMscDiffEnhValue.String = '';

            obj.hMscGt = uipanel();
            GUISettings.applyUIPanelStyle(obj.hMscGt);
            obj.hMscGt.Title = 'Ground Truth Configuration';

            obj.hMscGtStatus = uicontrol('Style', 'text');
            obj.hMscGtStatus.Parent = obj.hMscGt;
            GUISettings.applyUIControlStyle(obj.hMscGtStatus);
            obj.hMscGtStatus.HorizontalAlignment = 'left';
            obj.hMscGtStatus.FontAngle = 'italic';
            obj.hMscGtStatus.String = '';

            obj.hMscGtConfigure = uicontrol('Style', 'pushbutton');
            obj.hMscGtConfigure.Parent = obj.hMscGt;
            GUISettings.applyUIControlStyle(obj.hMscGtConfigure);
            obj.hMscGtConfigure.String = 'Configure';

            obj.hMscGtRemove = uicontrol('Style', 'pushbutton');
            obj.hMscGtRemove.Parent = obj.hMscGt;
            GUISettings.applyUIControlStyle(obj.hMscGtRemove);
            obj.hMscGtRemove.String = 'Remove';

            obj.hMscGtOptimise = uicontrol('Style', 'text');
            obj.hMscGtOptimise.Parent = obj.hMscGt;
            GUISettings.applyUIControlStyle(obj.hMscGtOptimise);
            obj.hMscGtOptimise.HorizontalAlignment = 'left';
            obj.hMscGtOptimise.String = ...
                'Use ground truth to optimise match threshold selection:';

            obj.hMscGtOptimiseValue = uicontrol('Style', 'popupmenu');
            obj.hMscGtOptimiseValue.Parent = obj.hMscGt;
            GUISettings.applyUIControlStyle(obj.hMscGtOptimiseValue);
            obj.hMscGtOptimiseValue.String = {'Off', ...
                'Optimise by precision', 'Optimise by recall', ...
                'Optimise by F1 score'};

            obj.hMscBatch = uipanel();
            GUISettings.applyUIPanelStyle(obj.hMscBatch);
            obj.hMscBatch.Title = 'Parameter sweeping through batch operation';

            obj.hMscBatchEnabled = uicontrol('Style', 'checkbox');
            obj.hMscBatchEnabled.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchEnabled);
            obj.hMscBatchEnabled.String = 'Batch operation mode enabled';

            obj.hMscBatchParam = uicontrol('Style', 'text');
            obj.hMscBatchParam.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchParam);
            obj.hMscBatchParam.HorizontalAlignment = 'left';
            obj.hMscBatchParam.String = 'Sweep parameter name:';

            obj.hMscBatchParamValue = uicontrol('Style', 'edit');
            obj.hMscBatchParamValue.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchParamValue);
            obj.hMscBatchParamValue.String = '';

            obj.hMscBatchValues = uicontrol('Style', 'text');
            obj.hMscBatchValues.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchValues);
            obj.hMscBatchValues.HorizontalAlignment = 'left';
            obj.hMscBatchValues.String = 'Sweep parameter values:';

            obj.hMscBatchValuesValue = uicontrol('Style', 'edit');
            obj.hMscBatchValuesValue.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchValuesValue);
            obj.hMscBatchValuesValue.String = '';

            obj.hMscBatchParallelise = uicontrol('Style', 'checkbox');
            obj.hMscBatchParallelise.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchParallelise);
            obj.hMscBatchParallelise.String = 'Parallelise tests';

            obj.hMscBatchTrim = uicontrol('Style', 'checkbox');
            obj.hMscBatchTrim.Parent = obj.hMscBatch;
            GUISettings.applyUIControlStyle(obj.hMscBatchTrim);
            obj.hMscBatchTrim.String = ...
                'Trim individual test results from batch results file';

            obj.hMscUI = uipanel();
            GUISettings.applyUIPanelStyle(obj.hMscUI);
            obj.hMscUI.Title = 'Graphical User Interface Controls';

            obj.hMscUIProgressType = uicontrol('Style', 'text');
            obj.hMscUIProgressType.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIProgressType);
            obj.hMscUIProgressType.HorizontalAlignment = 'left';
            obj.hMscUIProgressType.String = 'Progress UI Style:';

            obj.hMscUIProgressTypeValue = uicontrol('Style', 'popupmenu');
            obj.hMscUIProgressTypeValue.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIProgressTypeValue);
            obj.hMscUIProgressTypeValue.String = {'Graphical', 'Console'};

            obj.hMscUIResults = uicontrol('Style', 'checkbox');
            obj.hMscUIResults.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIResults);
            obj.hMscUIResults.String = 'Open a Results GUI on completion';

            obj.hMscUIWarn = uicontrol('Style', 'text');
            obj.hMscUIWarn.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIWarn);
            obj.hMscUIWarn.String = ['Note: Lowering the below values can ' ...
                'significantly decrease performance!'];
            obj.hMscUIWarn.ForegroundColor = GUISettings.COL_ERROR;
            obj.hMscUIWarn.FontWeight = 'bold';
            obj.hMscUIWarn.FontAngle = 'italic';

            obj.hMscUIPerc = uicontrol('Style', 'text');
            obj.hMscUIPerc.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIPerc);
            obj.hMscUIPerc.String = 'Percent value update frequency (%):';
            obj.hMscUIPerc.HorizontalAlignment = 'left';

            obj.hMscUIPercValue = uicontrol('Style', 'edit');
            obj.hMscUIPercValue.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIPercValue);
            obj.hMscUIPercValue.String = '';

            obj.hMscUIPrepro = uicontrol('Style', 'text');
            obj.hMscUIPrepro.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIPrepro);
            obj.hMscUIPrepro.String = 'Preprocessing visualisation update frequency (%):';
            obj.hMscUIPrepro.HorizontalAlignment = 'left';

            obj.hMscUIPreproValue = uicontrol('Style', 'edit');
            obj.hMscUIPreproValue.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIPreproValue);
            obj.hMscUIPreproValue.String = '';

            obj.hMscUIDiff = uicontrol('Style', 'text');
            obj.hMscUIDiff.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIDiff);
            obj.hMscUIDiff.String = 'Difference matrix visualisation update frequency (%):';
            obj.hMscUIDiff.HorizontalAlignment = 'left';

            obj.hMscUIDiffValue = uicontrol('Style', 'edit');
            obj.hMscUIDiffValue.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIDiffValue);
            obj.hMscUIDiffValue.String = '';

            obj.hMscUIContr = uicontrol('Style', 'text');
            obj.hMscUIContr.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIContr);
            obj.hMscUIContr.String = 'Local contrast visualisation update frequency (%):';
            obj.hMscUIContr.HorizontalAlignment = 'left';

            obj.hMscUIContrValue = uicontrol('Style', 'edit');
            obj.hMscUIContrValue.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIContrValue);
            obj.hMscUIContrValue.String = '';

            obj.hMscUIMatch = uicontrol('Style', 'text');
            obj.hMscUIMatch.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIMatch);
            obj.hMscUIMatch.String = 'Matching visualisation update frequency (%):';
            obj.hMscUIMatch.HorizontalAlignment = 'left';

            obj.hMscUIMatchValue = uicontrol('Style', 'edit');
            obj.hMscUIMatchValue.Parent = obj.hMscUI;
            GUISettings.applyUIControlStyle(obj.hMscUIMatchValue);
            obj.hMscUIMatchValue.String = '';

            % Done button
            obj.hDone = uicontrol('Style', 'pushbutton');
            GUISettings.applyUIControlStyle(obj.hDone);
            obj.hDone.String = 'Done';

            % Callbacks (must be last, otherwise empty objects passed...)
            obj.hScreen.Callback = {@obj.cbChangeScreen};
            obj.hImPrRefSample.Callback = {@obj.cbChangePreviewImage};
            obj.hImPrQuerySample.Callback = {@obj.cbChangePreviewImage};
            obj.hImPrRefresh.Callback = {@obj.cbRefreshPreprocessed};
            obj.hImPrResizeW.Callback = {@obj.cbChangeResize};
            obj.hImPrResizeH.Callback = {@obj.cbChangeResize};
            obj.hImPrResizeLock.Callback = {@obj.cbLockAspectRatio};
            obj.hImPrResizeMethodValue.Callback = {@obj.cbChangeResize};
            obj.hImPrNormThreshValue.Callback = {@obj.cbChangeNorm};
            obj.hImPrNormStrengthValue.Callback = {@obj.cbChangeNorm};
            obj.hMatchSearchLengthValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMatchSearchVminValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMatchSearchVmaxValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMatchSearchMethodValue.Callback = {@obj.cbChooseSearchMethod};
            obj.hMatchSearchVstepValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMatchCriMethodValue.Callback = {@obj.cbSelectMatchMethod};
            obj.hMatchCriWindowValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMatchCriUValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMatchCriThresholdValue.Callback = {@obj.cbRefreshDiagrams};
            obj.hMscGtConfigure.Callback = {@obj.cbConfigureGroundTruth};
            obj.hMscGtRemove.Callback = {@obj.cbRemoveGroundTruth};
            obj.hMscBatchEnabled.Callback = {@obj.cbBatchToggle};
            obj.hDone.Callback = {@obj.cbDone};
        end

        function disableProcessingPreviews(obj, mask)
            axs = [ ...
                obj.hImPrRefAxCrop   , obj.hImPrQueryAxCrop; ...
                obj.hImPrRefAxResize , obj.hImPrQueryAxResize; ...
                obj.hImPrRefAxNorm   , obj.hImPrQueryAxNorm ...
                ];
            axs = axs(logical(mask));
            arrayfun(@(x) alpha(x, ConfigureGUI.IMAGE_FADE), axs);
        end

        function drawMatchDiagrams(obj)
            % Useful temporaries
            ds = str2num(obj.hMatchSearchLengthValue.String);
            vmin = str2num(obj.hMatchSearchVminValue.String);
            vmax = str2num(obj.hMatchSearchVmaxValue.String);
            vstep = str2num(obj.hMatchSearchVstepValue.String);

            rwin = str2num(obj.hMatchCriWindowValue.String);

            % Draw the trajectory preview diagram
            FACTOR = 1.5;
            sz = round(0.5*ds*FACTOR) * 2 - 1;  % Ensure there is always a middle
            center = ceil(0.5*sz);
            offBack = floor((ds-1)/2);
            offFront = floor(ds/2);
            if (sz ~= length(obj.dataTraj))
                obj.dataTraj = rand(sz);
            end
            cla(obj.hMatchSearchAx);
            hold(obj.hMatchSearchAx, 'on');
            obj.hMatchSearchAx.Visible = 'off';
            h = imagesc(obj.dataTraj, 'Parent', obj.hMatchSearchAx);
            h.AlphaData = 0.5;
            h = plot(obj.hMatchSearchAx, center, center, 'k.');
            h.MarkerSize = h.MarkerSize * 6;
            for k = [vmin:vstep:vmax vmax]
                th = atan(k);
                if k == vmin || k == vmax || ...
                        obj.hMatchSearchMethodValue.Value == 1
                    h =plot(obj.hMatchSearchAx, ...
                        [center-offBack center+offFront], ...
                        [center-offBack*sin(th) center+offFront*sin(th)], 'k');
                end
                if k == vmin || k == vmax
                    h.LineWidth = h.LineWidth * 3;
                end
            end

            % Add text annotations to the trajectory preview diagram
            h = text(obj.hMatchSearchAx, center+offFront, ...
                center+offFront*sin(atan(vmin)), '$v_{min}$', ...
                'interpreter', 'latex', 'HorizontalAlignment', 'left', ...
                'VerticalAlignment', 'bottom');
            h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
            h = text(obj.hMatchSearchAx, center+offFront, ...
                center+offFront*sin(atan(vmax)), '$v_{max}$', ...
                'interpreter', 'latex', 'HorizontalAlignment', 'left', ...
                'VerticalAlignment', 'top');
            h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
            if obj.hMatchSearchMethodValue.Value == 1
                h = text(obj.hMatchSearchAx, center+offFront, ...
                    center+offFront*mean([sin(atan(vmin)) sin(atan(vmax))]), ...
                    '$v_{step}$', 'interpreter', 'latex', 'HorizontalAlignment', ...
                    'left', 'VerticalAlignment', 'middle');
                h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
            end
            plot(obj.hMatchSearchAx, [center-offBack center+offFront], ...
                [sz sz], 'k');
            h = plot(obj.hMatchSearchAx, center-offBack, sz, 'k<');
            h.MarkerFaceColor = 'k';
            h = plot(obj.hMatchSearchAx, center+offFront, sz, 'k>');
            h.MarkerFaceColor = 'k';
            h = text(obj.hMatchSearchAx, center, sz-0.5, '$d_s$', ...
                'interpreter', 'latex', 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'bottom');
            h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
            hold(obj.hMatchSearchAx, 'off');

            % Draw match selection diagram for the chosen method
            if (obj.hMatchCriMethodValue.Value == 2)
                % Draw the thresholded selction method diagram
                if (length(obj.dataThresh) ~= 100)
                    obj.dataThresh = -1 * rand(1, 100);
                end
                cla(obj.hMatchCriAx);
                hold(obj.hMatchCriAx, 'on');
                h = plot(obj.hMatchCriAx, obj.dataThresh, 'k.');
                h.MarkerSize = h.MarkerSize * 2;
                r = rectangle(obj.hMatchCriAx, 'Position', ...
                    [0 -1 100 0.4]);
                r.FaceColor = [GUISettings.COL_SUCCESS 0.25];
                r.EdgeColor = 'none';
                h = plot(obj.hMatchCriAx, [0 100], [-0.6 -0.6], 'k:');
                h.LineWidth = h.LineWidth * 2;
                hold(obj.hMatchCriAx, 'off');
                obj.hMatchCriAx.YLim = [-1 0];
                obj.hMatchCriAx.XLabel.String = 'query image #';
                obj.hMatchCriAx.XTick = [];
                obj.hMatchCriAx.YLabel.String = 'trajectory score';
                obj.hMatchCriAx.YTick = [];

                % Add text annotations for the thresholded selection method
                h = text(obj.hMatchCriAx, 100, -0.6, '$\lambda$', ...
                    'interpreter', 'latex', 'HorizontalAlignment', 'right', ...
                    'VerticalAlignment', 'top');
                h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
            else
                % Draw the windowed selection method diagram
                if (length(obj.dataWindow) ~= 5*rwin)
                    obj.dataWindow = -1 * rand(1, 5*rwin);
                    obj.dataWindow(2*rwin:3*rwin-1) = ...
                        obj.dataWindow(2*rwin:3*rwin-1) - 0.5;
                    obj.dataWindow(end/2) = -1.5;
                end
                minwin = min(obj.dataWindow(2*rwin:3*rwin-1));
                minout = min([obj.dataWindow(1:2*rwin-1) ...
                    obj.dataWindow(3*rwin:end)]);
                cla(obj.hMatchCriAx);
                hold(obj.hMatchCriAx, 'on');
                h = plot(obj.hMatchCriAx, obj.dataWindow, 'k.');
                h.MarkerSize = h.MarkerSize * 2;
                r = rectangle(obj.hMatchCriAx, 'Position', ...
                    [2*rwin -1.75 rwin 1.75]);
                r.FaceColor = [GUISettings.COL_DEFAULT 0.25];
                r.EdgeColor = 'none';
                h = plot(obj.hMatchCriAx, [0 5*rwin], [minwin minwin], 'k');
                h.LineWidth = h.LineWidth * 2;
                h = plot(obj.hMatchCriAx, [0 5*rwin], [minout minout], 'k:');
                h.LineWidth = h.LineWidth * 2;
                hold(obj.hMatchCriAx, 'off');
                obj.hMatchCriAx.YLim = [-1.75 0];
                obj.hMatchCriAx.XLabel.String = 'reference image #';
                obj.hMatchCriAx.XTick = [];
                obj.hMatchCriAx.YLabel.String = 'trajectory score';
                obj.hMatchCriAx.YTick = [];

                % Add text annotations to the windowed selection method diagram
                h = text(obj.hMatchCriAx, 5*rwin, minout, '$min_2$', ...
                    'interpreter', 'latex', 'HorizontalAlignment', 'right', ...
                    'VerticalAlignment', 'top');
                h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
                h = text(obj.hMatchCriAx, 5*rwin, minwin, '$min_1$', ...
                    'interpreter', 'latex', 'HorizontalAlignment', 'right', ...
                    'VerticalAlignment', 'bottom');
                h.FontSize = h.FontSize * GUISettings.LATEX_FACTOR * 1.25;
            end
        end

        function drawProcessingPreviews(obj)
            % Get a config representing current UI state
            % TODO VERY lazy inefficient way to do this...
            configTemp = obj.config;
            obj.strip();
            configCurrent = obj.config;
            obj.config = configTemp;

            % Get vectors representing the crop limits
            cropRef = SafeData.str2vector(obj.hImPrCropRefValue.String);
            cropQuery = SafeData.str2vector(obj.hImPrCropQueryValue.String);

            % Generate all of the required images
            refImg = datasetOpenImage(obj.config.reference, ...
                obj.hImPrRefSample.Value, obj.numbersRef);
            queryImg = datasetOpenImage(obj.config.query, ...
                obj.hImPrQuerySample.Value, obj.numbersQuery);
            [refImgOut, refImgs] = SeqSLAMInstance.preprocessSingle( ...
                refImg, configCurrent.seqslam.image_processing, 'reference', 1);
            [queryImgOut, queryImgs] = SeqSLAMInstance.preprocessSingle( ...
                queryImg, configCurrent.seqslam.image_processing, 'query', 1);

            % Clear all axes
            cla(obj.hImPrRefAxCrop);
            cla(obj.hImPrRefAxResize);
            cla(obj.hImPrRefAxNorm);
            cla(obj.hImPrQueryAxCrop);
            cla(obj.hImPrQueryAxResize);
            cla(obj.hImPrQueryAxNorm);

            % Show all of the images
            imshow(refImg, 'Parent', obj.hImPrRefAxCrop);
            imshow(refImgs{2}, 'Parent', obj.hImPrRefAxResize);
            imshow(refImgOut, 'Parent', obj.hImPrRefAxNorm);
            imshow(queryImg, 'Parent', obj.hImPrQueryAxCrop);
            imshow(queryImgs{2}, 'Parent', obj.hImPrQueryAxResize);
            imshow(queryImgOut, 'Parent', obj.hImPrQueryAxNorm);

            % Draw the crop-boxes
            obj.hImPrRefCropBox = imrect(obj.hImPrRefAxCrop, ...
                [cropRef(1) cropRef(2) cropRef(3)-cropRef(1) ...
                cropRef(4)-cropRef(2)]);
            obj.hImPrRefCropBox.setPositionConstraintFcn( ...
                @obj.constrainedRefPosition);
            obj.hImPrQueryCropBox = imrect(obj.hImPrQueryAxCrop, ...
                [cropQuery(1) cropQuery(2) cropQuery(3)-cropQuery(1) ...
                cropQuery(4)-cropQuery(2)]);
            obj.hImPrQueryCropBox.setPositionConstraintFcn( ...
                @obj.constrainedQueryPosition);
            obj.hImPrQueryCropBox.setFixedAspectRatioMode(1);
        end

        function generateImageLists(obj)
            obj.numbersRef = SeqSLAMInstance.numbers(obj.config.reference);
            obj.listImagesRef = datasetImageList(obj.config.reference, ...
                obj.numbersRef);
            obj.numbersQuery = SeqSLAMInstance.numbers(obj.config.query);
            obj.listImagesQuery = datasetImageList(obj.config.query, ...
                obj.numbersQuery);
        end

        function valid = isDataValid(obj, screen)
            valid = false;
            if isempty(screen) || screen == 1
                % Normalisation threshold
                v = str2num(obj.hImPrNormThreshValue.String);
                if v < 0 || v > 1
                    uiwait(errordlg(['Normalisation edge threshold must ' ...
                        'be in range [0, 1]'], 'modal'));
                    return;
                end

                % Normalisation strength
                v = str2num(obj.hImPrNormStrengthValue.String);
                if v < -1 || v > 1
                    uiwait(errordlg( ...
                        'Normalisation strength must be in range [-1, 1]', ...
                        'modal'));
                    return;
                end
            end

            valid = true;
        end

        function openScreen(obj, screen)
            % Clear everything off the screen
            obj.clearScreen();

            % Add the appropriate elements for the screen
            if (screen == 1)
                % Image preprocessing settings
                HelpPopup.setDestination(obj.hHelp, ...
                    'seqslam_settings/image_preprocessing');

                % Show the appropriate options and axes
                obj.hImPrRef.Visible = 'on';
                obj.hImPrRefSample.Visible = 'on';
                obj.hImPrRefAxCrop.Visible = 'on';
                obj.hImPrRefAxResize.Visible = 'on';
                obj.hImPrRefAxNorm.Visible = 'on';
                obj.hImPrQuery.Visible = 'on';
                obj.hImPrQuerySample.Visible = 'on';
                obj.hImPrQueryAxCrop.Visible = 'on';
                obj.hImPrQueryAxResize.Visible = 'on';
                obj.hImPrQueryAxNorm.Visible = 'on';
                obj.hImPrRefresh.Visible = 'on';
                obj.hImPrCropRef.Visible = 'on';
                obj.hImPrCropRefValue.Visible = 'on';
                obj.hImPrCropQuery.Visible = 'on';
                obj.hImPrCropQueryValue.Visible = 'on';
                obj.hImPrResize.Visible = 'on';
                obj.hImPrResizeW.Visible = 'on';
                obj.hImPrResizeX.Visible = 'on';
                obj.hImPrResizeH.Visible = 'on';
                obj.hImPrResizeLock.Visible = 'on';
                obj.hImPrResizeMethod.Visible = 'on';
                obj.hImPrResizeMethodValue.Visible = 'on';
                obj.hImPrNorm.Visible = 'on';
                obj.hImPrNormThresh.Visible = 'on';
                obj.hImPrNormThreshValue.Visible = 'on';
                obj.hImPrNormStrength.Visible = 'on';
                obj.hImPrNormStrengthValue.Visible = 'on';

                % Force a refresh of all of the previews
                obj.drawProcessingPreviews();
            elseif (screen == 2)
                % Matching settings
                HelpPopup.setDestination(obj.hHelp, ...
                    'seqslam_settings/sequence_matching');

                % Show the appropriate options and axes
                obj.hMatchSearchTitle.Visible = 'on';
                obj.hMatchSearchAx.Visible = 'on';
                obj.hMatchSearchLength.Visible = 'on';
                obj.hMatchSearchLengthValue.Visible = 'on';
                obj.hMatchSearchVmin.Visible = 'on';
                obj.hMatchSearchVminValue.Visible = 'on';
                obj.hMatchSearchVmax.Visible = 'on';
                obj.hMatchSearchVmaxValue.Visible = 'on';
                obj.hMatchSearchMethod.Visible = 'on';
                obj.hMatchSearchMethodValue.Visible = 'on';
                if obj.hMatchSearchMethodValue.Value == 1
                    obj.hMatchSearchVstep.Visible = 'on';
                    obj.hMatchSearchVstepValue.Visible = 'on';
                end
                obj.hMatchCriTitle.Visible = 'on';
                obj.hMatchCriAx.Visible = 'on';
                obj.hMatchCriMethod.Visible = 'on';
                obj.hMatchCriMethodValue.Visible = 'on';
                obj.hMatchCriWindow.Visible = 'on';
                obj.hMatchCriWindowValue.Visible = 'on';
                obj.hMatchCriU.Visible = 'on';
                obj.hMatchCriUValue.Visible = 'on';
                obj.hMatchCriThreshold.Visible = 'on';
                obj.hMatchCriThresholdValue.Visible = 'on';

                % Force a refresh of all of the diagrams
                obj.drawMatchDiagrams();

                % Force the calling of any necessary callbacks
                obj.cbSelectMatchMethod(obj.hMatchCriMethodValue, []);
            elseif (screen == 3)
                % Other settings
                HelpPopup.setDestination(obj.hHelp, ...
                    'seqslam_settings/miscellaneous');

                % Show the appropriate options
                obj.hMscDiff.Visible = 'on';
                obj.hMscGt.Visible = 'on';
                obj.hMscBatch.Visible = 'on';
                obj.hMscUI.Visible = 'on';
            end

            % Force a draw at the end
            drawnow();
        end

        function populate(obj)
            % Use the first image in each dataset to get some reference dimensions
            obj.dimRef = size(datasetOpenImage(obj.config.reference, 1, ...
                obj.numbersRef));
            obj.dimQuery = size(datasetOpenImage(obj.config.query, 1, ...
                obj.numbersQuery));

            % Dump all data from the config struct to the UI
            obj.hImPrCropRefValue.String = SafeData.noEmpty( ...
                SafeData.vector2str( ...
                obj.config.seqslam.image_processing.crop.reference), ...
                ['1, 1, ' num2str(obj.dimRef(2)) ', ' num2str(obj.dimRef(1))]);
            obj.hImPrCropQueryValue.String = SafeData.noEmpty( ...
                SafeData.vector2str( ...
                obj.config.seqslam.image_processing.crop.query), ['1, 1, ' ...
                num2str(obj.dimQuery(2)) ', ' num2str(obj.dimQuery(1))]);
            obj.hImPrResizeW.String = SafeData.noEmpty( ...
                obj.config.seqslam.image_processing.downsample.width, ...
                obj.dimRef(2));
            obj.hImPrResizeH.String = SafeData.noEmpty( ...
                obj.config.seqslam.image_processing.downsample.height, ...
                obj.dimRef(1));
            obj.hImPrNormThreshValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.image_processing.normalisation.threshold,...
                0.5);
            obj.hImPrNormStrengthValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.image_processing.normalisation.strength, ...
                0.5);

            obj.hMatchSearchLengthValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.search.d_s, 10);
            obj.hMatchSearchVminValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.search.v_min, 0.8);
            obj.hMatchSearchVmaxValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.search.v_max, 1.2);
            method = SafeData.noEmpty(obj.config.seqslam.search.method, ...
                'traj');
            if strcmpi(method, 'hybrid')
                obj.hMatchSearchMethodValue.Value = 3;
            elseif strcmpi(method, 'cone')
                obj.hMatchSearchMethodValue.Value = 2;
            else
                obj.hMatchSearchMethodValue.Value = 1;
            end
            obj.hMatchSearchVstepValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.search.method_traj.v_step, 0.1);
            method = SafeData.noEmpty(obj.config.seqslam.matching.method, ...
                'window');
            if strcmpi(method, 'thresh')
                obj.hMatchCriMethodValue.Value = 2;
            else
                obj.hMatchCriMethodValue.Value = 1;
            end
            obj.hMatchCriWindowValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.matching.method_window.r_window, 10);
            obj.hMatchCriUValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.matching.method_window.u, 1.11);
            obj.hMatchCriThresholdValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.matching.method_thresh.threshold, 2.5);

            obj.hMscDiffEnhValue.String = SafeData.noEmpty( ...
                obj.config.seqslam.diff_matrix.contrast.r_window, 10);
            opt = obj.config.ground_truth.use_to_auto_optimise;
            if strcmpi(opt, 'f1')
                obj.hMscGtOptimiseValue.Value = 4;
            elseif strcmpi(opt, 'p')
                obj.hMscGtOptimiseValue.Value = 3;
            elseif strcmpi(opt, 'r')
                obj.hMscGtOptimiseValue.Value = 2;
            else
                obj.hMscGtOptimiseValue.Value = 1;
            end

            obj.hMscBatchEnabled.Value = SafeData.noEmpty( ...
                obj.config.batch.enabled, false);
            obj.cbBatchToggle(obj.hMscBatchEnabled, []);
            obj.hMscBatchParamValue.String = SafeData.noEmpty( ...
                obj.config.batch.param, '');
            obj.hMscBatchValuesValue.String = SafeData.vector2str( ...
                obj.config.batch.values);
            obj.hMscBatchParallelise.Value = SafeData.noEmpty( ...
                obj.config.batch.parallelise, false);
            obj.hMscBatchTrim.Value = SafeData.noEmpty( ...
                obj.config.batch.trim_results, true);
            ui = SafeData.noEmpty(obj.config.ui.progress.type, 'graphical');
            if strcmpi(ui, 'console')
                obj.hMscUIProgressTypeValue.Value = 2;
            else
                obj.hMscUIProgressTypeValue.Value = 1;
            end
            obj.hMscUIResults.Value = SafeData.noEmpty( ...
                obj.config.ui.results, false);
            obj.hMscUIPercValue.String = SafeData.noEmpty( ...
                obj.config.ui.progress.percent_freq, 1);
            obj.hMscUIPreproValue.String = SafeData.noEmpty( ...
                obj.config.ui.progress.preprocess_freq, 5);
            obj.hMscUIDiffValue.String = SafeData.noEmpty( ...
                obj.config.ui.progress.diff_matrix_freq, 5);
            obj.hMscUIContrValue.String = SafeData.noEmpty( ...
                obj.config.ui.progress.enhance_freq, 5);
            obj.hMscUIMatchValue.String = SafeData.noEmpty( ...
                obj.config.ui.progress.match_freq, 5);

            % Handle populating the ground truth
            if obj.config.ground_truth.exists
                obj.cachedGroundTruth = obj.config.ground_truth;
            else
                obj.cachedGroundTruth = [];
            end
            obj.updateGroundTruthOptions();
        end

        function sizeGUI(obj)
            % Get some reference dimensions (max width of headings, and
            % default height of a button
            widthUnit = obj.hImPrRefresh.Extent(3);
            heightUnit = obj.hDone.Extent(4);

            % Size and position of the figure
            obj.hFig.Position = [0, 0, ...
                widthUnit * ConfigureGUI.FIG_WIDTH_FACTOR, ...
                heightUnit * ConfigureGUI.FIG_HEIGHT_FACTOR];
            movegui(obj.hFig, 'center');

            % Now that the figure (space for placing UI elements is set),
            % size all of the elements
            SpecSize.size(obj.hScreen, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.66);

            SpecSize.size(obj.hImPrRef, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.33, GUISettings.PAD_MED);
            SpecSize.size(obj.hImPrRefSample, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRef);
            SpecSize.size(obj.hImPrRefAxCrop, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRef);
            SpecSize.size(obj.hImPrRefAxCrop, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 2/3);
            SpecSize.size(obj.hImPrRefAxResize, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRef);
            SpecSize.size(obj.hImPrRefAxResize, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 2/3);
            SpecSize.size(obj.hImPrRefAxNorm, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRef);
            SpecSize.size(obj.hImPrRefAxNorm, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 2/3);
            SpecSize.size(obj.hImPrQuery, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.33, GUISettings.PAD_MED);
            SpecSize.size(obj.hImPrQuerySample, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrQuery);
            SpecSize.size(obj.hImPrQueryAxCrop, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrQuery);
            SpecSize.size(obj.hImPrQueryAxCrop, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 2/3);
            SpecSize.size(obj.hImPrQueryAxResize, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrQuery);
            SpecSize.size(obj.hImPrQueryAxResize, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 2/3);
            SpecSize.size(obj.hImPrQueryAxNorm, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrQuery);
            SpecSize.size(obj.hImPrQueryAxNorm, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 2/3);
            SpecSize.size(obj.hImPrRefresh, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.33, GUISettings.PAD_MED);
            SpecSize.size(obj.hImPrCropRef, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hImPrRefresh);
            SpecSize.size(obj.hImPrCropRefValue, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRefresh);
            SpecSize.size(obj.hImPrCropQuery, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRefresh);
            SpecSize.size(obj.hImPrCropQueryValue, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRefresh);
            SpecSize.size(obj.hImPrResize, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hImPrRefresh);
            SpecSize.size(obj.hImPrResizeW, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.2);
            SpecSize.size(obj.hImPrResizeX, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.1, GUISettings.PAD_MED);
            SpecSize.size(obj.hImPrResizeH, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.2);
            SpecSize.size(obj.hImPrResizeLock, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.4);
            SpecSize.size(obj.hImPrResizeMethod, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRefresh);
            SpecSize.size(obj.hImPrResizeMethodValue, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hImPrRefresh);
            SpecSize.size(obj.hImPrNorm, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hImPrRefresh);
            SpecSize.size(obj.hImPrNormThresh, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.4);
            SpecSize.size(obj.hImPrNormThreshValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.3);
            SpecSize.size(obj.hImPrNormStrength, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.4);
            SpecSize.size(obj.hImPrNormStrengthValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hImPrRefresh, 0.3);

            SpecSize.size(obj.hMatchSearchTitle, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig);
            SpecSize.size(obj.hMatchSearchAx, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.5, 4*GUISettings.PAD_LARGE);
            SpecSize.size(obj.hMatchSearchAx, SpecSize.HEIGHT, ...
                SpecSize.RATIO, 1);
            SpecSize.size(obj.hMatchSearchLength, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchSearchLengthValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchSearchVmin, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchSearchVminValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchSearchVmax, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchSearchVmaxValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchSearchMethod, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchSearchMethodValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchSearchVstep, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchSearchVstepValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchCriTitle, SpecSize.WIDTH, ...
                SpecSize.MATCH, obj.hFig);
            SpecSize.size(obj.hMatchCriAx, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.5, 4*GUISettings.PAD_LARGE);
            SpecSize.size(obj.hMatchCriAx, SpecSize.HEIGHT, SpecSize.RATIO, ...
                0.5);
            SpecSize.size(obj.hMatchCriMethod, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchCriMethodValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchCriWindow, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchCriWindowValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchCriU, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hFig, 0.25);
            SpecSize.size(obj.hMatchCriUValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);
            SpecSize.size(obj.hMatchCriThreshold, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.25);
            SpecSize.size(obj.hMatchCriThresholdValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);

            SpecSize.size(obj.hMscDiff, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscDiff, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.1);
            SpecSize.size(obj.hMscDiffEnh, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscDiff, 0.35, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hMscDiffEnhValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscDiff, 0.1, GUISettings.PAD_MED);

            SpecSize.size(obj.hMscGt, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscGt, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.15);
            SpecSize.size(obj.hMscGtStatus, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscGt, 0.75);
            SpecSize.size(obj.hMscGtStatus, SpecSize.HEIGHT, ...
                SpecSize.PERCENT, obj.hMscGt, 0.3);
            SpecSize.size(obj.hMscGtConfigure, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscGtRemove, SpecSize.WIDTH, SpecSize.WRAP, ...
                GUISettings.PAD_MED);
            SpecSize.size(obj.hMscGtOptimise, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscGt, 0.4);
            SpecSize.size(obj.hMscGtOptimiseValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscGt, 0.3);

            SpecSize.size(obj.hMscBatch, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscBatch, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.2);
            SpecSize.size(obj.hMscBatchEnabled, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.25);
            SpecSize.size(obj.hMscBatchParam, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.2);
            SpecSize.size(obj.hMscBatchParamValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.75);
            SpecSize.size(obj.hMscBatchValues, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.2);
            SpecSize.size(obj.hMscBatchValuesValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.75);
            SpecSize.size(obj.hMscBatchParallelise, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.5);
            SpecSize.size(obj.hMscBatchTrim, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscBatch, 0.5);

            SpecSize.size(obj.hMscUI, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hFig, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscUI, SpecSize.HEIGHT, SpecSize.PERCENT, ...
                obj.hFig, 0.3);
            SpecSize.size(obj.hMscUIProgressType, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.15);
            SpecSize.size(obj.hMscUIProgressTypeValue, SpecSize.WIDTH, ...
                SpecSize.WRAP, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscUIResults, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.5);
            SpecSize.size(obj.hMscUIWarn, SpecSize.WIDTH, SpecSize.MATCH, ...
                obj.hMscUI, GUISettings.PAD_LARGE);
            SpecSize.size(obj.hMscUIPerc, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hMscUI, 0.35, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hMscUIPercValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.1, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscUIPrepro, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.35, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hMscUIPreproValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.1, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscUIDiff, SpecSize.WIDTH, SpecSize.PERCENT, ...
                obj.hMscUI, 0.35, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hMscUIDiffValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.1, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscUIContr, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.35, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hMscUIContrValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.1, GUISettings.PAD_MED);
            SpecSize.size(obj.hMscUIMatch, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.35, GUISettings.PAD_SMALL);
            SpecSize.size(obj.hMscUIMatchValue, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hMscUI, 0.1, GUISettings.PAD_MED);

            SpecSize.size(obj.hDone, SpecSize.WIDTH, ...
                SpecSize.PERCENT, obj.hFig, 0.2);

            % Then, systematically place
            SpecPosition.positionIn(obj.hFig, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hFig, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionIn(obj.hScreen, obj.hFig, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hScreen, obj.hFig, ...
                SpecPosition.CENTER_X);

            SpecPosition.positionRelative(obj.hImPrRef, obj.hScreen, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hImPrRef, obj.hFig, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrRefSample, obj.hImPrRef, ...
                SpecPosition.BELOW);
            SpecPosition.positionRelative(obj.hImPrRefSample, obj.hImPrRef, ...
                SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrRefAxCrop, ...
                obj.hImPrRefSample, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrRefAxCrop, obj.hImPrRef, ...
                SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrRefAxResize, ...
                obj.hImPrRefAxCrop, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrRefAxResize, ...
                obj.hImPrRef, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrRefAxNorm, ...
                obj.hImPrRefAxResize, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrRefAxNorm, obj.hImPrRef, ...
                SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrQuery, obj.hImPrRef, ...
                SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hImPrQuery, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hImPrQuerySample, ...
                obj.hImPrQuery, SpecPosition.BELOW);
            SpecPosition.positionRelative(obj.hImPrQuerySample, ...
                obj.hImPrQuery, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrQueryAxCrop, ...
                obj.hImPrQuerySample, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrQueryAxCrop, ...
                obj.hImPrQuery, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrQueryAxResize, ...
                obj.hImPrQueryAxCrop, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrQueryAxResize, ...
                obj.hImPrQuery, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrQueryAxNorm, ...
                obj.hImPrQueryAxResize, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrQueryAxNorm, ...
                obj.hImPrQuery, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrRefresh, ...
                obj.hImPrRefSample, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hImPrRefresh, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrCropRef, ...
                obj.hImPrRefAxCrop, SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrCropRef, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrCropRefValue, ...
                obj.hImPrCropRef, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrCropRefValue, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrCropQuery, ...
                obj.hImPrCropRefValue, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrCropQuery, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrCropQueryValue, ...
                obj.hImPrCropQuery, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrCropQueryValue, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrResize, ...
                obj.hImPrRefAxResize, SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrResize, obj.hImPrRefresh, ...
                SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrResizeW, obj.hImPrResize, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrResizeW, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrResizeX, ...
                obj.hImPrResizeW, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hImPrResizeX, ...
                obj.hImPrResizeW, SpecPosition.RIGHT_OF, GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hImPrResizeH, ...
                obj.hImPrResizeW, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hImPrResizeH, ...
                obj.hImPrResizeX, SpecPosition.RIGHT_OF, GUISettings.PAD_SMALL);
            SpecPosition.positionRelative(obj.hImPrResizeLock, ...
                obj.hImPrResizeW, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hImPrResizeLock, ...
                obj.hImPrRefresh, SpecPosition.RIGHT);
            SpecPosition.positionRelative(obj.hImPrResizeMethod, ...
                obj.hImPrResizeW, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrResizeMethod, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrResizeMethodValue, ...
                obj.hImPrResizeMethod, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrResizeMethodValue, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrNorm, obj.hImPrRefAxNorm, ...
                SpecPosition.TOP, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrNorm, obj.hImPrRefresh, ...
                SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrNormThresh, ...
                obj.hImPrNorm, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrNormThresh, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrNormThreshValue, ...
                obj.hImPrNormThresh, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hImPrNormThreshValue, ...
                obj.hImPrNormThresh, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hImPrNormStrength, ...
                obj.hImPrNormThresh, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hImPrNormStrength, ...
                obj.hImPrRefresh, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hImPrNormStrengthValue, ...
                obj.hImPrNormStrength, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hImPrNormStrengthValue, ...
                obj.hImPrNormStrength, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hMatchSearchTitle, obj.hScreen, ...
                SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hMatchSearchTitle, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchSearchAx, ...
                obj.hMatchSearchTitle, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hMatchSearchAx, obj.hFig, ...
                SpecPosition.LEFT, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMatchSearchLengthValue, ...
                obj.hMatchSearchTitle, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hMatchSearchLengthValue, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMatchSearchLength, ...
                obj.hMatchSearchLengthValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchSearchLength, ...
                obj.hMatchSearchLengthValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchVminValue, ...
                obj.hMatchSearchLengthValue, SpecPosition.BELOW, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchVminValue, ...
                obj.hMatchSearchLengthValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchSearchVmin, ...
                obj.hMatchSearchVminValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchSearchVmin, ...
                obj.hMatchSearchVminValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchVmaxValue, ...
                obj.hMatchSearchVminValue, SpecPosition.BELOW, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchVmaxValue, ...
                obj.hMatchSearchVminValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchSearchVmax, ...
                obj.hMatchSearchVmaxValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchSearchVmax, ...
                obj.hMatchSearchVmaxValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchMethodValue, ...
                obj.hMatchSearchVmaxValue, SpecPosition.BELOW, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchMethodValue, ...
                obj.hMatchSearchVmaxValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchSearchMethod, ...
                obj.hMatchSearchMethodValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchSearchMethod, ...
                obj.hMatchSearchMethodValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchVstepValue, ...
                obj.hMatchSearchMethodValue, SpecPosition.BELOW, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchSearchVstepValue, ...
                obj.hMatchSearchMethodValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchSearchVstep, ...
                obj.hMatchSearchVstepValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchSearchVstep, ...
                obj.hMatchSearchVstepValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchCriTitle, ...
                obj.hMatchSearchAx, SpecPosition.BELOW, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMatchCriTitle, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchCriAx, ...
                obj.hMatchCriTitle, SpecPosition.BELOW, ...
                2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMatchCriAx, obj.hFig, ...
                SpecPosition.LEFT, 3*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMatchCriMethodValue, ...
                obj.hMatchCriTitle, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMatchCriMethodValue, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMatchCriMethod, ...
                obj.hMatchCriMethodValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchCriMethod, ...
                obj.hMatchCriMethodValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchCriWindowValue, ...
                obj.hMatchCriMethodValue, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMatchCriWindowValue, ...
                obj.hMatchCriMethodValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchCriWindow, ...
                obj.hMatchCriWindowValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchCriWindow, ...
                obj.hMatchCriWindowValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchCriUValue, ...
                obj.hMatchCriWindowValue, SpecPosition.BELOW, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchCriUValue, ...
                obj.hMatchCriWindowValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchCriU, ...
                obj.hMatchCriUValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchCriU, ...
                obj.hMatchCriUValue, SpecPosition.LEFT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMatchCriThresholdValue, ...
                obj.hMatchCriMethodValue, SpecPosition.BELOW, ...
                GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMatchCriThresholdValue, ...
                obj.hMatchCriMethodValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMatchCriThreshold, ...
                obj.hMatchCriThresholdValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMatchCriThreshold, ...
                obj.hMatchCriThresholdValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);

            SpecPosition.positionRelative(obj.hMscDiff, obj.hScreen, ...
                SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscDiff, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hMscDiffEnh, obj.hMscDiff, ...
                SpecPosition.TOP, 2.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscDiffEnh, obj.hMscDiff, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscDiffEnhValue, ...
                obj.hMscDiffEnh, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscDiffEnhValue, ...
                obj.hMscDiffEnh, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscGt, obj.hMscDiff, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscGt, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hMscGtStatus, obj.hMscGt, ...
                SpecPosition.TOP, 2.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscGtStatus, obj.hMscGt, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscGtRemove, ...
                obj.hMscGtStatus, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hMscGtRemove, obj.hMscGt, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMscGtConfigure, ...
                obj.hMscGtStatus, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscGtConfigure, ...
                obj.hMscGtRemove, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscGtOptimise, ...
                obj.hMscGtStatus, SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMscGtOptimise, ...
                obj.hMscGtStatus, SpecPosition.LEFT);
            SpecPosition.positionRelative(obj.hMscGtOptimiseValue, ...
                obj.hMscGtOptimise, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscGtOptimiseValue, ...
                obj.hMscGtOptimise, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscBatch, obj.hMscGt, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscBatch, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hMscBatchEnabled, obj.hMscBatch, ...
                SpecPosition.TOP, 1.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscBatchEnabled, obj.hMscBatch, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscBatchParam, ...
                obj.hMscBatchEnabled, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hMscBatchParam, obj.hMscBatch, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscBatchParamValue, ...
                obj.hMscBatchParam, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscBatchParamValue, ...
                obj.hMscBatchParam, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscBatchValues, ...
                obj.hMscBatchParam, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscBatchValues, ...
                obj.hMscBatchParam, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMscBatchValuesValue, ...
                obj.hMscBatchValues, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscBatchValuesValue, ...
                obj.hMscBatchValues, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscBatchParallelise, ...
                obj.hMscBatchValues, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hMscBatchParallelise, ...
                obj.hMscBatch, SpecPosition.LEFT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMscBatchTrim, ...
                obj.hMscBatchParallelise, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hMscBatchTrim, obj.hMscBatch, ...
                SpecPosition.RIGHT);
            SpecPosition.positionRelative(obj.hMscUI, obj.hMscBatch, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscUI, obj.hFig, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionIn(obj.hMscUIProgressType, obj.hMscUI, ...
                SpecPosition.TOP, 2.5*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscUIProgressType, obj.hMscUI, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIProgressTypeValue, ...
                obj.hMscUIProgressType, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscUIProgressTypeValue, ...
                obj.hMscUIProgressType, SpecPosition.RIGHT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIResults, ...
                obj.hMscUIProgressType, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hMscUIResults, obj.hMscUI, ...
                SpecPosition.RIGHT, GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMscUIWarn, ...
                obj.hMscUIProgressType, SpecPosition.BELOW, ...
                2*GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscUIWarn, obj.hMscUI, ...
                SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMscUIPerc, obj.hMscUIWarn, ...
                SpecPosition.BELOW, GUISettings.PAD_LARGE);
            SpecPosition.positionIn(obj.hMscUIPerc, obj.hMscUI, ...
                SpecPosition.LEFT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIPercValue, ...
                obj.hMscUIPerc, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscUIPercValue, ...
                obj.hMscUIPerc, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIPrepro, ...
                obj.hMscUIPerc, SpecPosition.BELOW, 2*GUISettings.PAD_LARGE);
            SpecPosition.positionRelative(obj.hMscUIPrepro, ...
                obj.hMscUIPerc, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMscUIPreproValue, ...
                obj.hMscUIPrepro, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscUIPreproValue, ...
                obj.hMscUIPrepro, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIDiff, ...
                obj.hMscUIPrepro, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIDiff, ...
                obj.hMscUIPrepro, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMscUIDiffValue, ...
                obj.hMscUIDiff, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscUIDiffValue, ...
                obj.hMscUIDiff, SpecPosition.RIGHT_OF, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIContrValue, ...
                obj.hMscUIPrepro, SpecPosition.CENTER_Y);
            SpecPosition.positionIn(obj.hMscUIContrValue, obj.hMscUI, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIContr, ...
                obj.hMscUIContrValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscUIContr, ...
                obj.hMscUIContrValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIMatchValue, ...
                obj.hMscUIContrValue, SpecPosition.BELOW, GUISettings.PAD_MED);
            SpecPosition.positionRelative(obj.hMscUIMatchValue, ...
                obj.hMscUIContrValue, SpecPosition.CENTER_X);
            SpecPosition.positionRelative(obj.hMscUIMatch, ...
                obj.hMscUIMatchValue, SpecPosition.CENTER_Y);
            SpecPosition.positionRelative(obj.hMscUIMatch, ...
                obj.hMscUIMatchValue, SpecPosition.LEFT_OF, ...
                GUISettings.PAD_MED);

            SpecPosition.positionIn(obj.hDone, obj.hFig, ...
                SpecPosition.RIGHT, GUISettings.PAD_MED);
            SpecPosition.positionIn(obj.hDone, obj.hFig, ...
                SpecPosition.BOTTOM, GUISettings.PAD_MED);
        end

        function strip(obj)
            % Strip data from the UI, and store it in the config struct
            v = SafeData.str2vector(obj.hImPrCropRefValue.String);
            if isequal(v, [1 1 obj.dimRef(2) obj.dimRef(1)])
                obj.config.seqslam.image_processing.crop.reference = [];
            else
                obj.config.seqslam.image_processing.crop.reference = v;
            end
            v = SafeData.str2vector(obj.hImPrCropQueryValue.String);
            if isequal(v, [1 1 obj.dimQuery(2) obj.dimQuery(1)])
                obj.config.seqslam.image_processing.crop.query = [];
            else
                obj.config.seqslam.image_processing.crop.query = v;
            end
            obj.config.seqslam.image_processing.downsample.width = ...
                str2num(obj.hImPrResizeW.String);
            obj.config.seqslam.image_processing.downsample.height = ...
                str2num(obj.hImPrResizeH.String);
            obj.config.seqslam.image_processing.downsample.method = ...
                obj.hImPrResizeMethodValue.String{ ...
                obj.hImPrResizeMethodValue.Value};
            obj.config.seqslam.image_processing.normalisation.threshold = ...
                str2num(obj.hImPrNormThreshValue.String);
            obj.config.seqslam.image_processing.normalisation.strength = ...
                str2num(obj.hImPrNormStrengthValue.String);

            obj.config.seqslam.search.d_s = ...
                str2num(obj.hMatchSearchLengthValue.String);
            obj.config.seqslam.search.v_min = ...
                str2num(obj.hMatchSearchVminValue.String);
            obj.config.seqslam.search.v_max = ...
                str2num(obj.hMatchSearchVmaxValue.String);
            if obj.hMatchSearchMethodValue.Value == 3
                obj.config.seqslam.search.method = 'hybrid';
            elseif obj.hMatchSearchMethodValue.Value == 2
                obj.config.seqslam.search.method = 'cone';
            else
                obj.config.seqslam.search.method = 'traj';
                obj.config.seqslam.search.method_traj.v_step = ...
                    str2num(obj.hMatchSearchVstepValue.String);
            end
            if obj.hMatchCriMethodValue.Value == 2
                obj.config.seqslam.matching.method = 'thresh';
                obj.config.seqslam.matching.method_thresh.threshold = ...
                    str2num(obj.hMatchCriThresholdValue.String);
            else
                obj.config.seqslam.matching.method = 'window';
                obj.config.seqslam.matching.method_window.r_window = ...
                    str2num(obj.hMatchCriWindowValue.String);
                obj.config.seqslam.matching.method_window.u = ...
                    str2num(obj.hMatchCriUValue.String);
            end

            obj.config.seqslam.diff_matrix.contrast.r_window = ...
                str2num(obj.hMscDiffEnhValue.String);
            if ~isempty(obj.cachedGroundTruth)
                obj.config.ground_truth = obj.cachedGroundTruth;
                obj.config.ground_truth.exists = true;
                if obj.hMscGtOptimiseValue.Value == 4
                    obj.config.ground_truth.use_to_auto_optimise = 'f1';
                elseif obj.hMscGtOptimiseValue.Value == 3
                    obj.config.ground_truth.use_to_auto_optimise = 'r';
                elseif obj.hMscGtOptimiseValue.Value == 2
                    obj.config.ground_truth.use_to_auto_optimise = 'p';
                else
                    obj.config.ground_truth.use_to_auto_optimise = [];
                end
            else
                obj.config.ground_truth.exists = false;
            end
            if obj.hMscBatchEnabled.Value == 1
                obj.config.batch.enabled = true;
            else
                obj.config.batch.enabled = false;
            end
            obj.config.batch.param = obj.hMscBatchParamValue.String;
            obj.config.batch.values = SafeData.str2vector( ...
                obj.hMscBatchValuesValue.String);
            if obj.hMscBatchParallelise.Value == 1
                obj.config.batch.parallelise = true;
            else
                obj.config.batch.parallelise = false;
            end
            if obj.hMscBatchTrim.Value == 1
                obj.config.batch.trim_results = true;
            else
                obj.config.batch.trim_results = false;
            end
            if obj.hMscUIProgressTypeValue.Value == 2
                obj.config.ui.progress.type = 'console';
            else
                obj.config.ui.progress.type = 'graphical';
            end
            if obj.hMscUIResults.Value == 1
                obj.config.ui.results = true;
            else
                obj.config.ui.results = false;
            end
            obj.config.ui.progress.percent_freq = ...
                str2num(obj.hMscUIPercValue.String);
            obj.config.ui.progress.preprocess_freq = ...
                str2num(obj.hMscUIPreproValue.String);
            obj.config.ui.progress.diff_matrix_freq = ...
                str2num(obj.hMscUIDiffValue.String);
            obj.config.ui.progress.enhance_freq = ...
                str2num(obj.hMscUIContrValue.String);
            obj.config.ui.progress.match_freq = ...
                str2num(obj.hMscUIMatchValue.String);
        end

        function updateGroundTruthOptions(obj)
            % Update the status string
            [obj.hMscGtStatus.String, ...
                obj.hMscGtStatus.ForegroundColor] = ...
                GroundTruthPopup.gtDescription(obj.cachedGroundTruth);

            % Update the optimisation options
            if isempty(obj.cachedGroundTruth) || isequal( ...
                    obj.hMscGtStatus.ForegroundColor, GUISettings.COL_ERROR)
                status = 'off';
            else
                status = 'on';
            end
            obj.hMscGtOptimise.Enable = status;
            obj.hMscGtOptimiseValue.Enable = status;
        end
    end
end
