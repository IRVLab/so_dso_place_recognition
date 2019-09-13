classdef GUISettings

    properties (Constant)
        % Figure properties
        BACK_COL = 'w';
        MAIN_COL = [0.4 0.4 0.4];

        % Colouring settings
        COL_SUCCESS = [0 0.6 0];
        COL_WARNING = [1 0.4 0];
        COL_ERROR = [1 0 0];
        COL_LOADING = [0.6 0.6 0.6];
        COL_DEFAULT = [0 0 0];

        % Sizings (static for now)
        FONT_SCALE = 1.0;
        LATEX_FACTOR = 1.25;

        PAD_SMALL = 5;
        PAD_MED = 10;
        PAD_LARGE = 15;
    end

    methods (Static)
        function applyAnnotationStyle(ann)
            GUISettings.applyUICommonStyle(ann);
            ann.EdgeColor = 'none';
            ann.FontSize = get(groot, 'factoryTextboxshapeFontSize') * ...
                GUISettings.FONT_SCALE*GUISettings.LATEX_FACTOR;

            % Force the string to adopt a more consistent style when using
            % LaTex interpreter
            ann.Interpreter = 'latex';
        end

        function applyFigureStyle(fig)
            GUISettings.applyUICommonStyle(fig);
            fig.Color = GUISettings.BACK_COL;
            fig.NumberTitle = 'off';
            fig.MenuBar = 'none';
            fig.DockControls = 'off';
        end

        function applyUIAxesStyle(ax)
            GUISettings.applyUICommonStyle(ax);
            ax.Color = GUISettings.BACK_COL;
        end

        function applyUIControlStyle(control)
            GUISettings.applyUICommonStyle(control);
            control.BackgroundColor = GUISettings.BACK_COL;
            control.FontSize = get(groot, 'factoryUicontrolFontSize') * ...
                GUISettings.FONT_SCALE;
        end

        function applyUIPanelStyle(panel)
            GUISettings.applyUICommonStyle(panel);
            panel.BackgroundColor = GUISettings.BACK_COL;
            panel.FontSize = get(groot, 'factoryUipanelFontSize') * ...
                GUISettings.FONT_SCALE;
        end

        function axesHide(axes)
            axes.XTick = [];
            axes.XTickLabel = [];
            axes.XColor = 'none';
            axes.YTick = [];
            axes.YTickLabel = [];
            axes.YColor = 'none';
        end

        function axesDiffMatrixStyle(axes, limits)
            axes.Visible = 'on';
            axes.Box = 'off';
            axes.YDir = 'reverse';
            axes.XAxisLocation = 'top';
            axes.YAxisLocation = 'left';
            axes.XLim = [1 limits(2)] + [-0.5 0.5];
            axes.YLim = [1 limits(1)] + [-0.5 0.5];
            axes.XLabel.String = 'Query Image #';
            axes.YLabel.String = 'Reference Image #';
        end

        function axesDiffMatrixFocusStyle(axes, xvals, yvals)
            axes.Visible = 'on';
            GUISettings.axesHide(axes);
            axes.Box = 'on';
            axes.XColor = 'k';
            axes.YColor = 'k';
            axes.YDir = 'reverse';
            axes.XAxisLocation = 'top';
            axes.YAxisLocation = 'left';
            axes.XLim = [min(xvals) max(xvals)] + [-0.5 0.5];
            axes.YLim = [min(yvals) max(yvals)] + [-0.5 0.5];
        end

        function [out] = axesF1Style(axes, values, valueLabel)
            axes.Box = 'on';
            axes.Visible = 'on';
            axes.XColor = 'k';
            axes.YColor = 'k';
            axes.XLim = [min(values) max(values)];
            axes.YLim = [0 1];
            axes.XLabel.String = valueLabel;
            axes.XLabel.Interpreter = 'none';
            axes.YLabel.String = 'F1 Score';
        end

        function axesPrecisionRecallStyle(axes)
            axes.Box = 'on';
            axes.Visible = 'on';
            axes.XColor = 'k';
            axes.YColor = 'k';
            axes.XLim = [0 1];
            axes.YLim = [0 1];
            axes.XLabel.String = 'Recall';
            axes.YLabel.String = 'Precision';
        end

        function setFontScale(uicontrol, scale)
            uicontrol.FontSize = get(groot, 'factoryUicontrolFontSize') ...
                * scale;
        end
    end

    methods (Static, Access = private)
        function applyUICommonStyle(uielement)
            uielement.Units = 'pixels';
        end
    end
end

