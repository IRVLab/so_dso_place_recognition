classdef SpecSize < uint16

    enumeration
        % Specify dimension
        WIDTH (0)
        HEIGHT (1)
        
        % Specify sizing method
        MATCH (2)
        PERCENT (3)
        ABSOLUTE (4)
        WRAP (5)
        RATIO (6)
    end
    
    methods (Static)
        function size(handle, dim, method, varargin)
            p = inputParser;
            p.addRequired('handle', @ishandle);
            p.addRequired('dim', ...
                @(x) x == SpecSize.WIDTH || x == SpecSize.HEIGHT);
            p.addRequired('method', ...
                @(x) x >= SpecSize.MATCH && x <= SpecSize.RATIO);
            p.parse(handle, dim, method);
            
            if method == SpecSize.MATCH
                % Expected arguments: handle, dim, method, handleref
                % Optional arguments: padding
                if nargin < 4 || ~ishandle(varargin{1})
                    error('Invalid: Match needs a reference handle');
                end
                if nargin > 4 && ~isnumeric(varargin{2})
                    error('Invalid: Padding must be numeric');
                elseif nargin < 5
                    padding = 0;
                else
                    padding = varargin{2};
                end
                SpecSize.sizeMatch(handle, dim, varargin{1}, padding);
            elseif method == SpecSize.PERCENT
                % Expected arguments: handle, dim, method, handleref
                % Optional arguments: percent, padding
                if nargin < 4 || ~ishandle(varargin{1})
                    error('Invalid: Percent needs a reference handle');
                end
                if nargin > 4 && ~isnumeric(varargin{2})
                    error('Invalid: Percent value must be numeric');
                elseif nargin < 5
                    percent = 1.0;
                else
                    percent = varargin{2};
                end
                if nargin > 5 && ~isnumeric(varargin{3})
                    error('Invalid: Padding must be numeric');
                elseif nargin < 6
                    padding = 0;
                else
                    padding = varargin{3};
                end
                SpecSize.sizePercent(handle, dim, varargin{1}, percent, ...
                    padding);
            elseif method == SpecSize.ABSOLUTE
                % Expected arguments: handle, dim, method, value
                if nargin < 4 || ~isnumeric(varargin{1})
                    error('Invalid: Absolute needs a numeric value');
                end
                SpecSize.sizeAbsolute(handle, dim, varargin{1});
            elseif method == SpecSize.WRAP
                % Expected arguments: handle, dim, method
                % Optional arguments: padding
                if nargin > 3 && ~isnumeric(varargin{1})
                    error('Invalid: Padding must be numeric');
                elseif nargin < 4
                    padding = 0;
                else
                    padding = varargin{1};
                end
                SpecSize.sizeWrap(handle, dim, padding);
            elseif method == SpecSize.RATIO
                % Expected arguments: handle, dim, method
                % Optional arguments: ratio
                if nargin > 3 && ~isnumeric(varargin{1})
                    error('Invalid: Ratio must be numeric');
                elseif nargin < 4
                    ratio = 1;
                else
                    ratio = varargin{1};
                end
                SpecSize.sizeRatio(handle, dim, ratio);
            end
        end
    end

    methods (Static, Access = private)
        function sizeAbsolute(handle, dim, value)
            handle.Position(3+dim) = value; 
        end

        function sizeMatch(handle, dim, handleRef, padding)
           if (nargin < 3)
               padding = 0;
           end
           handle.Position(3+dim) = handleRef.Position(3+dim) - 2*padding;
        end
        
        function sizePercent(handle, dim, handleRef, percent, padding)
            if (nargin < 4)
               padding = 0;
            end
           handle.Position(3+dim) = handleRef.Position(3+dim)*percent - ...
               2*padding;
        end
        
        function sizeRatio(handle, dim, ratio)
            handle.Position(3+dim) = handle.Position(4-dim) * ratio;
        end

        function sizeWrap(handle, dim, padding)
            handle.Position(3+dim) = 1000;
            handle.Position(3+dim) = handle.Extent(3+dim) + 2*padding;
        end
    end
end

