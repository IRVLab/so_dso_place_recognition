classdef SpecPosition

    enumeration
        % Positioning options
        LEFT_OF
        RIGHT_OF
        ABOVE
        BELOW
        
        % Alignment options
        CENTER_X
        CENTER_Y
        LEFT
        RIGHT
        TOP
        BOTTOM
    end
    
    methods (Static)
        function positionRelative(handle, handleRef, method, varargin)
            p = inputParser();
            p.addRequired('handle', @ishandle);
            p.addRequired('handleRef', @ishandle);
            p.addRequired('method', @(x) isa(x, 'SpecPosition'));
            p.addOptional('offset', 0, @isnumeric);
            p.parse(handle, handleRef, method, varargin{:});
            
            SpecPosition.position(handle, handleRef.Position, method, ...
                p.Results.offset);
        end
        
        function positionIn(handle, handleParent, method, varargin)
            p = inputParser();
            p.addRequired('handle', @ishandle);
            p.addRequired('handleParent', @ishandle);
            p.addRequired('method', @(x) isa(x, 'SpecPosition'));
            p.addOptional('offset', 0, @isnumeric);
            p.parse(handle, handleParent, method, varargin{:});
            
            % Enforce that the handle is in the parent
            % TODO
            
            SpecPosition.position(handle, ...
                handleParent.Position .* [0 0 1 1], method, ...
                p.Results.offset);
        end
    end
    
    methods (Static, Access = private)
         function position(handle, refPosition, method, offset)
            % Perform the positioning
            if (method == SpecPosition.LEFT_OF)
                handle.Position(1) = refPosition(1) - ...
                    handle.Position(3) - offset;
            elseif (method == SpecPosition.RIGHT_OF)
                handle.Position(1) = refPosition(1) + refPosition(3) + ...
                    offset;
            elseif (method == SpecPosition.ABOVE)
                handle.Position(2) = refPosition(2) + refPosition(4) + ...
                    offset;
            elseif (method == SpecPosition.BELOW)
                handle.Position(2) = refPosition(2) - ...
                    handle.Position(4) - offset;
            elseif (method == SpecPosition.CENTER_X)
                handle.Position(1) = refPosition(1) + ...
                    0.5*(refPosition(3) - handle.Position(3)) + offset;
            elseif (method == SpecPosition.CENTER_Y)
                handle.Position(2) = refPosition(2) + ...
                    0.5*(refPosition(4) - handle.Position(4)) + offset;
            elseif (method == SpecPosition.LEFT)
                handle.Position(1) = refPosition(1) + offset;
            elseif (method == SpecPosition.RIGHT)
                handle.Position(1) = refPosition(1) + refPosition(3) - ...
                    handle.Position(3) - offset;
            elseif (method == SpecPosition.TOP)
                handle.Position(2) = refPosition(2) + refPosition(4) - ...
                    handle.Position(4) - offset;
            elseif (method == SpecPosition.BOTTOM)
                handle.Position(2) = refPosition(2) + offset;
            end
        end
    end
end

