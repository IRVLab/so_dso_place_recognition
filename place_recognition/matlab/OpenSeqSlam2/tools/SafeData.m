classdef SafeData < uint16

    enumeration
        STRING (0)
        NUMERIC (1)
        BOOLEAN (2)
    end

    methods (Static)
        function valid = isValid(todo)
            % Approach to data safety here is to limit its entry into the
            % system. This means that edit boxes should validate their input
            % before allowing the user to move on. The only invalid entry is no
            % entry (i.e. empty), which we can easily handle when attempting to
            % access the values later on.
            %
            % This has the following consequences throughout the toolbox:
            % 1) extraction of values from edit boxes in GUIs can be assumed
            %    safe
            % 2) all edits must perform data validation
            % 3) similarly, bringing in of values from *.xml files must be
            %    silently evaluated (fails converted to empty)
            % 4) ALL access of values must treat the isempty case!
            %
            % TODO implement all of this in a later commit!
        end

        function val = noEmpty(data, emptyValue)
            if isempty(data)
                val = emptyValue;
            else
                val = data;
            end
        end

        function v = str2vector(s)
            if isempty(s)
                v = [];
            else
                v = cellfun(@(x) str2num(x), strsplit(s, ','), ...
                    'UniformOutput', false);
                v = [v{:}];
            end
        end

        function s = vector2str(v)
            s = strjoin(strsplit(num2str(v), ' '), ', ');
        end
    end
end
