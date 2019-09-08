function [exists, value] = findDeepField(struc, deepName)
    % Parse the list of fields from the deep name
    fields = strsplit(deepName, '.');

    % Recursively check for existence of the subfield
    value = [];
    s = struc;
    for k = 1:length(fields)
        f = fields{k};
        if isfield(s, f)
            s = getfield(s, f);
        else
            exists = false;
            return;
        end
    end
    exists = true;
    value = s;
end
