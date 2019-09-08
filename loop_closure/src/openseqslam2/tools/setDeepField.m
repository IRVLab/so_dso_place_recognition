function s = setDeepField(struc, deepName, value)
    % Parse the list of fields from the deep name
    fields = strsplit(deepName, '.');

    % Set the deep field
    s = setfield(struc, fields{:}, value);
end
