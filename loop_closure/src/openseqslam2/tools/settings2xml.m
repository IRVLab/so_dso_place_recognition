function settings2xml(s, saveLocation)
    % Perform any substitutions
    % TODO this should be defined MUCH more robustly...
    VAR_ROOT = '$TOOLBOX_ROOT';
    s.reference.path = strrep(s.reference.path, toolboxRoot(), VAR_ROOT);
    s.query.path = strrep(s.query.path, toolboxRoot(), VAR_ROOT);
    s.results.path = strrep(s.results.path, toolboxRoot(), VAR_ROOT);
    if ~isempty(s.ground_truth.file.path)
        s.ground_truth.file.path = strrep(s.ground_truth.file.path, ...
            toolboxRoot(), VAR_ROOT);
    end

    % Create the document node, and root element
    doc = com.mathworks.xml.XMLUtils.createDocument('seqslam-settings');

    % Get the root node (which corresponds to the struct)
    root = doc.getDocumentElement();

    % Recursively loop through each of the elements of the struct, performing
    % the relevant writing operation
    writeRecursive(s, root, doc);

    % Write the XML model to the save location
    xmlwrite(saveLocation, doc);

    function writeRecursive(s, currentNode, doc)
        % Loop over all fields of the struct
        fs = fieldnames(s);
        for k = 1:length(fs)
            x = getfield(s, fs{k});

            % Add to DOM, depending on type of field (don't bother writing
            % empty fields
            if isempty(x)
                continue;
            elseif isstruct(x)
                % Add the nested settings group
                nextNode = doc.createElement('settings-group');
                nextNode.setAttribute('name', fs{k});
                currentNode.appendChild(nextNode);

                % Make the recursive call to write
                writeRecursive(x, nextNode, doc);
            else
                % Create a settings node
                settingNode = doc.createElement('setting');
                settingNode.setAttribute('name', fs{k});

                % Add the setting value (and report the type)
                if ischar(x) || isstr(x)
                    settingNode.setAttribute('value', x);
                    settingNode.setAttribute('type', 'string');
                elseif isscalar(x) && islogical(x)
                    settingNode.setAttribute('value', num2str(x));
                    settingNode.setAttribute('type', 'boolean');
                elseif isscalar(x) && isnumeric(x)
                    settingNode.setAttribute('value', num2str(x));
                    settingNode.setAttribute('type', 'numeric');
                elseif isvector(x) && isnumeric(x)
                    settingNode.setAttribute('value', SafeData.vector2str(x));
                    settingNode.setAttribute('type', 'vector');
                end

                % Add the node
                currentNode.appendChild(settingNode);
            end
        end
    end
end

