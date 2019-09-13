function [numbers, ext, startToken, endToken] = datasetPictureProfile(directory)
    % TODO the logical in this function is a little bit weak. It does not handle
    % if:
    % - the start and end tokens are different (only the dominant token should
    %   be used)

    % Get all of the possible image extensions
    exts = arrayfun(@(x) x.ext, imformats, 'uni', 0);
    exts = [exts{:}];

    % Loop over every possible image extension, recording the extension that
    % matches the most image files in the directory
    directorySafe = fullfile(directory);
    ext = '';
    startToken = ''; endToken = '';
    numbers = [];
    for k = 1:length(exts)
        % Get the filenames of all files matching that extension
        fns = dir([directorySafe filesep() '*.' exts{k}]);
        fns = {fns.name};

        % Skip the rest of the loop if: no files matched, or number of matches
        % is less than what has been matched for a previous extension
        if isempty(fns) || length(fns) < length(numbers)
            continue;
        end

        % Use regex on the filenames, and extract the tokens (the first token is
        % the prefix, second number, and third postfix)
        tokens = regexp(fns, ['^(.*?)(\d+)(.?\.' exts{k} ')'], ...
            'tokens');
        numbers = cellfun(@(x) str2num(x{1}{2}), tokens);
        ext = exts{k};
        startToken = tokens{1}{1}{1}; endToken = tokens{1}{1}{3};
    end

    % Sort the numbers before we return
    numbers = sort(numbers);
end
