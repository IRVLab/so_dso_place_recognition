function [selected, paramValue] = optimalMatch(allMatches, matchingConfig, ...
        gtMatrix, criteria)
    % Get the best score range
    if ~strcmp(matchingConfig.method, 'thresh')
        best = SeqSLAMInstance.usFromMatches(allMatches.min_scores, ...
            matchingConfig.method_window.r_window);
    else
        best = min(allMatches.min_scores);
    end

    % Iterate over all possible test values
    TESTS = 30;
    testVals = linspace(min(best), max(best), TESTS);
    ps = zeros(size(testVals));
    rs = zeros(size(testVals));
    f1s = zeros(size(testVals));
    for k = 1:TESTS
        if ~strcmpi(matchingConfig.method, 'thresh')
            s(k) = SeqSLAMInstance.thresholdWindowed(allMatches, ...
                matchingConfig.method_window.r_window, testVals(k));
        else
            s(k) = SeqSLAMInstance.thresholdBasic(allMatches, testVals(k));
        end
        [ps(k), rs(k)] = calcPR(s(k).matches, gtMatrix);
        f1s(k) = f1score(ps(k), rs(k));
    end

    % Get the best result based on the supplied criteria
    if strcmp(criteria, 'p')
        [best, bestI] = max(ps);
    elseif strcmp(criteria, 'r')
        [best, bestI] = max(rs);
    else
        [best, bestI] = max(f1s);
    end
    selected = s(bestI);
    paramValue = testVals(bestI);
end
