function [p, r] = calcPR(matches, gtMatrix)
    % Turn the matches into the corresponding coordinates
    cs = matches2coords(matches);

    % Calculate precision (# correct matches / # matches)
    p = sum(arrayfun(@(x) gtMatrix(cs(x,1), cs(x,2)), 1:size(cs,1))) / ...
        size(cs,1);

    % Calculate recall (# correct matches / # available matches)
    r = sum(arrayfun(@(x) gtMatrix(cs(x,1), cs(x,2)), 1:size(cs,1))) / ...
        sum(max(gtMatrix));
end
