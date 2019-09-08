function coords = matches2coords(matches)
    coords = [(1:length(matches)); matches]';
    coords = coords(~isnan(matches),:); % Coords corresponding to each match
end

