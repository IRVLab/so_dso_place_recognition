clear
close all

% type = 'M2DP';
% type = 'SC';
type = 'GIST';

hist1 = load(strcat(strcat(strcat('../../results/Tara/1/history'), type), '.txt'));
hist2 = load(strcat(strcat(strcat('../../results/Tara/3/history'), type), '.txt'));

% get distance matrix
switch type
    case 'M2DP'
        dist_m = processM2DP(hist1, hist2, 0, 0);
    case 'SC'
        dist_m = processSC(hist1, hist2, 0, 0);
    case 'GIST'
        dist_m = processGIST(hist1, hist2, 0, 0);
end

% get closest distance for each index
[dist_v, dist_idx] = min(dist_m');
[~, dist_rank] = sort(dist_v);


%% show individual results and intersection
top_count = length(dist_rank);
% top_count = 20;
lp_ti = [dist_rank(1:top_count)', dist_idx(dist_rank(1:top_count))'];
figure('Name', 'Loop Closure')
plot(lp_ti(:,1),lp_ti(:,2), 'b*');
title('Detected');
