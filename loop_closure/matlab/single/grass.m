clear
close all

type = 'M2DP';
type = 'SC';
type = 'GIST';

hist = load(strcat(strcat(strcat('../../results/Tara/grass/history'), type), '.txt'));

% get distance matrix
switch type
    case 'M2DP'
        [diff_m, diff_m_t, diff_m_i] = processM2DP(hist, 0, 0, -1);
    case 'SC'
        [diff_m, diff_m_t, diff_m_i] = processSC(hist, 0, 0, -1);
    case 'GIST'
        diff_m = processGIST(hist, 0, 0, -1);
end

res = zeros(size(diff_m));
for i=1:size(res,1)
    [pks,locs] = findpeaks(1-diff_m(i,:)); pks = 1-pks;
    [~,t3] = mink(pks,3,2);
    res(i, locs(t3)) = 1;
end
imshow(res)

% imshow((diff_m-min(min(diff_m)))/(max(max(diff_m))-min(min(diff_m))));
% colormap(gca,jet);
% title(type);
