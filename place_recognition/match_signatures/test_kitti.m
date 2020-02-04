clear 
close all

% AUCs =
%DELIGHT  0.7426    0.6216    0.9185
%M2DP     0.6373    0.5715    0.9484
%SC       0.7321    0.6525    0.8974
%BoW      0.8934    0.8674    0.9682
%GIST     0.8412    0.7563    0.9253
% 
% TRs =
%DELIGHT  0.5340    0.4931    0.5172
%M2DP     0.1611    0.1367    0.9416
%SC       0.5648    0.5691    0.6790
%BoW      0.7883    0.8094    0.9629
%GIST     0.7741    0.6588    0.7294

types = ["delight","m2dp","sc","bow","gist";];

run_seq = ["../results/KITTI/seq00/", "../results/KITTI/seq05/", "../results/KITTI/seq06/"];

TRs = zeros(size(types, 2), size(run_seq, 1));
AUCs = zeros(size(types, 2), size(run_seq, 1));
for ti=1:size(types,2)
    for si=1:size(run_seq,2)
        [AUCs(ti, si), TRs(ti, si)] = run_kitti(types(ti), run_seq(si));
        pause(0.5);
    end
end
AUCs 
TRs

function [AUC, top_recall] = run_kitti(type, path)
mask_width = 100;
loop_diff = 10;

% load data
incoming_id = load(strcat(path, 'incoming_id_file.txt'))+1;
gt_full = load(strcat(path, 'gt.txt'));
gt = gt_full(incoming_id, [4,8,12]);
hist = load(strcat(strcat(strcat(strcat(path, 'history_')), type), '.txt'));

[AUC, top_recall] = run_test(type, hist, hist, gt, gt, loop_diff, mask_width);
end
