clear 
close all

types = ["delight","m2dp","sc","bow","gist"];
run_seq = ["../results/KITTI/seq00/", "../results/KITTI/seq02/", "../results/KITTI/seq05/", "../results/KITTI/seq06/", "../results/KITTI/seq07/"];

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

[AUC, top_recall, lp_detected] = run_test(type, hist, hist, gt, gt, loop_diff, mask_width);

figure('Name', 'Trajectory')
plot3(gt(:,1),gt(:,2),gt(:,3), '.-')
hold on
for i=1:size(lp_detected,1)
    plot3([gt(lp_detected(i,1),1);gt(lp_detected(i,2),1)],[gt(lp_detected(i,1),2);gt(lp_detected(i,2),2)],[gt(lp_detected(i,1),3);gt(lp_detected(i,2),3)],'ro')
    hold on
end
axis equal
title('Trajectory')
end
