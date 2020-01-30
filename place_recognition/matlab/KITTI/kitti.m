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

% run_seq = ["../../results/KITTI/00/", "../../results/KITTI/05/", "../../results/KITTI/06/"];
run_seq = ["../../results/KITTI/07/"];

TRs = zeros(size(types, 2), size(run_seq, 1));
AUCs = zeros(size(types, 2), size(run_seq, 1));
for ti=1:size(types,2)
    for si=1:size(run_seq,2)
        [AUCs(ti, si), TRs(ti, si)] = run_test(types(ti), run_seq(si));
        pause(0.5);
    end
end
AUCs 
TRs

function [AUC, top_recall] = run_test(type, path)
mask_width = 100;
loop_dist = 10;

incoming_id = load(strcat(path, 'incoming_id_file.txt'));
gt_full = load(strcat(path, 'gt.txt'));

incoming_id = incoming_id+1;
gt = gt_full(incoming_id, :);

hist = load(strcat(strcat(strcat(strcat(path, 'history_')), type), '.txt'));

% get distance matrix 
tic 
switch type 
    case 'm2dp'
        [dist_m, dist_m_t, dist_m_i] = processM2DP(hist);
    case 'sc'
        [dist_m, dist_m_t, dist_m_i] = processSC(hist);
    case 'delight'
        dist_m = processDELIGHT(hist);
    case 'gist'
        dist_m = processGIST(hist);
    case 'bow'
        dist_m = processBoW(hist);
end
tm = toc;
% type
% tm = 1000 * tm / size(dist_m,1)
    
for i=1:size(dist_m,1)
    for j=1:size(dist_m,2)
        if(abs(i-j)<mask_width)
            dist_m(i,j) = Inf;
        end
    end
end

% get closest distance for each index
[dist_v, dist_idx] = min(dist_m');
[~, dist_rank] = sort(dist_v);

% get ground truth
lp_gt = [];
for i=1:size(gt,1)
    min_dist = inf;
    min_j = -1;
    for j=1:size(gt,1)
        if(abs(i-j)<mask_width)
            continue;
        end
        dist = gt(i,:) - gt(j,:);
        dist = dist*dist';
        if(min_dist > dist)
            min_dist = dist;
            min_j = j;
        end
    end
    if(min_dist<loop_dist*loop_dist)
        lp_gt = [lp_gt; i, min_j];
    end
end
total_lp = length(lp_gt);

% calculate true position and false position
tp = 0;
fp = 0;
precision = zeros(size(gt,1),1);
recall = zeros(size(gt,1),1);
top_recall = 0;
top_count = 0;
for i=1:size(gt,1)
    a = dist_rank(i);
    b = dist_idx(dist_rank(i));
    dist = gt(a,:) - gt(b,:);
    dist = dist*dist';
    if(dist<loop_dist*loop_dist)
        tp = tp+1;
    else
        fp = fp+1;
    end
    precision(i) = tp / (tp+fp);
    recall(i) = tp / total_lp;

    if(precision(i)==1)
        top_count = i;
        top_recall = recall(i);
    end
end
figure
subplot(1,2,1)
plot(recall, precision)
axis([0 1 0 1])
AUC = trapz(recall,precision);
title(strcat(strcat(strcat('Precision-Recall(', num2str(top_recall)), '), AUC='), num2str(AUC)));

%% show trajectory
% top_count = 250;
subplot(1,2,2)
plot3(gt(:,4), gt(:,8), gt(:,12), '.-')
hold on
% plot3(gt(lp_gt(:,1),4), gt(lp_gt(:,1),8), gt(lp_gt(:,1),12), 'g.')
% hold on
% plot3(gt(lp_gt(:,2),4), gt(lp_gt(:,2),8), gt(lp_gt(:,2),12), 'g.')
% hold on
lp_detected = [dist_rank(1:top_count)', dist_idx(dist_rank(1:top_count))'];
for i=1:size(lp_detected,1)
    plot3([gt(lp_detected(i,1),4);gt(lp_detected(i,2),4)],[gt(lp_detected(i,1),8);gt(lp_detected(i,2),8)],[gt(lp_detected(i,1),12);gt(lp_detected(i,2),12)],'ro')
    hold on
end
axis equal
title('Trajectory')

%% show individual results and intersection for M2DP and SC
% top_count = 300;
if strcmp(type, 'm2dp') || strcmp(type, 'sc')
    for i=1:size(dist_m,1)
        for j=1:size(dist_m,2)
            if(abs(i-j)<mask_width)
                dist_m_t(i,j) = Inf;
                dist_m_i(i,j) = Inf;
            end
        end
    end
    [dist_v_t, dist_t_idx] = min(dist_m_t);
    [~, rank_t] = sort(dist_v_t);
    [dist_v_i, dist_i_idx] = min(dist_m_i);
    [~, rank_i] = sort(dist_v_i);
    [dist_v_ti, dist_ti_idx] = min(dist_m);
    [~, rank_ti] = sort(dist_v_ti);
    lp_t = [rank_t(1:top_count)', dist_t_idx(rank_t(1:top_count))'];
    lp_i = [rank_i(1:top_count)', dist_i_idx(rank_i(1:top_count))'];
    lp_ti = [rank_ti(1:top_count)', dist_ti_idx(rank_ti(1:top_count))'];
    figure('Name', 'Place Recognition')
    subplot(1,2,1)
    plot(lp_t(:,1),lp_t(:,2), 'r.');
    hold on
    plot(lp_i(:,1),lp_i(:,2), 'g.');
    hold on
    plot(lp_ti(:,1),lp_ti(:,2), 'b*');
    legend('Pts Count', 'Ave Color', 'Intersection')
    title('Detected');
    subplot(1,2,2)
    plot(lp_gt(:,1),lp_gt(:,2), 'k*');
    title('Ground Truth');
end

end
