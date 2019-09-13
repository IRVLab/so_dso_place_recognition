clear
close all

% type = 'M2DP';
type = 'SC';
% type = 'DELIGHT';
% type = 'GIST';
% type = 'FBoW';

start_idx = 20;
end_idx = 20;
mask_width = 50;
loop_dist = 10;
incoming_id = load('../../results/KITTI/06/incoming_id_file.txt');
gt_full = load('../../results/KITTI/06/gt.txt');

incoming_id = incoming_id(start_idx+1:size(incoming_id,1)-end_idx, :)+1;
gt = gt_full(incoming_id,:);
gt_xy = alignZ(gt(:,[4,8,12]));

if ~strcmp(type, 'FBoW')
    hist = load(strcat(strcat(strcat('../../results/KITTI/06/history'), type), '.txt'));
end

% get distance matrix
switch type
    case 'M2DP'
        [dist_m, dist_m_t, dist_m_i] = processM2DP(hist, start_idx, end_idx, mask_width);
    case 'SC'
        [dist_m, dist_m_t, dist_m_i] = processSC(hist, start_idx, end_idx, mask_width);
    case 'DELIGHT'
        dist_m = processDELIGHT(hist, start_idx, end_idx, mask_width);
    case 'GIST'
        dist_m = processGIST(hist, start_idx, end_idx, mask_width);
    case 'FBoW'
        dist_m = load('../../results/KITTI/06/FBoW/dist_mat.txt');
        dist_m = dist_m(start_idx+1:size(dist_m,1)-end_idx, start_idx+1:size(dist_m,2)-end_idx);
        dist_m = 1-dist_m;
        for i=1:size(dist_m,1)
            for j=1:size(dist_m,2)
                if(abs(i-j)<mask_width)
                    dist_m(i,j) = Inf;
                end
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
top_recall
AUC = trapz(recall,precision)
title(strcat(strcat(strcat('Precision-Recall(', num2str(top_recall)), '), AUC='), num2str(AUC)));

%% show trajectory
subplot(1,2,2)
plot(gt_xy(:,1), gt_xy(:,2), '.-')
hold on
plot(gt_xy(lp_gt(:,1),1), gt_xy(lp_gt(:,1),2), 'go')
hold on
plot(gt_xy(lp_gt(:,2),1), gt_xy(lp_gt(:,2),2), 'go')
hold on
lp_detected = [dist_rank(1:top_count)', dist_idx(dist_rank(1:top_count))'];
for i=1:size(lp_detected,1)
    plot([gt_xy(lp_detected(i,1),1);gt_xy(lp_detected(i,2),1)],[gt_xy(lp_detected(i,1),2);gt_xy(lp_detected(i,2),2)],'r-')
    hold on
end
title('Trajectory')

%% show individual results and intersection for M2DP and SC
% top_count = 300;
if strcmp(type, 'M2DP') || strcmp(type, 'SC')
    [dist_v_t, dist_t_idx] = min(dist_m_t);
    [~, rank_t] = sort(dist_v_t);
    [dist_v_i, dist_i_idx] = min(dist_m_i);
    [~, rank_i] = sort(dist_v_i);
    [dist_v_ti, dist_ti_idx] = min(dist_m);
    [~, rank_ti] = sort(dist_v_ti);
    % top_count = 100;
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

%% project 3D to 2D
function X = alignZ(XYZ)
    xyz0=mean(XYZ);
    A=bsxfun(@minus,XYZ,xyz0); %center the data
    [~,~,V]=svd(A,0);
    XYZ = A*V;
    angle = XYZ(floor(size(XYZ,1)/4),1:2) - XYZ(1,1:2);
    angle = atan2(angle(2), angle(1));
    R = [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1];
    XYZ = XYZ * R';
    X = XYZ - XYZ(1,:);
end
