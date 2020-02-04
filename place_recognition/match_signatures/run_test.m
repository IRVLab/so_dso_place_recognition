function [AUC, top_recall] = run_test(type, hist1, hist2, gt1, gt2, loop_diff, mask_width)
% get ground truth 
lp_gt = [];
for i=1:size(gt1,1)
    min_diff = inf;
    min_j = -1;
    for j=1:size(gt2,1)
        if(abs(i-j)<mask_width)
            continue;
        end
        diff = gt1(i,:) - gt2(j,:);
        diff = diff*diff';
        if(min_diff > diff)
            min_diff = diff;
            min_j = j;
        end
    end
    if(min_diff<loop_diff*loop_diff)
        lp_gt = [lp_gt; i, min_j];
    end
end
total_lp = length(lp_gt);

%% get difference matrix 
tic 
switch type 
    case 'm2dp' 
        [diff_m, diff_m_p, diff_m_i] = processM2DP(hist1, hist2);
    case 'sc'
        [diff_m, diff_m_p, diff_m_i] = processSC(hist1, hist2);
    case 'delight'
        diff_m = processDELIGHT(hist1, hist2);
    case 'gist'
        diff_m = processGIST(hist1, hist2);
    case 'bow'
        diff_m = processBoW(hist1, hist2);
end
tm = toc;
type
tm = 1000 * tm / size(diff_m,1)

% mask out place pairs that is too close
for i=1:size(diff_m,1)
    for j=1:size(diff_m,2)
        if(abs(i-j)<mask_width)
            diff_m(i,j) = Inf;
        end
    end
end

%% get precision recall
% get closest difference for each index
[diff_v, diff_idx] = min(diff_m');
[~, diff_rank] = sort(diff_v);

tp = 0;
fp = 0;
precision = zeros(size(gt1,1),1);
recall = zeros(size(gt1,1),1);
top_recall = 0;
top_count = 0;
for i=1:size(gt1,1)
    a = diff_rank(i);
    b = diff_idx(diff_rank(i));
    diff = gt1(a,:) - gt2(b,:);
    diff = diff*diff';
    if(diff<loop_diff*loop_diff)
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
AUC = trapz(recall,precision);

% plot results
figure
subplot(1,2,1)
plot(recall, precision, 'r.')
axis([0 1 0 1])
title(strcat(strcat(strcat('Precision-Recall(', num2str(top_recall)), '), AUC='), num2str(AUC)));
grid on

subplot(1,2,2)
plot3(gt1(:,1),gt1(:,2),gt1(:,3), 'g')
hold on
plot3(gt2(:,1),gt2(:,2),100+gt2(:,3), 'b')
for i=1:size(lp_gt,1)
    plot3(gt1(lp_gt(i,1),1),gt1(lp_gt(i,1),2),gt1(lp_gt(i,1),3), 'y.')
    hold on
end
lp_detected = [diff_rank(1:top_count)', diff_idx(diff_rank(1:top_count))'];
for i=1:size(lp_detected,1)
    plot3([gt1(lp_detected(i,1),1);gt2(lp_detected(i,2),1)],[gt1(lp_detected(i,1),2);gt2(lp_detected(i,2),2)],[gt1(lp_detected(i,1),3);100+gt2(lp_detected(i,2),3)], 'r')
    hold on
end
axis equal
title('Trajectory')

%% show individual results and intersection for M2DP and SC
% top_count = 300;
if strcmp(type, 'm2dp') || strcmp(type, 'sc')
    for i=1:size(diff_m,1)
        for j=1:size(diff_m,2)
            if(abs(i-j)<mask_width)
                diff_m_p(i,j) = Inf;
                diff_m_i(i,j) = Inf;
            end
        end
    end
    [diff_v_p, diff_p_idx] = min(diff_m_p);
    [~, rank_p] = sort(diff_v_p);
    [diff_v_i, diff_i_idx] = min(diff_m_i);
    [~, rank_i] = sort(diff_v_i);
    [diff_v_pi, diff_pi_idx] = min(diff_m);
    [~, rank_pi] = sort(diff_v_pi);
    lp_p = [rank_p(1:top_count)', diff_p_idx(rank_p(1:top_count))'];
    lp_i = [rank_i(1:top_count)', diff_i_idx(rank_i(1:top_count))'];
    lp_pi = [rank_pi(1:top_count)', diff_pi_idx(rank_pi(1:top_count))'];
    figure('Name', 'Place Recognition')
    subplot(1,2,1)
    plot(lp_p(:,1),lp_p(:,2), 'r.');
    hold on
    plot(lp_i(:,1),lp_i(:,2), 'g.');
    hold on
    plot(lp_pi(:,1),lp_pi(:,2), 'b*');
    legend('Pts Count', 'Ave Color', 'Intersection')
    title('Detected');
    subplot(1,2,2)
    plot(lp_gt(:,1),lp_gt(:,2), 'k*');
    title('Ground Truth');
end

end