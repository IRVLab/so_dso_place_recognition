function [AUC, top_recall, lp_detected] = run_test(type, hist1, hist2, gt1, gt2, loop_diff, mask_width)
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
        [diff_m_p, diff_m_i] = processM2DP(hist1, hist2);
    case 'sc'
        [diff_m_p, diff_m_i] = processSC(hist1, hist2);
    case 'delight'
        diff_m = processDELIGHT(hist1, hist2);
    case 'gist'
        diff_m = processGIST(hist1, hist2);
    case 'bow'
        diff_m = processBoW(hist1, hist2);
end
if strcmp(type, 'm2dp') || strcmp(type, 'sc')
    p_weight = 2;
    diff_m = p_weight*normalize(diff_m_p,2) + normalize(diff_m_i,2);
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
lp_detected = [diff_rank(1:top_count)', diff_idx(diff_rank(1:top_count))'];

% plot results
figure
plot(recall, precision, 'r.')
axis([0 1 0 1])
title(strcat(strcat(strcat('Precision-Recall(', num2str(top_recall)), '), AUC='), num2str(AUC)));
grid on

end
