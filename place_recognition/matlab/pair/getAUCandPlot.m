function [AUC, top_recall] = getAUCandPlot(diff_m, gt1, gt2, loop_diff, run_count)
% get closest difference for each index
[diff_v, diff_idx] = min(diff_m');
[~, diff_rank] = sort(diff_v);

% get ground truth 
lp_gt = [];
for i=1:size(gt1,1)
    min_diff = inf;
    min_j = -1;
    for j=1:size(gt2,1)
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

% calculate true position and false position
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
figure('Name', strcat('Run ', num2str(run_count)))
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
title('Trajectory')
axis equal

end