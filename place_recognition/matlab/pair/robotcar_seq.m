clear
close all

type = 'SC';
type = 'SeqSC';
type = 'SeqSLAM1';
type = 'SeqSLAM2';

dates = ["2014-07-14-14-49-50";"2014-11-28-12-07-13";"2014-12-12-10-45-15";"2015-02-10-11-58-05";
        "2015-05-19-14-06-38";"2015-05-22-11-14-30";"2015-08-13-16-02-58";"2015-10-30-13-52-14"];

run_seq = [5,6; 5,7; 5,8; 5,4; 7,1; 7,8; 7,4; 8,2; 8,4; 4,3];

TRs = zeros(1, size(run_seq,1));
AUCs = zeros(1, size(run_seq,1));
for seqi=1:size(run_seq,1)
% for seqi=1
    run1 = dates(run_seq(seqi,1));
    run2 = dates(run_seq(seqi,2));
    [AUCs(seqi), TRs(seqi)] = run_test(run1, run2, seqi, type);
    pause(0.5);
end
AUCs
TRs

function [AUC, top_recall] = run_test(run1, run2, run_count, type)
loop_dist = 25;

incoming1_id = load(strcat(strcat('../../results/RobotCar/',run1),'/incoming_id_file.txt'));
incoming2_id = load(strcat(strcat('../../results/RobotCar/',run2),'/incoming_id_file.txt'));
gt1_full = load(strcat(strcat('../../results/RobotCar/',run1),'/gps.txt'));
gt2_full = load(strcat(strcat('../../results/RobotCar/',run2),'/gps.txt'));
incoming1_id = incoming1_id+1;
incoming2_id = incoming2_id+1;
gt1 = gt1_full(incoming1_id,:);
gt2 = gt2_full(incoming2_id,:);


% get difference matrix
switch type
    case 'SC'
        hist1 = load(strcat(strcat(strcat('../../results/RobotCar/',run1),'/history'), 'SC.txt'));
        hist2 = load(strcat(strcat(strcat('../../results/RobotCar/',run2),'/history'), 'SC.txt'));
        hist1 = hist1(1:10:end, :);
        hist2 = hist2(1:10:end, :);
        diff_m = processSC(hist1, hist2);
    case 'SeqSLAM1'
        load(strcat(strcat('../../results/RobotCar/SeqSLAM/t', num2str(run_count)), '/diff_matrix.mat'));
        diff_m = diffMatrix.base;
    case 'SeqSLAM2'
        load(strcat(strcat('../../results/RobotCar/SeqSLAM/t', num2str(run_count)), '/matching.mat'));
        diff_m = matching.all.min_scores;
    case 'SeqSC'
        load(strcat(strcat('../../results/RobotCar/SeqSLAM/t', num2str(run_count)), '/matchingSC.mat'));
        diff_m = matchingSC;
end

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
    if(min_diff<loop_dist*loop_dist)
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
    if(diff<loop_dist*loop_dist)
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
