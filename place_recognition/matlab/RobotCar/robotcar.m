clear 
close all

% AUCs =
%DELIGHT  0.7655    0.5306    0.3004    0.0354    0.6717    0.4313    0.0244    0.3019    0.0101    0.0229
%M2DP     0.8981    0.8375    0.5002    0.3001    0.8679    0.5186    0.2526    0.5336    0.3534    0.5417
%SC       0.9533    0.9425    0.7717    0.6909    0.9274    0.7770    0.5841    0.6322    0.4599    0.8120
%BoW      0.5577    0.3417    0.2076    0.2998    0.3049    0.4179    0.3708    0.0022    0.2928    0.0012
%GIST     0.9319    0.9177    0.6787    0.7781    0.9141    0.6942    0.7377    0.0026    0.6056    0.0004
% 
% TRs =
%DELIGHT  0.0232    0.0280    0.0123         0    0.0296    0.0167    0.0051    0.0077         0    0.0073
%M2DP     0.2736    0.2494    0.0008    0.0302    0.0669    0.0089    0.0363    0.0473    0.0377    0.0115
%SC       0.7444    0.5539    0.3973    0.2828    0.7479    0.4161    0.2349    0.3159    0.2370    0.5902
%BoW      0.0324    0.0206    0.0231    0.0305    0.0049    0.0342    0.0995         0    0.0432         0
%GIST     0.7940    0.3766    0.2415    0.1758    0.5029    0.2415    0.1558         0    0.1089         0
    
    
types = ["delight","m2dp","sc","bow","gist";];

dates = ["2014-07-14-14-49-50"; "2014-11-28-12-07-13"; "2014-12-12-10-45-15";
         "2015-02-10-11-58-05"; "2015-05-19-14-06-38"; "2015-05-22-11-14-30";
         "2015-08-13-16-02-58"; "2015-10-30-13-52-14"];

run_seq = [5,6; 5,7; 5,8; 5,4; 7,1; 7,8; 7,4; 8,2; 8,4; 4,3];

TRs = zeros(size(types, 2), size(run_seq, 1));
AUCs = zeros(size(types, 2), size(run_seq, 1));

for ti=1:size(types,2)
% for ti=2
    for si=1:size(run_seq,1)
    % for seqi=7
        run1 = dates(run_seq(si,1));
        run2 = dates(run_seq(si,2));
        [AUCs(ti, si), TRs(ti, si)] = run_test(run1, run2, si, types(ti));
        pause(0.5);
    end
end
AUCs 
TRs

function [AUC, top_recall] = run_test(run1, run2, run_count, type) 
loop_dist = 25;

incoming1_id = load(strcat(strcat('../../results/RobotCar/', run1), '/incoming_id_file.txt'));
incoming2_id = load(strcat(strcat('../../results/RobotCar/', run2), '/incoming_id_file.txt'));
gt1_full = load(strcat(strcat('../../results/RobotCar/', run1), '/gps.txt'));
gt2_full = load(strcat(strcat('../../results/RobotCar/', run2), '/gps.txt'));
incoming1_id = incoming1_id +1;
incoming2_id = incoming2_id +1;
gt1 = gt1_full(incoming1_id, :);
gt2 = gt2_full(incoming2_id, :);

hist1 = load(strcat(strcat(strcat(strcat('../../results/RobotCar/', run1), '/history_'), type), '.txt'));
hist2 = load(strcat(strcat(strcat(strcat('../../results/RobotCar/', run2), '/history_'), type), '.txt'));

% tic 
% get difference matrix 
switch type 
    case 'm2dp' 
        diff_m = processM2DP(hist1, hist2);
    case 'sc'
        diff_m = processSC(hist1, hist2);
    case 'delight'
        diff_m = processDELIGHT(hist1, hist2);
    case 'gist'
        diff_m = processGIST(hist1, hist2);
    case 'bow'
        diff_m = processBoW(hist1, hist2);
end
% toc

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
