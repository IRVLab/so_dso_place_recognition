clear 
close all

% type = 'M2DP';
type = 'SC';
% type = 'DELIGHT';
% type = 'GIST';
% type = 'BoW';

dates = ["2014-07-14-14-49-50"; "2014-11-28-12-07-13"; "2014-12-12-10-45-15";
         "2015-02-10-11-58-05"; "2015-05-19-14-06-38"; "2015-05-22-11-14-30";
         "2015-08-13-16-02-58"; "2015-10-30-13-52-14"];

run_seq = [5,6; 5,7; 5,8; 5,4; 7,1; 7,8; 7,4; 8,2; 8,4; 4,3];

TRs = zeros(1, size(run_seq, 1));
AUCs = zeros(1, size(run_seq, 1));
for seqi=1:size(run_seq,1)
% for seqi=7
    run1 = dates(run_seq(seqi,1));
    run2 = dates(run_seq(seqi,2));
    [ AUCs(seqi), TRs(seqi) ] = run_test(run1, run2, seqi, type);
    pause(0.5);
end
AUCs 
TRs

function [AUC, top_recall] = run_test(run1, run2, run_count, type) 
start_idx = 0;
end_idx = 0;
loop_diff = 25;

incoming1_id = load(strcat(strcat('../../results/RobotCar/', run1), '/incoming_id_file.txt'));
incoming2_id = load(strcat(strcat('../../results/RobotCar/', run2), '/incoming_id_file.txt'));
gt1_full = load(strcat(strcat('../../results/RobotCar/', run1), '/gps.txt'));
gt2_full = load(strcat(strcat('../../results/RobotCar/', run2), '/gps.txt'));
incoming1_id = incoming1_id(start_idx + 1: size(incoming1_id, 1) - end_idx, :) +1;
incoming2_id = incoming2_id(start_idx + 1: size(incoming2_id, 1) - end_idx, :) +1;
gt1 = gt1_full(incoming1_id, :);
gt2 = gt2_full(incoming2_id, :);

if (~strcmp(type, 'BoW'))
    hist1 = load(strcat(strcat(strcat(strcat('../../results/RobotCar/', run1), '/history'), type), '.txt'));
    hist2 = load(strcat(strcat(strcat(strcat('../../results/RobotCar/', run2), '/history'), type), '.txt'));
end

tic 
% get difference matrix 
switch type 
    case 'M2DP' 
        diff_m = processM2DP(hist1, hist2, start_idx, end_idx);
    case 'SC'
        diff_m = processSC(hist1, hist2, start_idx, end_idx);
    case 'DELIGHT'
        diff_m = processDELIGHT(hist1, hist2, start_idx, end_idx);
    case 'GIST'
        diff_m = processGIST(hist1, hist2, start_idx, end_idx);
    case 'BoW'
        diff_m = load(strcat(strcat('../../results/RobotCar/BoW/t', num2str(run_count)), '/diff_mat.txt'));
        diff_m = diff_m(start_idx+1:size(diff_m,1)-end_idx, start_idx+1:size(diff_m,2)-end_idx);
        diff_m = 1 - diff_m;
end
toc

[AUC, top_recall] = getAUCandPlot(diff_m, gt1, gt2, loop_diff, run_count);
end
