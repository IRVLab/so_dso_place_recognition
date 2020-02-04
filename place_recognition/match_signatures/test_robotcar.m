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
types = ["delight";];

dates = ["2014-07-14-14-49-50"; "2014-11-28-12-07-13"; "2014-12-12-10-45-15";
         "2015-02-10-11-58-05"; "2015-05-19-14-06-38"; "2015-05-22-11-14-30";
         "2015-08-13-16-02-58"; "2015-10-30-13-52-14"];

run_seq = [5,6; 5,7; 5,8; 5,4; 7,1; 7,8; 7,4; 8,2; 8,4; 4,3];

TRs = zeros(size(types, 2), size(run_seq, 1));
AUCs = zeros(size(types, 2), size(run_seq, 1));

for ti=1:size(types,2)
% for ti=4
    for si=1:size(run_seq,1)
%     for si=1
        [AUCs(ti, si), TRs(ti, si)] = run_robotcar(dates(run_seq(si,1)), dates(run_seq(si,2)), types(ti));
        pause(0.5);
    end
end
AUCs 
TRs

function [AUC, top_recall] = run_robotcar(run1, run2, type) 
mask_width = 0;
loop_diff = 25;

% load data
incoming1_id = load(strcat(strcat('../results/RobotCar/', run1), '/incoming_id_file.txt'))+1;
incoming2_id = load(strcat(strcat('../results/RobotCar/', run2), '/incoming_id_file.txt'))+1;
gt1_full = load(strcat(strcat('../results/RobotCar/', run1), '/gps.txt'));
gt2_full = load(strcat(strcat('../results/RobotCar/', run2), '/gps.txt'));
gt1 = gt1_full(incoming1_id, :);
gt2 = gt2_full(incoming2_id, :);
hist1 = load(strcat(strcat(strcat(strcat('../results/RobotCar/', run1), '/history_'), type), '.txt'));
hist2 = load(strcat(strcat(strcat(strcat('../results/RobotCar/', run2), '/history_'), type), '.txt'));

[AUC, top_recall] = run_test(type, hist1, hist2, gt1, gt2, loop_diff, mask_width);
end
