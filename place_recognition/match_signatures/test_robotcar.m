clear 
close all

dates = ["2014-07-14-14-49-50"; "2014-11-28-12-07-13"; "2014-12-12-10-45-15";
         "2015-02-10-11-58-05"; "2015-05-19-14-06-38"; "2015-05-22-11-14-30";
         "2015-08-13-16-02-58"; "2015-10-30-13-52-14"];
    
types = ["delight","m2dp","sc","bow","gist"];
run_seq = [5,6; 5,7; 5,8; 5,4; 7,1; 7,8; 7,4; 8,2; 8,4; 4,3];

% types = ["sc"];
% run_seq = [5,6; 5,4];

TRs = zeros(size(types, 2), size(run_seq, 1));
AUCs = zeros(size(types, 2), size(run_seq, 1));

for ti=1:size(types,2)
    for si=1:size(run_seq,1)
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

[AUC, top_recall, lp_detected] = run_test(type, hist1, hist2, gt1, gt2, loop_diff, mask_width);

% plot recognized places
figure('Name', 'Trajectory')
plot3(gt1(:,1),gt1(:,2),gt1(:,3), 'b')
hold on
plot3(gt2(:,1),gt2(:,2),100+gt2(:,3), 'g')
hold on
for i=1:size(lp_detected,1)
    plot3([gt1(lp_detected(i,1),1);gt2(lp_detected(i,2),1)],[gt1(lp_detected(i,1),2);gt2(lp_detected(i,2),2)],[gt1(lp_detected(i,1),3);100+gt2(lp_detected(i,2),3)], 'r')
    hold on
end
axis equal
end
