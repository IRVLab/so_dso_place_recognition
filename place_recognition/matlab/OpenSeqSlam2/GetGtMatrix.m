function GetGtMatrix(run1, run2, gt_file)
start_idx = 0;
end_idx = 0;
loop_dist = 25;

incoming1_id = load(strcat(strcat('../../results/RobotCar/',run1),'/incoming_id_file.txt'));
incoming2_id = load(strcat(strcat('../../results/RobotCar/',run2),'/incoming_id_file.txt'));
gt1_full = load(strcat(strcat('../../results/RobotCar/',run1),'/gps.txt'));
gt2_full = load(strcat(strcat('../../results/RobotCar/',run2),'/gps.txt'));
incoming1_id = incoming1_id(start_idx+1:10:size(incoming1_id,1)-end_idx, :)+1;
incoming2_id = incoming2_id(start_idx+1:10:size(incoming2_id,1)-end_idx, :)+1;
gt1 = gt1_full(incoming1_id,:);
gt2 = gt2_full(incoming2_id,:);

x = zeros(size(gt1,1), size(gt2,1));
for i=1:size(gt1,1)
    min_dist = inf;
    min_j = -1;
    for j=1:size(gt2,1)
        dist = gt1(i,:) - gt2(j,:);
        dist = dist*dist';
        if(min_dist > dist)
            min_dist = dist;
            min_j = j;
        end
    end
    if(min_dist<loop_dist*loop_dist)
        x(i,min_j) = 1;
    end
end
x = x';
save(gt_file,'x');
end