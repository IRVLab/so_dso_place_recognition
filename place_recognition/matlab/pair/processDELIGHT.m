function dist = processDELIGHT(hist1, hist2, start_idx, end_idx)
    Mut = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;
           6,5,8,7,2,1,4,3,14,13,16,15,10,9,12,11;
           7,8,5,6,3,4,1,2,15,16,13,14,11,12,9,10;
           4,3,2,1,8,7,6,5,12,11,10,9,16,15,14,13;];
       
    hist1 = hist1(16*start_idx+1:size(hist1,1)-16*end_idx, :);
    hist2 = hist2(16*start_idx+1:size(hist2,1)-16*end_idx, :);

    m = size(hist1,1)/16;
    n = size(hist2,1)/16;

    % linearize hist
    hist1_vec = zeros(m, size(hist1,2)*16);
    for i=1:m
        temp = hist1((16*(i-1)+1):(16*i), :);
        temp = temp';
        hist1_vec(i,:) = temp(:);
    end
    
    % permute hist2 and linearize
    hist2_vec_mut = zeros(n, size(hist2,2)*16, 4);
    for k=1:4
        for j=1:n
            temp = hist2(16*(j-1)+Mut(k,:),:);
            temp = temp';
            hist2_vec_mut(j,:,k) = temp(:);
        end
    end

    dist = Inf*ones(m,n);
    % find the closest dist out of 4 configs for each loc in hist1
    B = hist2_vec_mut;
    for i=1:m
        A = repmat(hist1_vec(i, :),n,1,4);
        AB = A+B+0.001;
        A_B = A-B;
        S = (2*A_B.*A_B) ./ AB;
        dist_i = sum(S,2);
        dist_i = [dist_i(:,1,1), dist_i(:,1,2), dist_i(:,1,3), dist_i(:,1,4)];
        dist_i = min(dist_i');
        dist(i,:) = dist_i';
    end
end