function diff_m_ci = processM2DP(hist1, hist2, start_idx, end_idx)
    hist1 = hist1(4*start_idx+1:size(hist1,1)-4*end_idx, :);
    hist1_c = hist1(:,1:size(hist1,2)/2);
    hist1_i = hist1(:,size(hist1,2)/2+1:end);
    
    hist2 = hist2(4*start_idx+1:size(hist2,1)-4*end_idx, :);
    hist2_c = hist2(:,1:size(hist2,2)/2);
    hist2_i = hist2(:,size(hist2,2)/2+1:end);
    
    diff_m_c = process(hist1_c, hist2_c);
    diff_m_i = process(hist1_i, hist2_i);
    
    % weight diff
    w_c = std2(diff_m_c);
    w_i = std2(diff_m_i);
    diff_m_ci = (normalize(diff_m_c,2)*w_c + normalize(diff_m_i,2)*w_i)/(w_c+w_i);
end

function diff_m = process(hist1, hist2)
    m = size(hist1,1)/4;
    n = size(hist2,1)/4;
    res_full = (1-hist1*hist2')/2;
    diff_m = zeros(m,n);
    for i=1:m
        for j=1:n
            diff_m(i,j) = min(min(res_full(4*i-3:4*i, 4*j-3:4*j)));
        end
    end
end