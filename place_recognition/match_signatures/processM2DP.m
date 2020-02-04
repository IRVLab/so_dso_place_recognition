function [diff_m_pi, diff_m_p, diff_m_i] = processM2DP(hist1, hist2)
    hist1_p = hist1(:,1:size(hist1,2)/2);
    hist1_i = hist1(:,size(hist1,2)/2+1:end);
    
    hist2_p = hist2(:,1:size(hist2,2)/2);
    hist2_i = hist2(:,size(hist2,2)/2+1:end);
    
    diff_m_p = process(hist1_p, hist2_p);
    diff_m_i = process(hist1_i, hist2_i);
    
    % weight diff
    w_p = std2(diff_m_p);
    w_i = std2(diff_m_i);
    diff_m_pi = (normalize(diff_m_p,2)*w_p + normalize(diff_m_i,2)*w_i)/(w_p+w_i);
end

function diff_m = process(hist1, hist2)
    m = size(hist1,1)/4;
    n = size(hist2,1)/4;
    diff_full = (1-hist1*hist2')/2;
    diff_m = zeros(m,n);
    for i=1:m
        for j=1:n
            diff_m(i,j) = min(min(diff_full(4*i-3:4*i, 4*j-3:4*j)));
        end
    end
end