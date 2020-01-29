function diff_m_ci = processSC(hist1, hist2)
    hist1_c = hist1(:,1:size(hist1,2)/2);
    hist1_i = hist1(:,size(hist1,2)/2+1:end);
    
    hist2_c = hist2(:,1:size(hist2,2)/2);
    hist2_i = hist2(:,size(hist2,2)/2+1:end);
    
    diff_m_c = process(hist1_c, hist2_c);
    diff_m_i = process(hist1_i, hist2_i);
    
    % weight diff  
    w_c = std2(diff_m_c);
    w_i = std2(diff_m_i);
    diff_m_ci = (normalize(diff_m_c,2)*w_c + normalize(diff_m_i,2)*w_i)/(w_c+w_i);
end

function res = process(hist1, hist2)
    m = size(hist1,1);
    n = size(hist2,1);
    for i=1:m
        hist1(i,:) = hist1(i,:) / norm(hist1(i,:));
    end
    for i=1:n
        hist2(i,:) = hist2(i,:) / norm(hist2(i,:));
    end
    res = inf*ones(m,n);
    for i=1:m
        % permute forward and backward
        sig_i = zeros(120, 1200);
        for k=1:60
            sig_i(2*k-1, :) = permute_sc(hist1, i, k, false);
            sig_i(2*k, :)   = permute_sc(hist1, i, k, true);
        end
        
        diff_m = (1-sig_i * hist2')/2;
        diff_v = min(diff_m);
        res(i,:) = diff_v;
    end
end

% permute
function sig = permute_sc(hist, i, idx, reverse)
    h_img = reshape(hist(i,:), [20,60]);
    if(~reverse)
        sig = h_img(:,[idx:1:60,1:1:(idx-1)]);
    else
        sig = h_img(:,[idx:-1:1,60:-1:(idx+1)]);
    end
    sig = sig(:)';
end