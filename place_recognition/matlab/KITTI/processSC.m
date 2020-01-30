function [diff_m_pi, diff_m_p, diff_m_i] = processSC(hist)
    hist_t = hist(:,1:size(hist,2)/2);
    hist_i = hist(:,size(hist,2)/2+1:end);

    diff_m_p = process(hist_t);
    diff_m_i = process(hist_i);

    % weight dist
    w_p = std2(diff_m_p);
    w_i = std2(diff_m_i);
    diff_m_pi = (normalize(diff_m_p,2)*w_p + normalize(diff_m_i,2)*w_i)/(w_p+w_i);
end

function dist = process(hist)
    m = size(hist,1);
    for i=1:m
        hist(i,:) = hist(i,:) / norm(hist(i,:));
    end
    dist = inf*ones(m,m);
    for i=1:m
        % SC is of size 60x20
        sig_i = zeros(120, 1200);
        for k=1:60
            sig_i(2*k-1, :) = undist_sc(hist, i, k, false);
            sig_i(2*k, :)   = undist_sc(hist, i, k, true);
        end
        dist_m = (1-sig_i * hist')/2;
        dist_v = min(dist_m);
        dist(i,:) = dist_v;
    end
end

function sig = undist_sc(hist, i, idx, reverse)
    h_img = reshape(hist(i,:), [20,60]); 
    if(~reverse)
        sig = h_img(:,[idx:1:60,1:1:(idx-1)]);	
    else
        sig = h_img(:,[idx:-1:1,60:-1:(idx+1)]);
    end
    sig = sig(:)';
end