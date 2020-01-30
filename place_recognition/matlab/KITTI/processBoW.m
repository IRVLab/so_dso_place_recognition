function diff_m = processBoW(hist)
    hist_i = hist(1:2:end,:);
    hist_v = hist(2:2:end,:);
    m = size(hist_i,1);
    
    % L1 score of sparse vectors
    diff_m = ones(m,m);
    for i=1:m
        for j=1:m
            diff_m(i,j) = 1-l1_score(hist_i(i,:), hist_v(i,:), hist_i(j,:), hist_v(j,:));
        end
    end
end

function score = l1_score(i1, v1, i2, v2) 
    ii1 = 1;
    ii2 = 1;
    score = 0.0;
    
    while(ii1<length(i1) && i1(ii1)>-1 && ii2<length(i2) && i2(ii2)>-1)
        if(i1(ii1) == i2(ii2))
            score = score + abs(v1(ii1)-v2(ii2)) - abs(v1(ii1)) - abs(v2(ii2));
            ii1 = ii1+1;
            ii2 = ii2+1;
        else
            if(i1(ii1) < i2(ii2))
                ii1 = ii1+1;
            else
                ii2 = ii2+1;
            end
        end
    end
    
    score = -score / 2.0;
end