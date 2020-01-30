function diff_m = processBoW(hist1, hist2)
    hist1_i = hist1(1:2:end,:);
    hist1_v = hist1(2:2:end,:);
    hist2_i = hist2(1:2:end,:);
    hist2_v = hist2(2:2:end,:);
    m = size(hist1_i,1);
    n = size(hist2_i,1);
    
    % L1 score of sparse vectors
    diff_m = ones(m,n);
    for i=1:m
        for j=1:n
            diff_m(i,j) = 1-l1_score(hist1_i(i,:), hist1_v(i,:), hist2_i(j,:), hist2_v(j,:));
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