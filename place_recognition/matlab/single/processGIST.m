function res = processGIST(hist, start_idx, end_idx)
    hist = hist(start_idx+1:size(hist,1)-end_idx, :);
    m = size(hist,1);
    res = Inf * ones(m,m);
    for i=1:m
        for j=1:m
            res(i,j)=sum((hist(i,:)-hist(j,:)).^2);
        end
    end
end