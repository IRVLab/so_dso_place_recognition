function res = processGIST(hist, start_idx, end_idx, mask_width)
    hist = hist(start_idx+1:size(hist,1)-end_idx, :);
    m = size(hist,1);
    res = Inf * ones(m,m);
    for i=1:m
        for j=1:m
            if(abs(i-j)<mask_width)
                res(i,j)=Inf;
            else
                res(i,j)=sum((hist(i,:)-hist(j,:)).^2);
            end
        end
    end
    res = normalize(res,2);
end