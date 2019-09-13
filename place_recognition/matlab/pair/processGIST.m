function res = processGIST(hist1, hist2, start_idx, end_idx)
    hist1 = hist1(start_idx+1:size(hist1,1)-end_idx, :);
    hist2 = hist2(start_idx+1:size(hist2,1)-end_idx, :);

    m = size(hist1,1);
    n = size(hist2,1);
    res = Inf * ones(m,n);
    for i=1:m
        for j=1:n
            res(i,j)=sum((hist1(i,:)-hist2(j,:)).^2);
        end
    end
end