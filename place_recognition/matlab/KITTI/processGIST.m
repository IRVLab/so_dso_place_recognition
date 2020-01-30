function res = processGIST(hist)
    m = size(hist,1);
    res = Inf * ones(m,m);
    for i=1:m
        for j=1:m
            res(i,j)=sum((hist(i,:)-hist(j,:)).^2);
        end
    end
end