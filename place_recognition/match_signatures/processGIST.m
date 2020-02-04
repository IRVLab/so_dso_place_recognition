function diff_m = processGIST(hist1, hist2)
    m = size(hist1,1);
    n = size(hist2,1);
    diff_m = zeros(m,n);
    for i=1:m
        for j=1:n
            diff_m(i,j)=sum((hist1(i,:)-hist2(j,:)).^2);
        end
    end
end