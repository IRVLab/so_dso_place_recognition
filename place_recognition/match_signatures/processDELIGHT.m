function dist = processDELIGHT(hist1, hist2)
    Mut = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;
           6,5,8,7,2,1,4,3,14,13,16,15,10,9,12,11;
           7,8,5,6,3,4,1,2,15,16,13,14,11,12,9,10;
           4,3,2,1,8,7,6,5,12,11,10,9,16,15,14,13;];

    m = size(hist1,1)/16;
    n = size(hist2,1)/16;

    dist = zeros(m,n);
    for i=1:m
        A = hist1((16*(i-1)+1):(16*i), :);
        for j=1:n
            B = hist2((16*(j-1)+1):(16*j), :);
            min_dist = Inf;
            for k=1:4
                Ak=A(:);
                Bk = B(Mut(k,:),:);
                Bk = Bk(:);
                AB_s = Ak+Bk;
                ts = 0;
                tc = 0;
                for l=1:size(Ak,1)
                    if(AB_s(l)>0)
                        s = 2*(Ak(l)-Bk(l))^2/AB_s(l);
                        ts = ts + s;
                        tc = tc + 1;
                    end
                end
                ts = ts / tc;
                if(min_dist > ts)
                    min_dist = ts;
                end
            end
            dist(i,j) = min_dist;
        end
    end
end