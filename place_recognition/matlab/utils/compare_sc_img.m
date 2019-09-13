function compare_sc_img(hist, i1, i2)
    dist_m = zeros(1,4);
    for i=1:4
        dist = hist(i1, :) - rot_img(hist(i2, :), i);
        dist_m(1,i) = dist*dist';
    end

    figure
    [~, idx] = min(dist_m)

    img1 = get_img(hist(i1,:));
    img2 = get_img(rot_img(hist(i2, :), idx));

    subplot(2,1,1);
    imshow(img1);
    colormap(gca,jet);
    subplot(2,1,2);
    imshow(img2);
    colormap(gca,jet);

    sum1 = sum(img1);
    sum2 = sum(img2);
end

function res = rot_img(sig, i)
    seq = [[1: 1:30,31: 1:60];
           [31: 1:60,1: 1:30];
           [60:-1:31,30:-1:1];
           [30:-1:1,60:-1:31];];

    img = reshape(sig, [20,60]);
    img = img(:, seq(i,:));
    res = img(:)';
end

function img = get_img(sig)
    img = reshape(sig, [20,60]);
    rg = max(max(img(img~=0))) - min(min(img(img~=0)));
    if(rg~=0)
        img(img~=0) = 0.1 + 0.9*(img(img~=0) - min(min(img(img~=0)))) / rg;
    end
end
