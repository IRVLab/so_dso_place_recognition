function img = patchNormalise(img, threshold, strength)
    % Taken heavily from implementation in openSeqSLAM
    % TODO credit

    %    ys = 1:sideLength:size(img,1)+1;
    %    xs = 1:sideLength:size(img,2)+1;
    %
    %    for y = 1:length(ys)-1
    %        for x = 1:length(xs)-1
    %            p = img(ys(y):ys(y+1)-1, xs(x):xs(x+1)-1);
    %            pp = p(:);
    %
    %            if (mode ~= 0)
    %                pp = double(pp);
    %                img(ys(y):ys(y+1)-1, xs(x):xs(x+1)-1) = 127 + reshape( ...
    %                    round((pp-mean(pp))/std(pp)), sideLength, sideLength);
    %            else
    %                f = 255.0/double(max(pp) - min(pp));
    %                img(ys(y):ys(y+1)-1, xs(x):xs(x+1)-1) = round(f*(p-min(pp)));
    %            end
    %        end
    %    end

    img = localcontrast(img, threshold, strength);
end
