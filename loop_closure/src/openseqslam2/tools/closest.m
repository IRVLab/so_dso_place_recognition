function x = closest(val, vector)
    [~, I] = min(abs(vector - val));
    x = vector(I);
end
