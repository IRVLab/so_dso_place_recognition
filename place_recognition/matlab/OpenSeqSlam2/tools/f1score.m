function f1 = f1score(p, r)
    f1 = 2 .* p .* r ./ (p + r);
end
