function s = fmtNum(x,n)
    if isempty(x) || isnan(x), s="-"; return; end
    s = num2str(x, ['%.' num2str(n) 'f']);
end
