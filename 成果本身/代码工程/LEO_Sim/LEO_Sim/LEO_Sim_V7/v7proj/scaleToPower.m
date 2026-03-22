function y = scaleToPower(y0, Ptarget_mW)
    if Ptarget_mW <= 0 || all(y0==0)
        y = zeros(size(y0)); 
        return;
    end
    P0 = mean(abs(y0).^2);
    y = y0 * sqrt(Ptarget_mW / (P0 + 1e-12));
end
