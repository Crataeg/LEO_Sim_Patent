function j = scaleToJSR(j0, s, JSR_dB)
    if all(j0==0)
        j = j0; return;
    end
    Ps = mean(abs(s).^2);
    Pj_target = Ps * 10^(JSR_dB/10);
    j = j0 * sqrt(Pj_target / (mean(abs(j0).^2) + 1e-12));
end
