function j = genJammerSimple(cfg, s)
    Ns = length(s);
    switch cfg.type
        case 'none'
            j = zeros(Ns,1);
        case 'tone'
            n = (0:Ns-1).';
            j0 = exp(1j*2*pi*cfg.f0*n);
            j = scaleToJSR(j0, s, cfg.JSR_dB);
        case 'pbnj'
            u = randn(Ns,1) + 1j*randn(Ns,1);
            hBP = fir1(80, cfg.band);
            j0 = filter(hBP,1,u);
            j = scaleToJSR(j0, s, cfg.JSR_dB);
        case 'mod'
            n = (0:Ns-1).';
            xI = qpskRand(Ns);
            mask = zeros(Ns,1);
            burstLen = max(32, round(cfg.duty*Ns));
            startIdx = randi([1, Ns-burstLen+1]);
            mask(startIdx:startIdx+burstLen-1)=1;
            j0 = (xI .* exp(1j*2*pi*cfg.df*n)) .* mask;
            j = scaleToJSR(j0, s, cfg.JSR_dB);
        otherwise
            j = zeros(Ns,1);
    end
end
