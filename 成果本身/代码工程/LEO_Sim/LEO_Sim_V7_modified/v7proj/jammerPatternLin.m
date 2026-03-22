function patLin = jammerPatternLin(offDeg, mainDeg, sideDeg, mainGain_dB, sideGain_dB, floorGain_dB)
    g = zeros(size(offDeg));
    g(offDeg <= mainDeg) = mainGain_dB;
 
    mid = (offDeg > mainDeg) & (offDeg <= sideDeg);
    if any(mid)
        t = (offDeg(mid) - mainDeg) ./ max(1e-9, (sideDeg-mainDeg));
        g(mid) = mainGain_dB + t.*(sideGain_dB - mainGain_dB);
    end
 
    far = offDeg > sideDeg;
    g(far) = floorGain_dB;
 
    patLin = 10.^(g/10);
end
