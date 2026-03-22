function PL = fspl_dB_vec(range_m, freq_MHz)
    d_km = max(range_m, 1)/1000;
    PL = 32.45 + 20*log10(d_km) + 20*log10(freq_MHz);
end
