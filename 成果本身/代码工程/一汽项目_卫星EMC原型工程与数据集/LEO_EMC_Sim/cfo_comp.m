function y = cfo_comp(r)
cfg = evalin('base','cfg');

rx = r(:);
N = length(rx);
Rs = cfg.phy.Rs;
t = (0:N-1).';

if cfg.rx.cfoMethod == 1
    cfo_hat = cfg.leo.fD_Hz;
else
    L = cfg.phy.preambleHalfLen;
    if N < 2*L
        cfo_hat = 0;
    else
        r1 = rx(1:L);
        r2 = rx(L+1:2*L);
        P = sum(conj(r1).*r2);
        cfo_hat = angle(P) * Rs / (2*pi*L);
    end
end

y = rx .* exp(-1j*2*pi*cfo_hat*t/Rs);
end
