function i = interf_gen(x)
cfg = evalin('base','cfg');

sig = x(:);
N = length(sig);
Ps = mean(abs(sig).^2) + 1e-12;
JS = 10^(cfg.emc.JS_dB/10);
Pj = Ps * JS;
t = (0:N-1).';

switch cfg.emc.type
    case 1
        i = sqrt(Pj/2) * (randn(N,1)+1j*randn(N,1));
    case 2
        A = sqrt(Pj);
        f0 = cfg.emc.toneOffset_Hz;
        i = A * exp(1j*2*pi*f0*t/cfg.phy.Rs);
    case 3
        duty = cfg.emc.pulseDuty;
        per = cfg.emc.pulsePeriodSym;
        onN = max(1, round(duty*per));
        mask = false(N,1);
        pos = 1;
        while pos <= N
            mask(pos:min(pos+onN-1,N)) = true;
            pos = pos + per;
        end
        P_on = Pj / max(duty,1e-6);
        i = zeros(N,1);
        w = sqrt(P_on/2) * (randn(N,1)+1j*randn(N,1));
        i(mask) = w(mask);
    case 4
        Ieq = Pj * 10^(-cfg.emc.ACIR_dB/10);
        i = sqrt(Ieq/2) * (randn(N,1)+1j*randn(N,1));
    case 5
        Psrc = 10^(cfg.emc.cositeSrcP_dBW/10);
        coup = 10^(cfg.emc.cositeCoupling_dB/10);
        Ieq = Psrc * coup;
        i = sqrt(Ieq/2) * (randn(N,1)+1j*randn(N,1));
    otherwise
        i = zeros(N,1);
end
end
