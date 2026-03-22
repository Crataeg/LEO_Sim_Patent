function [r, meta] = sampleIQFromPowers(Ps_mW, Pi_mW, Pj_mW, Pn_mW, cfg, k)
    % Map (PS, PI, PJ, PN) -> complex-baseband IQ snapshot.
    %
    % Route-1 change:
    %   - No cfg.JammerType / cfg.CCIType.
    %   - Jammer morphology is driven by cfg.InfoCode (continuous in [0,1]).
    %
    % Targets:
    %  - mean(|s|^2) ~= Ps_mW
    %  - mean(|i|^2) ~= Pi_mW
    %  - mean(|j|^2) ~= Pj_mW
    %  - mean(|n|^2) ~= Pn_mW

    Ns = cfg.Ns;
    n = (0:Ns-1).';

    meta = struct();

    % deterministic per time-step (repeatable snapshots)
    st = rng;
    rng(cfg.Seed + k);

    % Desired signal (QPSK)
    s0 = qpskRand(Ns);
    s  = scaleToPower(s0, Ps_mW);

    % Co-channel interference (kept fixed): wideband modulated interferer
    iSig = zeros(Ns,1);
    if Pi_mW > 0
        xI = qpskRand(Ns);
        df = cfg.mod.dfRange(1) + diff(cfg.mod.dfRange)*rand;
        if rand > 0.5, df = -df; end
        i0 = xI .* exp(1j*2*pi*df*n);
        iSig = scaleToPower(i0, Pi_mW);
    end

    % Jammer: continuous-mixture morphology driven by InfoCode
    jSig = zeros(Ns,1);
    if Pj_mW > 0
        infoCode = cfg.InfoCode;
        if isempty(infoCode), infoCode = [0.5 0.5]; end

        w = jammerMixtureWeightsFromInfoCode(infoCode);  % [tone, pbnj, mod]
        wTone = w(1); wPbnj = w(2); wMod = w(3);

        % Tone component
        jTone = zeros(Ns,1);
        if wTone > 0
            f0 = cfg.tone.f0Range(1) + diff(cfg.tone.f0Range)*rand;
            if rand > 0.5, f0 = -f0; end
            jTone = exp(1j*2*pi*f0*n);
        end

        % PBNJ component (band-pass noise)
        jPbnj = zeros(Ns,1);
        if wPbnj > 0
            u  = randn(Ns,1) + 1j*randn(Ns,1);
            f1 = cfg.pbnj.bandStartRange(1) + diff(cfg.pbnj.bandStartRange)*rand;

            % bandwidth influenced by infoCode(1)
            bw0 = cfg.pbnj.bwRange(1) + diff(cfg.pbnj.bwRange)*rand;
            a = min(max(infoCode(1),0),1);
            bw = bw0 * (0.35 + 1.30*a);

            f2 = min(0.49, f1 + bw);
            h  = fir1(cfg.pbnj.firOrder, [f1 f2]);
            jPbnj = filter(h, 1, u);
        end

        % Mod/bursty component
        jMod = zeros(Ns,1);
        if wMod > 0
            xJ = qpskRand(Ns);
            df = cfg.mod.dfRange(1) + diff(cfg.mod.dfRange)*rand;
            if rand > 0.5, df = -df; end

            beta = 0.5;
            if numel(infoCode) >= 2, beta = infoCode(2); end
            beta = min(max(beta,0),1);

            dutyLo = cfg.mod.dutyRange(1);
            dutyHi = cfg.mod.dutyRange(2);
            duty   = dutyLo + (dutyHi-dutyLo) * (0.25 + 0.75*beta);
            duty   = min(max(duty, dutyLo), dutyHi);

            mask = zeros(Ns,1);
            burstLen = max(32, round(duty*Ns));
            startIdx = randi([1, Ns-burstLen+1]);
            mask(startIdx:startIdx+burstLen-1) = 1;

            jMod = (xJ .* exp(1j*2*pi*df*n)) .* mask;
        end

        % Combine components (keep total power via final scaleToPower)
        j0 = sqrt(wTone)*jTone + sqrt(wPbnj)*jPbnj + sqrt(wMod)*jMod;
        jSig = scaleToPower(j0, Pj_mW);
    end

    % AWGN
    w = sqrt(Pn_mW/2) * (randn(Ns,1) + 1j*randn(Ns,1));

    r = s + iSig + jSig + w;

    rng(st);

    % meta
    meta.Ps_meas = mean(abs(s).^2);
    meta.Pi_meas = mean(abs(iSig).^2);
    meta.Pj_meas = mean(abs(jSig).^2);
    meta.Pn_meas = mean(abs(w).^2);
end
