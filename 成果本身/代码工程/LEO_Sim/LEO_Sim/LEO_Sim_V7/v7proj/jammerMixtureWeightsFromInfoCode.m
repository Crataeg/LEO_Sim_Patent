function w = jammerMixtureWeightsFromInfoCode(infoCode)
    % Map InfoCode (continuous in [0,1]) -> mixture weights for {tone, pbnj, mod}.
    % This is ONLY used to synthesize an IQ snapshot consistent with the "info code",
    % and to provide a pseudo-label for display.
    if nargin < 1 || isempty(infoCode)
        infoCode = [0.5 0.5];
    end

    a = min(max(double(infoCode(1)), 0), 1); % narrowbandness
    b = 0.5;                                 % burst/mod strength
    if numel(infoCode) >= 2
        b = min(max(double(infoCode(2)), 0), 1);
    end

    wMod  = b;
    wPbnj = a*(1-b);
    wTone = (1-a)*(1-b);

    w = [wTone wPbnj wMod];
    s = sum(w);
    if s < 1e-12
        w = [1 0 0];
    else
        w = w / s;
    end
end
