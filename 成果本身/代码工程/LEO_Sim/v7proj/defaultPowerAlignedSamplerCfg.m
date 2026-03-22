function cfg = defaultPowerAlignedSamplerCfg()
    cfg = struct();

    % IQ snapshot length
    cfg.Ns = 2048;

    % STFT (keep consistent with training)
    cfg.Stft = struct('win',256,'overlap',128,'nfft',256,'fs',1);
    cfg.imgSize = [128 128];

    % If true: use PJ_mW (effective, post-AJ) -> align with main SINR
    % If false: use PJraw_mW (pre-AJ) -> show "raw interference" even if protected
    cfg.UsePostAJPower = true;
    % Route-1: continuous info code (in [0,1]) that drives waveform morphology.
    %   InfoCode(1): narrowbandness (0 -> tone-like, 1 -> noise-like)
    %   InfoCode(2): burst/modulation strength (0 -> continuous/noise, 1 -> bursty/modulated)
    cfg.InfoCode = [0.5 0.5];

    % Co-channel interferer model (kept fixed): modulated wideband
    cfg.CCIModel = 'mod';

    % Deterministic per-time-step randomization (so results are repeatable)
    cfg.Seed = 2028;

    % Detection threshold: treat interference as present if P > K*PN
    cfg.DetK = 1.0;

    % Parameter ranges (normalized frequency 0..0.5 for fs=1)
    cfg.tone = struct('f0Range',[0.05 0.45]);

    cfg.pbnj = struct( ...
        'bandStartRange',[0.03 0.30], ...
        'bwRange',[0.05 0.20], ...
        'firOrder',80);

    cfg.mod = struct( ...
        'dfRange',[0.03 0.15], ...
        'dutyRange',[0.15 0.35]);
end
