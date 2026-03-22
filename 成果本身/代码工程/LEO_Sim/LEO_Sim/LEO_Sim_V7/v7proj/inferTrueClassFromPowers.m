function cls = inferTrueClassFromPowers(Pi_mW, Pj_mW, Pn_mW, cfg)
    % Pseudo "truth" for display only.
    % Route-1: Do NOT rely on a hard-coded jammer type. Instead, infer a
    % dominant morphology label from the continuous info code (if jammer dominates).
    if max(Pi_mW, Pj_mW) <= cfg.DetK*Pn_mW
        cls = 'none';
        return;
    end

    if Pj_mW >= Pi_mW
        w = jammerMixtureWeightsFromInfoCode(cfg.InfoCode);
        [~, ix] = max(w);
        switch ix
            case 1, cls = 'tone';
            case 2, cls = 'pbnj';
            otherwise, cls = 'mod';
        end
    else
        % Co-channel modeled as modulated wideband
        cls = 'mod';
    end
end
