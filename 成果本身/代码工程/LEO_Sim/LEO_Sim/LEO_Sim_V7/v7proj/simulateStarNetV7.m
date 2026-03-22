function sim = simulateStarNetV7(model, TxP_J_effBase_dBm, jamAgg, JamScale_dB, opt)
%SIMULATESTARNETV7 Link-level simulation with optional forced serving/gateway.
%
% Inputs
%   model: packed by emcBuildLinkModel
%   TxP_J_effBase_dBm: jammer effective base switch. Set <= -100 to disable jammer.
%   jamAgg: jammer envelope, N x 1 in [0,1]
%   JamScale_dB: additional jammer scaling driven by GA
%   opt.ForceServing: optional N x 1 vector of satellite indices
%   opt.ForceGateway: optional N x 1 vector of satellite indices
%
% Notes
%   - For downlink, this matches the original V6 behavior.
%   - For uplink, the serving/gateway vectors can be forced to follow the
%     downlink path so that UL/DL are evaluated on the same route.

    if nargin < 5 || isempty(opt)
        opt = struct();
    end
    if ~isfield(opt, 'ForceServing'), opt.ForceServing = []; end
    if ~isfield(opt, 'ForceGateway'), opt.ForceGateway = []; end

    numSteps = model.numSteps;
    dt = model.dt;

    SINR = nan(numSteps,1);
    BER  = nan(numSteps,1);
    THR  = zeros(numSteps,1);
    DOPkHz = nan(numSteps,1);
    E2Ems  = nan(numSteps,1);
    Event  = strings(numSteps,1);

    Serving = zeros(numSteps,1);
    Gateway = zeros(numSteps,1);
    VisUser = zeros(numSteps,1);
    VisGW   = zeros(numSteps,1);
    HopCnt  = zeros(numSteps,1);

    PS_mW    = zeros(numSteps,1);
    PI_mW    = zeros(numSteps,1);
    PJ_mW    = zeros(numSteps,1);
    PJraw_mW = zeros(numSteps,1);
    AJ_Pre   = false(numSteps,1);
    AJ_Post  = false(numSteps,1);

    AJ_Active = false;
    AJ_Timer  = 0;
    JammerOn = (TxP_J_effBase_dBm > -100);

    for k = 1:numSteps
        visU = model.visU_all(k,:);
        visG = model.visG_all(k,:);
        VisUser(k) = sum(visU);
        VisGW(k)   = sum(visG);
        AJ_Pre(k) = AJ_Active;

        if ~any(visU)
            Event(k) = "No Service";
            AJ_Active = false;
            AJ_Timer = 0;
            continue;
        end

        % Gateway selection
        forceGW = 0;
        if ~isempty(opt.ForceGateway) && numel(opt.ForceGateway) >= k
            forceGW = opt.ForceGateway(k);
        end

        if forceGW > 0
            if forceGW <= numel(visG) && visG(forceGW)
                gwIdx = forceGW;
            else
                gwIdx = 0;
            end
        else
            if any(visG)
                idxG = find(visG);
                [~, bg] = max(model.elG(k, idxG));
                gwIdx = idxG(bg);
            else
                gwIdx = 0;
            end
        end
        Gateway(k) = gwIdx;

        cand = find(visU);
        sinrCand = -inf(size(cand));
        pSCand = zeros(size(cand));
        pICand = zeros(size(cand));
        jamPowerCand = zeros(size(cand));
        jamPowerCandRaw = zeros(size(cand));

        for ii = 1:numel(cand)
            si = cand(ii);

            p_s = model.pS_mW(k, si);

            ch = model.satChan(si);
            p_i = model.totCCI(k, ch) - model.pIself_mW(k, si);
            if p_i < 0
                p_i = 0;
            end

            p_j_raw = 0;
            p_j_eff = 0;
            if JammerOn && model.pJsum_base_mW(k, si) > 0
                baseShift_dB = TxP_J_effBase_dBm - model.TxP_J_base_dBm;
                jamScaleLin = 10.^((baseShift_dB + JamScale_dB*jamAgg(k))/10);
                p_j_raw = model.pJsum_base_mW(k, si) * jamScaleLin;
                p_j_eff = p_j_raw;
                if AJ_Active
                    p_j_eff = p_j_raw * 10.^(-model.AJ_NullDepth_dB/10);
                end
            end

            pSCand(ii) = p_s;
            pICand(ii) = p_i;
            jamPowerCand(ii) = p_j_eff;
            jamPowerCandRaw(ii) = p_j_raw;

            sinrLin = p_s / max(p_i + p_j_eff + model.p_n, realmin('double'));
            sinrCand(ii) = 10*log10(sinrLin);
        end

        % Serving selection
        forceServ = 0;
        if ~isempty(opt.ForceServing) && numel(opt.ForceServing) >= k
            forceServ = opt.ForceServing(k);
        end

        if forceServ > 0
            best = find(cand == forceServ, 1, 'first');
            if isempty(best)
                Event(k) = "No Service (Forced)";
                AJ_Active = false;
                AJ_Timer = 0;
                continue;
            end
        else
            if AJ_Active
                [~, best] = max(sinrCand);
            else
                [~, best] = max(model.elU(k, cand));
            end
        end

        servIdx = cand(best);
        Serving(k) = servIdx;

        PS_mW(k) = pSCand(best);
        PI_mW(k) = pICand(best);
        PJ_mW(k) = jamPowerCand(best);
        PJraw_mW(k) = jamPowerCandRaw(best);

        jamOn = JammerOn && (jamPowerCand(best) > 1.0*model.p_n) && (jamAgg(k) > 0.05);

        if jamOn
            AJ_Timer = AJ_Timer + dt;
            if AJ_Timer >= model.AJ_DelaySec
                AJ_Active = true;
            end
        else
            AJ_Timer = 0;
            AJ_Active = false;
        end
        AJ_Post(k) = AJ_Active;

        sinr_dB = sinrCand(best);
        SINR(k) = sinr_dB;

        v_r = model.rrU(k, servIdx);
        dopHz = -(v_r/model.c)*model.fc_Hz;
        DOPkHz(k) = dopHz/1e3;

        lin = 10.^(sinr_dB/10);
        ber = 0.5*erfc(sqrt(lin/2));
        ber = max(min(ber,0.5),1e-9);
        BER(k) = ber;

        if ber > 0.2
            THR(k) = 0;
        else
            eff = min(log2(1 + lin), 6);
            THR(k) = model.BW * eff * 0.8 / 1e6;
        end

        if gwIdx == 0
            E2Ems(k) = nan;
            HopCnt(k) = 0;
        else
            [pth, distISL] = shortestpath(model.Gisl, servIdx, gwIdx);
            if isinf(distISL)
                E2Ems(k) = nan;
                HopCnt(k) = 0;
            else
                HopCnt(k) = max(0, numel(pth) - 1);
                d_total = model.rU(k, servIdx) + model.rG(k, gwIdx) + distISL;
                E2Ems(k) = (d_total/model.c)*1e3;
            end
        end

        ch = model.satChan(servIdx);
        p_i_serv = model.totCCI(k, ch) - model.pIself_mW(k, servIdx);
        if p_i_serv < 0, p_i_serv = 0; end
        p_s_serv = model.pS_mW(k, servIdx);
        isCo = p_i_serv > p_s_serv/10;

        if gwIdx == 0
            if jamOn && ~AJ_Active
                Event(k) = "JAMMING!!! (No GW)";
            elseif jamOn && AJ_Active
                Event(k) = "Protected (No GW)";
            elseif isCo
                Event(k) = "CoChannel (No GW)";
            else
                Event(k) = "Normal (No GW)";
            end
        else
            if jamOn && ~AJ_Active
                Event(k) = "JAMMING!!!";
            elseif jamOn && AJ_Active
                Event(k) = "Protected";
            elseif isCo
                Event(k) = "CoChannel";
            else
                Event(k) = "Normal";
            end
        end
    end

    BLER = ones(numSteps,1);
    for k = 1:numSteps
        if isnan(BER(k))
            BLER(k) = 1;
        else
            BLER(k) = 1 - (1 - min(max(BER(k),0),1)).^model.BLK;
        end
    end

    Prx_dBm = 10*log10(max(PS_mW, realmin('double')));
    Strength_dBm = Prx_dBm + model.SignalStrengthOffset_dB;
    dopRate = [0; abs(diff(DOPkHz*1e3))/model.dt];

    valid = ~isnan(SINR);
    if any(valid)
        meanThr = mean(THR(valid));
        outageFrac = mean(THR(valid) < model.OutageThr_Mbps);
        meanSINR = mean(SINR(valid));
    else
        meanThr = 0;
        outageFrac = 1;
        meanSINR = -inf;
    end

    sim = struct();
    sim.Name = model.Name;
    sim.SINR = SINR;
    sim.BER = BER;
    sim.THR = THR;
    sim.DOPkHz = DOPkHz;
    sim.DopRate_Hzps = dopRate;
    sim.E2Ems = E2Ems;
    sim.Event = Event;
    sim.Serving = Serving;
    sim.Gateway = Gateway;
    sim.VisUser = VisUser;
    sim.VisGW = VisGW;
    sim.Hops = HopCnt;
    sim.BLER = BLER;
    sim.PS_mW = PS_mW;
    sim.PI_mW = PI_mW;
    sim.PJ_mW = PJ_mW;
    sim.PJraw_mW = PJraw_mW;
    sim.Pn_mW = model.p_n;
    sim.AJ_Pre = AJ_Pre;
    sim.AJ_Post = AJ_Post;
    sim.Prx_dBm = Prx_dBm;
    sim.SignalStrength_dBm = Strength_dBm;
    sim.meanThr = meanThr;
    sim.meanSINR = meanSINR;
    sim.outageFrac = outageFrac;
end
