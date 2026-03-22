function f = worstCaseObjectiveV7( ...
    x, netG, seqLen, zDim, cDim, modelDL, modelUL, target, W_outage, W_bler, W_energy)
%WORSTCASEOBJECTIVEV7 GA objective for downlink / uplink / end-to-end worst-case search.

    z = x(1:zDim);
    cCode = x(zDim + (1:cDim));
    JamScale_dB = x(end);

    jamAgg = genJamAggFromG(netG, z, cCode, seqLen, modelDL.numSteps);

    simDL = simulateStarNetV7(modelDL, modelDL.TxP_J_base_dBm, jamAgg, JamScale_dB, struct());

    target = lower(strtrim(target));
    switch target
        case 'uplink'
            simUL = simulateStarNetV7(modelUL, modelUL.TxP_J_base_dBm, jamAgg, JamScale_dB, ...
                struct('ForceServing', simDL.Serving, 'ForceGateway', simDL.Gateway));
            valid = ~isnan(simUL.SINR);
            if ~any(valid)
                f = 1e6;
                return;
            end
            meanThr = mean(simUL.THR(valid));
            outage = mean(simUL.THR(valid) < modelUL.OutageThr_Mbps);
            meanBler = mean(simUL.BLER(valid));

        case 'e2e'
            simUL = simulateStarNetV7(modelUL, modelUL.TxP_J_base_dBm, jamAgg, JamScale_dB, ...
                struct('ForceServing', simDL.Serving, 'ForceGateway', simDL.Gateway));
            thrE2E = min(simDL.THR, simUL.THR);
            blerE2E = max(simDL.BLER, simUL.BLER);
            valid = ~isnan(simDL.SINR) & ~isnan(simUL.SINR);
            if ~any(valid)
                f = 1e6;
                return;
            end
            meanThr = mean(thrE2E(valid));
            outage = mean(thrE2E(valid) < min(modelDL.OutageThr_Mbps, modelUL.OutageThr_Mbps));
            meanBler = mean(blerE2E(valid));

        otherwise
            valid = ~isnan(simDL.SINR);
            if ~any(valid)
                f = 1e6;
                return;
            end
            meanThr = mean(simDL.THR(valid));
            outage = mean(simDL.THR(valid) < modelDL.OutageThr_Mbps);
            meanBler = mean(simDL.BLER(valid));
    end

    energy = mean(jamAgg.^2);
    duty = mean(jamAgg > 0.2);
    penalty = W_energy*(energy + max(0, duty - 0.55).^2 * 10);

    f = -meanThr + W_outage*outage + W_bler*meanBler + penalty;
end
