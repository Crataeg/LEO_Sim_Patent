function sim = emcCombineE2E(simDL, simUL, outageThr_Mbps)
%EMCCOMBINEE2E End-to-end merged result using min(UL,DL) throughput.

    sim = struct();
    sim.Name = 'E2E';
    sim.THR = min(simDL.THR, simUL.THR);
    sim.BLER = max(simDL.BLER, simUL.BLER);
    sim.SINR = min(simDL.SINR, simUL.SINR);
    sim.Delay_ms = maxIgnoreNaN(simDL.E2Ems, simUL.E2Ems);
    sim.Serving = simDL.Serving;
    sim.Gateway = simDL.Gateway;
    sim.Event = strings(size(simDL.Event));
    for k = 1:numel(sim.Event)
        sim.Event(k) = "DL:" + string(simDL.Event(k)) + " | UL:" + string(simUL.Event(k));
    end

    valid = ~isnan(simDL.SINR) & ~isnan(simUL.SINR);
    if any(valid)
        sim.meanThr = mean(sim.THR(valid));
        sim.outageFrac = mean(sim.THR(valid) < outageThr_Mbps);
    else
        sim.meanThr = 0;
        sim.outageFrac = 1;
    end
end

function y = maxIgnoreNaN(a, b)
    y = a;
    maskA = isnan(a) & ~isnan(b);
    y(maskA) = b(maskA);
    maskB = ~isnan(a) & ~isnan(b);
    y(maskB) = max(a(maskB), b(maskB));
end
