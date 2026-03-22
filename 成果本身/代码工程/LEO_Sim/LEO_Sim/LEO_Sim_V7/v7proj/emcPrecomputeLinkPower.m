function [pS_mW, pIself_mW, totCCI] = emcPrecomputeLinkPower( ...
    r_m, visMask, freq_Hz, TxP_S_dBm, TxP_I_dBm, RxGain_dB, IntfPenalty_dB, satChan, reuseK)
%EMCPRECOMPUTELINKPOWER Precompute signal and CCI proxy powers for one link.

    [numSteps, Nsat] = size(r_m);
    pS_mW = zeros(numSteps, Nsat);
    pIself_mW = zeros(numSteps, Nsat);
    freq_MHz = freq_Hz/1e6;

    for i = 1:Nsat
        PL = fspl_dB_vec(r_m(:,i), freq_MHz);
        PsdBm = TxP_S_dBm - PL + RxGain_dB;
        PidBm = TxP_I_dBm - PL + (RxGain_dB - IntfPenalty_dB);

        tmpS = 10.^(PsdBm/10);
        tmpI = 10.^(PidBm/10);

        tmpS(~visMask(:,i)) = 0;
        tmpI(~visMask(:,i)) = 0;

        pS_mW(:,i) = tmpS;
        pIself_mW(:,i) = tmpI;
    end

    totCCI = zeros(numSteps, reuseK);
    for cidx = 1:reuseK
        members = find(satChan == cidx);
        if ~isempty(members)
            totCCI(:,cidx) = sum(pIself_mW(:, members), 2);
        end
    end
end
