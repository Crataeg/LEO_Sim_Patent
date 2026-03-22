function model = emcBuildLinkModel(name, numSteps, dt, ...
    visU_all, visG_all, azU, elU, rU, rrU, elG, rG, ...
    pS_mW, totCCI, satChan, pIself_mW, pJsum_base_mW, ...
    TxP_J_base_dBm, AJ_DelaySec, AJ_NullDepth_dB, fc_Hz, BW, p_n, Gisl, outageThrMbps, strengthOffset_dB)
%EMCBUILDLINKMODEL Pack link model inputs.

    model = struct();
    model.Name = name;
    model.numSteps = numSteps;
    model.dt = dt;
    model.visU_all = visU_all;
    model.visG_all = visG_all;
    model.azU = azU;
    model.elU = elU;
    model.rU = rU;
    model.rrU = rrU;
    model.elG = elG;
    model.rG = rG;
    model.pS_mW = pS_mW;
    model.totCCI = totCCI;
    model.satChan = satChan;
    model.pIself_mW = pIself_mW;
    model.pJsum_base_mW = pJsum_base_mW;
    model.TxP_J_base_dBm = TxP_J_base_dBm;
    model.AJ_DelaySec = AJ_DelaySec;
    model.AJ_NullDepth_dB = AJ_NullDepth_dB;
    model.fc_Hz = fc_Hz;
    model.c = 299792458;
    model.BW = BW;
    model.p_n = p_n;
    model.Gisl = Gisl;
    model.OutageThr_Mbps = outageThrMbps;
    model.SignalStrengthOffset_dB = strengthOffset_dB;
    model.BLK = 1024;
end
