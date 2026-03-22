function [rows, overallPass] = emcComputeComplianceRowsV7(cfg, simDL, simUL, simE2E, k)
%EMCCOMPUTECOMPLIANCEROWSV7 Build compliance table rows for dashboard.

    req = cfg.Requirements;
    rows = cell(0,4);
    passFlags = [];

    [rows, passFlags] = addMetric(rows, passFlags, 'DL SINR', ...
        simDL.SINR(k), sprintf('>= %.2f dB', req.MinSINR_dB), simDL.SINR(k) >= req.MinSINR_dB, 'dB');

    [rows, passFlags] = addMetric(rows, passFlags, 'UL SINR', ...
        simUL.SINR(k), sprintf('>= %.2f dB', req.MinSINR_dB), simUL.SINR(k) >= req.MinSINR_dB, 'dB');

    [rows, passFlags] = addMetric(rows, passFlags, 'DL Rx灵敏度', ...
        simDL.Prx_dBm(k), sprintf('>= %.2f dBm', req.RxSensitivity_dBm), simDL.Prx_dBm(k) >= req.RxSensitivity_dBm, 'dBm');

    [rows, passFlags] = addMetric(rows, passFlags, 'UL Rx灵敏度', ...
        simUL.Prx_dBm(k), sprintf('>= %.2f dBm', req.RxSensitivity_dBm), simUL.Prx_dBm(k) >= req.RxSensitivity_dBm, 'dBm');

    [rows, passFlags] = addMetric(rows, passFlags, 'DL 吞吐', ...
        simDL.THR(k), sprintf('>= %.2f Mbps', req.MinThr_Mbps), simDL.THR(k) >= req.MinThr_Mbps, 'Mbps');

    [rows, passFlags] = addMetric(rows, passFlags, 'UL 吞吐', ...
        simUL.THR(k), sprintf('>= %.2f Mbps', req.MinThr_Mbps), simUL.THR(k) >= req.MinThr_Mbps, 'Mbps');

    [rows, passFlags] = addMetric(rows, passFlags, 'E2E 吞吐(UL/DL最小值)', ...
        simE2E.THR(k), sprintf('>= %.2f Mbps', req.MinThr_Mbps), simE2E.THR(k) >= req.MinThr_Mbps, 'Mbps');

    [rows, passFlags] = addMetric(rows, passFlags, 'DL 多普勒变化率', ...
        simDL.DopRate_Hzps(k), sprintf('<= %.2f Hz/s', req.MaxDopplerRate_Hzps), simDL.DopRate_Hzps(k) <= req.MaxDopplerRate_Hzps, 'Hz/s');

    [rows, passFlags] = addMetric(rows, passFlags, 'UL 多普勒变化率', ...
        simUL.DopRate_Hzps(k), sprintf('<= %.2f Hz/s', req.MaxDopplerRate_Hzps), simUL.DopRate_Hzps(k) <= req.MaxDopplerRate_Hzps, 'Hz/s');

    if req.EnableConvertedStrengthCheck
        [rows, passFlags] = addMetric(rows, passFlags, 'DL 转换后信号强度', ...
            simDL.SignalStrength_dBm(k), sprintf('>= %.2f dBm', req.MinSignalStrength_dBm), simDL.SignalStrength_dBm(k) >= req.MinSignalStrength_dBm, 'dBm');
        [rows, passFlags] = addMetric(rows, passFlags, 'UL 转换后信号强度', ...
            simUL.SignalStrength_dBm(k), sprintf('>= %.2f dBm', req.MinSignalStrength_dBm), simUL.SignalStrength_dBm(k) >= req.MinSignalStrength_dBm, 'dBm');
    else
        rows(end+1,:) = {'转换后信号强度', '未启用', sprintf('>= %.2f dBm', req.MinSignalStrength_dBm), 'N/A'}; %#ok<AGROW>
    end

    [rows, passFlags] = addMetric(rows, passFlags, 'Ku EIRP(静态)', ...
        req.KuEIRP_Current_dBw, sprintf('>= %.2f dBw', req.KuEIRP_Min_dBw), req.KuEIRP_Current_dBw >= req.KuEIRP_Min_dBw, 'dBw');

    [rows, passFlags] = addMetric(rows, passFlags, 'Ku G/T(静态)', ...
        req.KuGT_Current_dBperK, sprintf('>= %.2f dB/K', req.KuGT_Min_dBperK), req.KuGT_Current_dBperK >= req.KuGT_Min_dBperK, 'dB/K');

    [rows, passFlags] = addMetric(rows, passFlags, 'JA3700 等级(人工录入)', ...
        req.JA3700_CurrentLevel, sprintf('>= Level %d', req.JA3700_TargetLevel), req.JA3700_CurrentLevel >= req.JA3700_TargetLevel, 'Level');

    overallPass = all(passFlags);
end

function [rows, passFlags] = addMetric(rows, passFlags, name, value, targetTxt, tf, unit)
    if nargin < 7
        unit = '';
    end
    if isempty(value) || (isnumeric(value) && any(isnan(value)))
        valTxt = 'NaN';
        tf = false;
    else
        if isnumeric(value)
            valTxt = sprintf('%.2f %s', double(value), unit);
        else
            valTxt = char(string(value));
        end
    end

    if tf
        statusTxt = 'PASS';
    else
        statusTxt = 'FAIL';
    end

    rows(end+1,:) = {name, valTxt, targetTxt, statusTxt}; %#ok<AGROW>
    passFlags(end+1) = logical(tf); %#ok<AGROW>
end
