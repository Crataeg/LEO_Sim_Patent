function emcWriteSummaryTextV7(filePath, cfg, simDL_Base, simDL_Worst, simUL_Base, simUL_Worst, simE2E_Base, simE2E_Worst)
%EMCWRITESUMMARYTEXTV7 Write a plain-text delivery summary.

    fid = fopen(filePath, 'w');
    if fid < 0
        return;
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, 'LEO StarNet EMC V7 Engineering Summary\n');
    fprintf(fid, 'Generated: %s\n\n', datestr(now));

    fprintf(fid, '[Startup]\n');
    fprintf(fid, 'Mode = %s\n', cfg.General.StartupMode);
    fprintf(fid, 'Epoch = %s\n', char(cfg.Time.Epoch));
    fprintf(fid, 'SampleTime = %.2f s\n\n', cfg.Time.SampleTime_s);

    fprintf(fid, '[Key Frequencies]\n');
    fprintf(fid, 'Downlink Fc = %.3f GHz\n', cfg.Downlink.Fc_Hz/1e9);
    fprintf(fid, 'Uplink   Fc = %.3f GHz\n\n', cfg.Uplink.Fc_Hz/1e9);

    fprintf(fid, '[Baseline vs Worst]\n');
    fprintf(fid, 'DL Base  : meanThr = %.2f Mbps, outage = %.2f %%\n', simDL_Base.meanThr, 100*simDL_Base.outageFrac);
    fprintf(fid, 'DL Worst : meanThr = %.2f Mbps, outage = %.2f %%\n', simDL_Worst.meanThr, 100*simDL_Worst.outageFrac);
    fprintf(fid, 'UL Base  : meanThr = %.2f Mbps, outage = %.2f %%\n', simUL_Base.meanThr, 100*simUL_Base.outageFrac);
    fprintf(fid, 'UL Worst : meanThr = %.2f Mbps, outage = %.2f %%\n', simUL_Worst.meanThr, 100*simUL_Worst.outageFrac);
    fprintf(fid, 'E2E Base : meanThr = %.2f Mbps, outage = %.2f %%\n', simE2E_Base.meanThr, 100*simE2E_Base.outageFrac);
    fprintf(fid, 'E2E Worst: meanThr = %.2f Mbps, outage = %.2f %%\n\n', simE2E_Worst.meanThr, 100*simE2E_Worst.outageFrac);

    fprintf(fid, '[Requirements]\n');
    fprintf(fid, 'Min SINR            = %.2f dB\n', cfg.Requirements.MinSINR_dB);
    fprintf(fid, 'Min Throughput      = %.2f Mbps\n', cfg.Requirements.MinThr_Mbps);
    fprintf(fid, 'Rx Sensitivity      = %.2f dBm\n', cfg.Requirements.RxSensitivity_dBm);
    fprintf(fid, 'Max Doppler Rate    = %.2f Hz/s\n', cfg.Requirements.MaxDopplerRate_Hzps);
    fprintf(fid, 'Ku EIRP Current/Min = %.2f / %.2f dBw\n', cfg.Requirements.KuEIRP_Current_dBw, cfg.Requirements.KuEIRP_Min_dBw);
    fprintf(fid, 'Ku G/T Current/Min  = %.2f / %.2f dB/K\n', cfg.Requirements.KuGT_Current_dBperK, cfg.Requirements.KuGT_Min_dBperK);
    fprintf(fid, 'JA3700 Level Cur/Tgt= %d / %d\n', cfg.Requirements.JA3700_CurrentLevel, cfg.Requirements.JA3700_TargetLevel);
end
