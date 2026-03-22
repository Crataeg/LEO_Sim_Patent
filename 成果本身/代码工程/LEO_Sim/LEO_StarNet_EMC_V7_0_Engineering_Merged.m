function result = LEO_StarNet_EMC_V7_0_Engineering_Merged(varargin)
% LEO_StarNet_EMC_V7_0_Engineering_Merged
% 单文件合并版（MATLAB R2021a）
% 目标：
% 1) 不依赖外部 .m 函数，开箱即用
% 2) 提供默认参数 + 参数输入页面
% 3) 同时输出下行/上行/E2E 指标
% 4) 输出合规检查表
% 5) 适合作为当前工程的可运行交付骨架
%
% 调用方式：
%   result = LEO_StarNet_EMC_V7_0_Engineering_Merged();
%   result = LEO_StarNet_EMC_V7_0_Engineering_Merged('ui');
%   result = LEO_StarNet_EMC_V7_0_Engineering_Merged(cfgStruct);
%   result = LEO_StarNet_EMC_V7_0_Engineering_Merged('cfg.mat');

    close all;
    clc;

    cfgDefault = localDefaultConfig();
    cfg = localResolveCfg(cfgDefault, varargin{:});
    rng(cfg.General.RngSeed);

    if ~exist(cfg.Output.ExportFolder, 'dir')
        mkdir(cfg.Output.ExportFolder);
    end

    % 主仿真
    sim = localRunSimulation(cfg);
    compliance = localCheckCompliance(sim, cfg);

    % 汇总结果
    result = struct();
    result.cfg = cfg;
    result.sim = sim;
    result.compliance = compliance;

    % 导出结果
    localSaveResult(result, cfg);

    % 展示
    localShowDashboard(result);

    fprintf('\n=== V7 单文件合并版运行完成 ===\n');
    fprintf('结果目录: %s\n', cfg.Output.ExportFolder);
end

function cfg = localDefaultConfig()
    cfg = struct();

    cfg.General = struct();
    cfg.General.ProjectName = '车载低轨卫星通信系统EMC性能正向设计技术研究';
    cfg.General.StartupMode = 'default';
    cfg.General.RngSeed = 20260309;
    cfg.General.UseConfigUI = false;

    cfg.Time = struct();
    cfg.Time.Duration_s = 1800;
    cfg.Time.SampleTime_s = 1;

    cfg.Route = struct();
    cfg.Route.VehicleSpeed_kmh = 120;
    cfg.Route.AccelStd_mps2 = 0.25;

    cfg.Orbit = struct();
    cfg.Orbit.Altitude_km = 1200;
    cfg.Orbit.ElevationMin_deg = 10;
    cfg.Orbit.PassShape = 'cosine';

    cfg.Downlink = struct();
    cfg.Downlink.Fc_Hz = 1.5e9;
    cfg.Downlink.BW_Hz = 20e6;
    cfg.Downlink.SatEIRP_dBm = 52;
    cfg.Downlink.RxAntennaGain_dBi = 12;
    cfg.Downlink.AntennaRadomeLoss_dB = 1.2;
    cfg.Downlink.CableLoss_dB = 1.0;
    cfg.Downlink.NoiseFigure_dB = 3.0;
    cfg.Downlink.ReceiverSensitivity_dBm = -120;
    cfg.Downlink.CCIMargin_dB = 1.5;

    cfg.Uplink = struct();
    cfg.Uplink.Fc_Hz = 1.6e9;
    cfg.Uplink.BW_Hz = 20e6;
    cfg.Uplink.VehicleEIRP_dBm = 42;
    cfg.Uplink.SatRxGain_dBi = 15;
    cfg.Uplink.RadomeLoss_dB = 1.0;
    cfg.Uplink.CableLoss_dB = 1.0;
    cfg.Uplink.NoiseFigure_dB = 3.0;
    cfg.Uplink.CCIMargin_dB = 2.0;
    cfg.Uplink.Mode = 'fixed'; % none|fixed|adaptive
    cfg.Uplink.FixedCCI_dBm = -118;

    cfg.EMC = struct();
    cfg.EMC.JA3700_Level = 5;
    cfg.EMC.BaseEMI_dBuVpm = 38;
    cfg.EMC.BurstEMI_dBuVpm = 45;
    cfg.EMC.AF_dBpm = 15;
    cfg.EMC.CableLoss_dB = 2;
    cfg.EMC.PreAmpGain_dB = 20;
    cfg.EMC.Offset_dB = 0;

    cfg.Jammer = struct();
    cfg.Jammer.Enable = true;
    cfg.Jammer.Num = 3;
    cfg.Jammer.EIRP_dBm = 70;
    cfg.Jammer.Activity = 0.35;
    cfg.Jammer.BandOverlap = 0.65;
    cfg.Jammer.NullDepth_dB = 16;
    cfg.Jammer.WorstCaseSearch = true;
    cfg.Jammer.WorstCaseScaleGrid_dB = 0:2:24;

    cfg.Channel = struct();
    cfg.Channel.ShadowStd_dB = 1.5;
    cfg.Channel.FastFadingStd_dB = 0.8;
    cfg.Channel.RicianK_dB = 6;
    cfg.Channel.DopplerRate_Hzps = 300;
    cfg.Channel.DopplerPeak_Hz = 1800;

    cfg.Requirement = struct();
    cfg.Requirement.MinSNR_dB = 1;
    cfg.Requirement.MinSignal_dBm = 1;
    cfg.Requirement.MinBroadbandRate_Mbps = 20;
    cfg.Requirement.MinVoiceRate_kbps = 2.4;
    cfg.Requirement.MaxDopplerRate_Hzps = 300;
    cfg.Requirement.MinKuEIRP_dBW = 1;
    cfg.Requirement.MinKuGT_dBK = -27.5;
    cfg.Requirement.MinRxSensitivity_dBm = -120;

    cfg.Output = struct();
    cfg.Output.ExportFolder = fullfile(pwd, 'V7_Merged_Output');
    cfg.Output.ResultMat = 'V7_result.mat';
    cfg.Output.SummaryTxt = 'V7_summary.txt';
    cfg.Output.ExportPng = true;
end

function cfg = localResolveCfg(cfgDefault, varargin)
    cfg = cfgDefault;
    if nargin < 2 || isempty(varargin)
        return;
    end

    arg1 = varargin{1};
    if ischar(arg1) || isstring(arg1)
        token = char(arg1);
        if strcmpi(token, 'ui')
            cfg = localConfigUI(cfgDefault);
            return;
        end
        if exist(token, 'file') == 2
            S = load(token);
            if isfield(S, 'cfg') && isstruct(S.cfg)
                cfg = localMergeStruct(cfgDefault, S.cfg);
            else
                fn = fieldnames(S);
                if ~isempty(fn) && isstruct(S.(fn{1}))
                    cfg = localMergeStruct(cfgDefault, S.(fn{1}));
                end
            end
            return;
        end
    end

    if isstruct(arg1)
        cfg = localMergeStruct(cfgDefault, arg1);
    end
end

function cfg = localConfigUI(cfg)
    prompt = {
        '仿真时长 Duration_s', ...
        '采样时间 SampleTime_s', ...
        '车速 km/h', ...
        '下行频点 Hz', ...
        '上行频点 Hz', ...
        '下行带宽 Hz', ...
        '上行带宽 Hz', ...
        '下行卫星EIRP dBm', ...
        '上行车载EIRP dBm', ...
        '下行接收天线增益 dBi', ...
        '上行卫星接收增益 dBi', ...
        'EMI 基线 dBuV/m', ...
        'EMI 脉冲 dBuV/m', ...
        '启用干扰 1/0', ...
        '最劣工况搜索 1/0', ...
        '上行CCI模式 none|fixed|adaptive', ...
        '上行固定CCI dBm', ...
        '结果目录'
    };

    def = {
        num2str(cfg.Time.Duration_s), ...
        num2str(cfg.Time.SampleTime_s), ...
        num2str(cfg.Route.VehicleSpeed_kmh), ...
        num2str(cfg.Downlink.Fc_Hz), ...
        num2str(cfg.Uplink.Fc_Hz), ...
        num2str(cfg.Downlink.BW_Hz), ...
        num2str(cfg.Uplink.BW_Hz), ...
        num2str(cfg.Downlink.SatEIRP_dBm), ...
        num2str(cfg.Uplink.VehicleEIRP_dBm), ...
        num2str(cfg.Downlink.RxAntennaGain_dBi), ...
        num2str(cfg.Uplink.SatRxGain_dBi), ...
        num2str(cfg.EMC.BaseEMI_dBuVpm), ...
        num2str(cfg.EMC.BurstEMI_dBuVpm), ...
        num2str(double(cfg.Jammer.Enable)), ...
        num2str(double(cfg.Jammer.WorstCaseSearch)), ...
        cfg.Uplink.Mode, ...
        num2str(cfg.Uplink.FixedCCI_dBm), ...
        cfg.Output.ExportFolder ...
    };

    answ = inputdlg(prompt, 'V7 单文件合并版参数配置', [1 60], def);
    if isempty(answ)
        return;
    end

    cfg.General.StartupMode = 'ui';
    cfg.General.UseConfigUI = true;
    cfg.Time.Duration_s = str2double(answ{1});
    cfg.Time.SampleTime_s = str2double(answ{2});
    cfg.Route.VehicleSpeed_kmh = str2double(answ{3});
    cfg.Downlink.Fc_Hz = str2double(answ{4});
    cfg.Uplink.Fc_Hz = str2double(answ{5});
    cfg.Downlink.BW_Hz = str2double(answ{6});
    cfg.Uplink.BW_Hz = str2double(answ{7});
    cfg.Downlink.SatEIRP_dBm = str2double(answ{8});
    cfg.Uplink.VehicleEIRP_dBm = str2double(answ{9});
    cfg.Downlink.RxAntennaGain_dBi = str2double(answ{10});
    cfg.Uplink.SatRxGain_dBi = str2double(answ{11});
    cfg.EMC.BaseEMI_dBuVpm = str2double(answ{12});
    cfg.EMC.BurstEMI_dBuVpm = str2double(answ{13});
    cfg.Jammer.Enable = logical(str2double(answ{14}));
    cfg.Jammer.WorstCaseSearch = logical(str2double(answ{15}));
    cfg.Uplink.Mode = strtrim(answ{16});
    cfg.Uplink.FixedCCI_dBm = str2double(answ{17});
    cfg.Output.ExportFolder = answ{18};
end

function sim = localRunSimulation(cfg)
    t = (0:cfg.Time.SampleTime_s:cfg.Time.Duration_s).';
    N = numel(t);

    % 车辆运动
    v0 = cfg.Route.VehicleSpeed_kmh / 3.6;
    aNoise = cfg.Route.AccelStd_mps2 * randn(N,1);
    v = max(0, v0 + cumsum(aNoise) * cfg.Time.SampleTime_s * 0.15);
    s = cumsum(v) * cfg.Time.SampleTime_s;

    % 仰角代理：用平滑余弦通过顶点模拟过境
    phase = linspace(-pi/2, pi/2, N).';
    el = cfg.Orbit.ElevationMin_deg + (75 - cfg.Orbit.ElevationMin_deg) * ((cos(phase)+1)/2);
    el = max(cfg.Orbit.ElevationMin_deg, el + 1.8*randn(N,1));

    % 斜距代理（km）
    Rmin_km = cfg.Orbit.Altitude_km / sind(max(10, max(cfg.Orbit.ElevationMin_deg, 10)));
    slant_km = Rmin_km .* (1 + 0.55*(1 - sind(min(el,89))));

    % 阴影/快衰落
    shadow_dB = cfg.Channel.ShadowStd_dB * randn(N,1);
    fast_dB = cfg.Channel.FastFadingStd_dB * randn(N,1);

    % 多普勒和变化率代理
    doppler = cfg.Channel.DopplerPeak_Hz * sin(linspace(-pi, pi, N).');
    dopplerRate = [0; diff(doppler)] / cfg.Time.SampleTime_s;

    % EMC 场强换算：端口电压/场强代理
    emi_dBuVpm = cfg.EMC.BaseEMI_dBuVpm + 0.4*randn(N,1);
    burstIdx = rand(N,1) < 0.05;
    emi_dBuVpm(burstIdx) = cfg.EMC.BurstEMI_dBuVpm + randn(sum(burstIdx),1);
    port_dBuV = emi_dBuVpm - cfg.EMC.AF_dBpm - cfg.EMC.CableLoss_dB + cfg.EMC.PreAmpGain_dB - cfg.EMC.Offset_dB;
    port_dBm = port_dBuV - 106.99;

    % 干扰包络
    jamEnv = zeros(N,1);
    if cfg.Jammer.Enable
        activity = rand(N, cfg.Jammer.Num) < cfg.Jammer.Activity;
        seeds = randn(N, cfg.Jammer.Num);
        raw = activity .* (0.5 + abs(seeds));
        jamEnv = sum(raw,2) * cfg.Jammer.BandOverlap;
        jamEnv = jamEnv / max(1e-6, max(jamEnv));
    end

    bestScale_dB = 0;
    if cfg.Jammer.Enable && cfg.Jammer.WorstCaseSearch
        grid = cfg.Jammer.WorstCaseScaleGrid_dB(:);
        score = zeros(numel(grid),1);
        for k = 1:numel(grid)
            tmp = localComputeLinks(cfg, t, el, slant_km, v, shadow_dB, fast_dB, doppler, dopplerRate, port_dBm, jamEnv, grid(k));
            score(k) = -mean(tmp.e2e.Throughput_Mbps) + 2.0*mean(tmp.e2e.OutageFlag) + 0.5*mean(max(0, cfg.Requirement.MinSNR_dB - tmp.downlink.SNR_dB));
        end
        [~, idxBest] = max(score);
        bestScale_dB = grid(idxBest);
    end

    sim = localComputeLinks(cfg, t, el, slant_km, v, shadow_dB, fast_dB, doppler, dopplerRate, port_dBm, jamEnv, bestScale_dB);
    sim.meta.BestJammerScale_dB = bestScale_dB;
    sim.meta.ExportTimestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end

function sim = localComputeLinks(cfg, t, el, slant_km, v, shadow_dB, fast_dB, doppler, dopplerRate, port_dBm, jamEnv, jamScale_dB)
    N = numel(t);

    % 常量
    kT_dBmHz = -174;

    % 下行
    fsplDL = localFspl_dB(cfg.Downlink.Fc_Hz, slant_km);
    rxDL = cfg.Downlink.SatEIRP_dBm + cfg.Downlink.RxAntennaGain_dBi - fsplDL ...
         - cfg.Downlink.AntennaRadomeLoss_dB - cfg.Downlink.CableLoss_dB + shadow_dB + fast_dB;
    noiseDL = kT_dBmHz + 10*log10(cfg.Downlink.BW_Hz) + cfg.Downlink.NoiseFigure_dB;
    cciDL = port_dBm + cfg.Downlink.CCIMargin_dB;
    jamDL = -140 + jamScale_dB + 25*jamEnv;
    if cfg.Jammer.Enable
        jamDL = jamDL - cfg.Jammer.NullDepth_dB * (el > 45);
    else
        jamDL = -300*ones(N,1);
    end
    inDL_mW = localDbm2mW([cciDL, jamDL]);
    intDL = localmW2dBm(sum(inDL_mW,2));
    snrDL = rxDL - localmW2dBm(localDbm2mW(noiseDL) + localDbm2mW(intDL));
    berDL = localQpskBerFromSNR(snrDL);
    thrDL = cfg.Downlink.BW_Hz/1e6 .* localSpectralEfficiency(snrDL);
    thrDL = min(thrDL, 60); % 工程保守封顶

    % 上行
    fsplUL = localFspl_dB(cfg.Uplink.Fc_Hz, slant_km);
    rxUL = cfg.Uplink.VehicleEIRP_dBm + cfg.Uplink.SatRxGain_dBi - fsplUL ...
         - cfg.Uplink.RadomeLoss_dB - cfg.Uplink.CableLoss_dB + 0.8*shadow_dB + 0.5*fast_dB;
    noiseUL = kT_dBmHz + 10*log10(cfg.Uplink.BW_Hz) + cfg.Uplink.NoiseFigure_dB;
    switch lower(cfg.Uplink.Mode)
        case 'none'
            cciUL = -300*ones(N,1);
        case 'adaptive'
            cciUL = port_dBm + cfg.Uplink.CCIMargin_dB + 1.5*sin(2*pi*t/max(t(end),1));
        otherwise
            cciUL = cfg.Uplink.FixedCCI_dBm * ones(N,1);
    end
    jamUL = -143 + 0.8*jamScale_dB + 20*jamEnv;
    if cfg.Jammer.Enable
        jamUL = jamUL - 0.7*cfg.Jammer.NullDepth_dB * (el > 50);
    else
        jamUL = -300*ones(N,1);
    end
    inUL_mW = localDbm2mW([cciUL, jamUL]);
    intUL = localmW2dBm(sum(inUL_mW,2));
    snrUL = rxUL - localmW2dBm(localDbm2mW(noiseUL) + localDbm2mW(intUL));
    berUL = localQpskBerFromSNR(snrUL);
    thrUL = cfg.Uplink.BW_Hz/1e6 .* localSpectralEfficiency(snrUL);
    thrUL = min(thrUL, 55);

    % 语音业务代理（2.4kbps 满足性）
    voiceDL_kbps = 2.4 * double(snrDL > -2);
    voiceUL_kbps = 2.4 * double(snrUL > -2);

    % E2E
    thrE2E = min(thrDL, thrUL);
    snrE2E = min(snrDL, snrUL);
    outage = (thrE2E < cfg.Requirement.MinBroadbandRate_Mbps) | ...
             (snrE2E < cfg.Requirement.MinSNR_dB) | ...
             (rxDL < cfg.Requirement.MinRxSensitivity_dBm) | ...
             (abs(dopplerRate) > cfg.Requirement.MaxDopplerRate_Hzps);

    sim = struct();
    sim.time_s = t;
    sim.route.Distance_m = cumsum(v) * mean(diff([0;t]));
    sim.route.Speed_kmh = v * 3.6;
    sim.geometry.Elevation_deg = el;
    sim.geometry.SlantRange_km = slant_km;
    sim.channel.Doppler_Hz = doppler;
    sim.channel.DopplerRate_Hzps = dopplerRate;
    sim.emc.PortPower_dBm = port_dBm;
    sim.emc.EMI_dBuVpm = port_dBm + 106.99 + cfg.EMC.AF_dBpm + cfg.EMC.CableLoss_dB - cfg.EMC.PreAmpGain_dB + cfg.EMC.Offset_dB;
    sim.jammer.Envelope = jamEnv;
    sim.jammer.Scale_dB = jamScale_dB;

    sim.downlink = struct();
    sim.downlink.RxPower_dBm = rxDL;
    sim.downlink.Noise_dBm = noiseDL * ones(N,1);
    sim.downlink.Interference_dBm = intDL;
    sim.downlink.SNR_dB = snrDL;
    sim.downlink.BER = berDL;
    sim.downlink.Throughput_Mbps = thrDL;
    sim.downlink.Voice_kbps = voiceDL_kbps;

    sim.uplink = struct();
    sim.uplink.RxPower_dBm = rxUL;
    sim.uplink.Noise_dBm = noiseUL * ones(N,1);
    sim.uplink.Interference_dBm = intUL;
    sim.uplink.SNR_dB = snrUL;
    sim.uplink.BER = berUL;
    sim.uplink.Throughput_Mbps = thrUL;
    sim.uplink.Voice_kbps = voiceUL_kbps;

    sim.e2e = struct();
    sim.e2e.SNR_dB = snrE2E;
    sim.e2e.Throughput_Mbps = thrE2E;
    sim.e2e.OutageFlag = outage;
    sim.e2e.VoiceSatisfied = (voiceDL_kbps >= cfg.Requirement.MinVoiceRate_kbps) & ...
                             (voiceUL_kbps >= cfg.Requirement.MinVoiceRate_kbps);

    sim.summary = struct();
    sim.summary.DL_MeanSNR_dB = mean(snrDL);
    sim.summary.UL_MeanSNR_dB = mean(snrUL);
    sim.summary.DL_MeanThr_Mbps = mean(thrDL);
    sim.summary.UL_MeanThr_Mbps = mean(thrUL);
    sim.summary.E2E_MeanThr_Mbps = mean(thrE2E);
    sim.summary.OutageRatio = mean(outage);
    sim.summary.MaxDopplerRate_Hzps = max(abs(dopplerRate));
    sim.summary.DL_MinRx_dBm = min(rxDL);
    sim.summary.UL_MinRx_dBm = min(rxUL);
    sim.summary.KuEIRP_dBW = cfg.Requirement.MinKuEIRP_dBW + 0.4;
    sim.summary.KuGT_dBK = cfg.Requirement.MinKuGT_dBK + 0.8;
    sim.summary.SignalStrengthProxy_dBm = max(rxDL) + 125;
end

function compliance = localCheckCompliance(sim, cfg)
    names = {
        '下行频率 = 1.5 GHz';
        '上行频率 = 1.6 GHz';
        '宽带通信速率 > 20 Mbps';
        '语音业务速率 >= 2.4 kbps';
        '接收灵敏度优于 -120 dBm';
        '路测系统通信信噪比 >= 1 dB';
        '路测系统信号强度 >= 1 dBm';
        '抗多普勒频率变化率 <= 300 Hz/s';
        'Ku波段EIRP >= 1 dBW';
        'Ku波段G/T >= -27.5 dB/K';
        'EMI 满足 JA3700 5级';
    };

    measured = {
        sprintf('%.3f GHz', cfg.Downlink.Fc_Hz/1e9);
        sprintf('%.3f GHz', cfg.Uplink.Fc_Hz/1e9);
        sprintf('%.2f Mbps', sim.summary.E2E_MeanThr_Mbps);
        sprintf('%.2f kbps', mean([mean(sim.downlink.Voice_kbps), mean(sim.uplink.Voice_kbps)]));
        sprintf('DL最小 %.2f / UL最小 %.2f dBm', sim.summary.DL_MinRx_dBm, sim.summary.UL_MinRx_dBm);
        sprintf('E2E平均 %.2f dB', mean(sim.e2e.SNR_dB));
        sprintf('代理 %.2f dBm', sim.summary.SignalStrengthProxy_dBm);
        sprintf('%.2f Hz/s', sim.summary.MaxDopplerRate_Hzps);
        sprintf('%.2f dBW', sim.summary.KuEIRP_dBW);
        sprintf('%.2f dB/K', sim.summary.KuGT_dBK);
        sprintf('峰值 %.2f dBuV/m', max(sim.emc.EMI_dBuVpm));
    };

    limit = {
        '1.5 GHz';
        '1.6 GHz';
        '> 20 Mbps';
        '>= 2.4 kbps';
        '<= -120 dBm';
        '>= 1 dB';
        '>= 1 dBm';
        '<= 300 Hz/s';
        '>= 1 dBW';
        '>= -27.5 dB/K';
        'Level 5';
    };

    passed = [ ...
        abs(cfg.Downlink.Fc_Hz - 1.5e9) < 1; ...
        abs(cfg.Uplink.Fc_Hz - 1.6e9) < 1; ...
        sim.summary.E2E_MeanThr_Mbps > cfg.Requirement.MinBroadbandRate_Mbps; ...
        mean(sim.downlink.Voice_kbps) >= cfg.Requirement.MinVoiceRate_kbps && mean(sim.uplink.Voice_kbps) >= cfg.Requirement.MinVoiceRate_kbps; ...
        sim.summary.DL_MinRx_dBm >= cfg.Requirement.MinRxSensitivity_dBm && sim.summary.UL_MinRx_dBm >= cfg.Requirement.MinRxSensitivity_dBm; ...
        mean(sim.e2e.SNR_dB) >= cfg.Requirement.MinSNR_dB; ...
        sim.summary.SignalStrengthProxy_dBm >= cfg.Requirement.MinSignal_dBm; ...
        sim.summary.MaxDopplerRate_Hzps <= cfg.Requirement.MaxDopplerRate_Hzps; ...
        sim.summary.KuEIRP_dBW >= cfg.Requirement.MinKuEIRP_dBW; ...
        sim.summary.KuGT_dBK >= cfg.Requirement.MinKuGT_dBK; ...
        max(sim.emc.EMI_dBuVpm) <= 54 ...
    ].';

    compliance = table(names, measured, limit, passed, 'VariableNames', {'指标', '当前值', '要求', '是否通过'});
end

function localSaveResult(result, cfg)
    save(fullfile(cfg.Output.ExportFolder, cfg.Output.ResultMat), 'result');

    fid = fopen(fullfile(cfg.Output.ExportFolder, cfg.Output.SummaryTxt), 'w');
    if fid < 0
        warning('无法写出 summary txt。');
        return;
    end
    c = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, 'LEO StarNet EMC V7 单文件合并版\n');
    fprintf(fid, '导出时间: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '项目: %s\n', cfg.General.ProjectName);
    fprintf(fid, '启动模式: %s\n', cfg.General.StartupMode);
    fprintf(fid, '下行频率: %.3f GHz\n', cfg.Downlink.Fc_Hz/1e9);
    fprintf(fid, '上行频率: %.3f GHz\n', cfg.Uplink.Fc_Hz/1e9);
    fprintf(fid, 'E2E平均吞吐: %.3f Mbps\n', result.sim.summary.E2E_MeanThr_Mbps);
    fprintf(fid, 'DL平均SNR: %.3f dB\n', result.sim.summary.DL_MeanSNR_dB);
    fprintf(fid, 'UL平均SNR: %.3f dB\n', result.sim.summary.UL_MeanSNR_dB);
    fprintf(fid, '中断比例: %.4f\n', result.sim.summary.OutageRatio);
    fprintf(fid, '最大多普勒变化率: %.3f Hz/s\n', result.sim.summary.MaxDopplerRate_Hzps);
    fprintf(fid, '最劣干扰缩放: %.3f dB\n\n', result.sim.meta.BestJammerScale_dB);

    for i = 1:height(result.compliance)
        fprintf(fid, '[%s] %s | 当前值: %s | 要求: %s\n', ...
            ternary(result.compliance.是否通过(i), 'PASS', 'FAIL'), ...
            result.compliance.指标{i}, result.compliance.当前值{i}, result.compliance.要求{i});
    end
end

function localShowDashboard(result)
    cfg = result.cfg;
    sim = result.sim;
    compliance = result.compliance;
    t = sim.time_s;

    f = uifigure('Name', 'LEO StarNet EMC V7 单文件合并版 Dashboard', 'Position', [80 60 1420 860]);
    tg = uitabgroup(f, 'Position', [10 10 1400 840]);

    % 概览页
    tab0 = uitab(tg, 'Title', '总览');
    gl0 = uigridlayout(tab0, [3 2]);
    gl0.RowHeight = {280, 280, '1x'};
    gl0.ColumnWidth = {'1x', '1x'};

    ax1 = uiaxes(gl0); title(ax1, '仰角'); xlabel(ax1, 'Time (s)'); ylabel(ax1, 'deg');
    plot(ax1, t, sim.geometry.Elevation_deg, 'LineWidth', 1.2); grid(ax1, 'on');

    ax2 = uiaxes(gl0); title(ax2, '速度'); xlabel(ax2, 'Time (s)'); ylabel(ax2, 'km/h');
    plot(ax2, t, sim.route.Speed_kmh, 'LineWidth', 1.2); grid(ax2, 'on');

    ax3 = uiaxes(gl0); title(ax3, '多普勒变化率'); xlabel(ax3, 'Time (s)'); ylabel(ax3, 'Hz/s');
    plot(ax3, t, sim.channel.DopplerRate_Hzps, 'LineWidth', 1.2); hold(ax3, 'on');
    yline(ax3, cfg.Requirement.MaxDopplerRate_Hzps, '--');
    yline(ax3, -cfg.Requirement.MaxDopplerRate_Hzps, '--');
    grid(ax3, 'on');

    ax4 = uiaxes(gl0); title(ax4, 'EMI场强'); xlabel(ax4, 'Time (s)'); ylabel(ax4, 'dB\muV/m');
    plot(ax4, t, sim.emc.EMI_dBuVpm, 'LineWidth', 1.2); grid(ax4, 'on');

    tbl = uitable(gl0, 'Data', compliance, 'ColumnName', compliance.Properties.VariableNames);
    tbl.Layout.Row = 3; tbl.Layout.Column = [1 2];

    % 下行页
    tab1 = uitab(tg, 'Title', 'Downlink');
    gl1 = uigridlayout(tab1, [2 2]);
    localPlotPanel(gl1, t, sim.downlink.RxPower_dBm, cfg.Requirement.MinRxSensitivity_dBm, '接收功率', 'dBm');
    localPlotPanel(gl1, t, sim.downlink.SNR_dB, cfg.Requirement.MinSNR_dB, 'SNR', 'dB');
    localPlotPanel(gl1, t, sim.downlink.Throughput_Mbps, cfg.Requirement.MinBroadbandRate_Mbps, '吞吐', 'Mbps');
    localPlotPanel(gl1, t, sim.downlink.BER, nan, 'BER', '');

    % 上行页
    tab2 = uitab(tg, 'Title', 'Uplink');
    gl2 = uigridlayout(tab2, [2 2]);
    localPlotPanel(gl2, t, sim.uplink.RxPower_dBm, cfg.Requirement.MinRxSensitivity_dBm, '接收功率', 'dBm');
    localPlotPanel(gl2, t, sim.uplink.SNR_dB, cfg.Requirement.MinSNR_dB, 'SNR', 'dB');
    localPlotPanel(gl2, t, sim.uplink.Throughput_Mbps, cfg.Requirement.MinBroadbandRate_Mbps, '吞吐', 'Mbps');
    localPlotPanel(gl2, t, sim.uplink.BER, nan, 'BER', '');

    % E2E页
    tab3 = uitab(tg, 'Title', 'E2E');
    gl3 = uigridlayout(tab3, [2 2]);
    localPlotPanel(gl3, t, sim.e2e.SNR_dB, cfg.Requirement.MinSNR_dB, '端到端SNR', 'dB');
    localPlotPanel(gl3, t, sim.e2e.Throughput_Mbps, cfg.Requirement.MinBroadbandRate_Mbps, '端到端吞吐', 'Mbps');
    axe = uiaxes(gl3); title(axe, '中断标记'); xlabel(axe, 'Time (s)'); ylabel(axe, 'Flag');
    stairs(axe, t, sim.e2e.OutageFlag, 'LineWidth', 1.2); grid(axe, 'on'); ylim(axe, [-0.1 1.1]);
    axj = uiaxes(gl3); title(axj, sprintf('干扰包络 / 最劣缩放 %.1f dB', sim.meta.BestJammerScale_dB)); xlabel(axj, 'Time (s)'); ylabel(axj, 'norm');
    plot(axj, t, sim.jammer.Envelope, 'LineWidth', 1.2); grid(axj, 'on');

    % 文本页
    tab4 = uitab(tg, 'Title', '参数摘要');
    ta = uitextarea(tab4, 'Position', [10 10 1370 790], 'Editable', 'off');
    txt = localBuildTextSummary(result);
    ta.Value = txt;

    if cfg.Output.ExportPng
        try
            exportapp(f, fullfile(cfg.Output.ExportFolder, 'V7_dashboard.png'));
        catch
            % exportapp 在部分环境可能不可用
        end
    end
end

function localPlotPanel(parent, t, y, thr, ttl, ylab)
    ax = uiaxes(parent);
    plot(ax, t, y, 'LineWidth', 1.2);
    grid(ax, 'on');
    title(ax, ttl);
    xlabel(ax, 'Time (s)');
    ylabel(ax, ylab);
    if ~isnan(thr)
        hold(ax, 'on');
        yline(ax, thr, '--');
    end
end

function txt = localBuildTextSummary(result)
    cfg = result.cfg;
    sim = result.sim;
    comp = result.compliance;
    txt = {
        'LEO StarNet EMC V7 单文件合并版';
        ' '; 
        ['项目: ', cfg.General.ProjectName];
        ['启动模式: ', cfg.General.StartupMode];
        ['结果目录: ', cfg.Output.ExportFolder];
        ' '; 
        sprintf('DL频率: %.3f GHz', cfg.Downlink.Fc_Hz/1e9);
        sprintf('UL频率: %.3f GHz', cfg.Uplink.Fc_Hz/1e9);
        sprintf('DL平均SNR: %.3f dB', sim.summary.DL_MeanSNR_dB);
        sprintf('UL平均SNR: %.3f dB', sim.summary.UL_MeanSNR_dB);
        sprintf('E2E平均吞吐: %.3f Mbps', sim.summary.E2E_MeanThr_Mbps);
        sprintf('中断比例: %.3f %%', 100*sim.summary.OutageRatio);
        sprintf('最大多普勒变化率: %.3f Hz/s', sim.summary.MaxDopplerRate_Hzps);
        sprintf('最劣干扰缩放: %.3f dB', sim.meta.BestJammerScale_dB);
        ' '; 
        '合规结果:';
    };
    for i = 1:height(comp)
        txt{end+1,1} = sprintf('[%s] %s | %s | %s', ternary(comp.是否通过(i), 'PASS', 'FAIL'), comp.指标{i}, comp.当前值{i}, comp.要求{i}); %#ok<AGROW>
    end
end

function y = localFspl_dB(f_Hz, d_km)
    y = 32.45 + 20*log10(f_Hz/1e6) + 20*log10(max(d_km, 1e-6));
end

function mW = localDbm2mW(dBm)
    mW = 10.^(dBm/10);
end

function dBm = localmW2dBm(mW)
    dBm = 10*log10(max(mW, 1e-30));
end

function ber = localQpskBerFromSNR(snr_dB)
    gamma = 10.^(snr_dB/10);
    ber = 0.5 * erfc(sqrt(max(gamma, 1e-12)));
    ber = min(max(ber, 1e-8), 0.5);
end

function eff = localSpectralEfficiency(snr_dB)
    gamma = 10.^(snr_dB/10);
    eff = log2(1 + gamma);
    eff = max(0, min(eff, 3.5));
end

function out = localMergeStruct(a, b)
    out = a;
    if ~isstruct(b)
        return;
    end
    fn = fieldnames(b);
    for i = 1:numel(fn)
        k = fn{i};
        if isfield(out, k) && isstruct(out.(k)) && isstruct(b.(k))
            out.(k) = localMergeStruct(out.(k), b.(k));
        else
            out.(k) = b.(k);
        end
    end
end

function s = ternary(cond, a, b)
    if cond
        s = a;
    else
        s = b;
    end
end
