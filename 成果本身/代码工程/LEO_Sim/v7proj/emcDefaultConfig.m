function cfg = emcDefaultConfig()
%EMCDEFAULTCONFIG Default engineering configuration for V7.
%   All values are chosen so that:
%   1) the model can run directly without any user input;
%   2) the user can later overwrite receiver-port / antenna / conversion data;
%   3) the current model continues to support the forward-design study.

    cfg = struct();

    %% General
    cfg.General = struct();
    cfg.General.ProjectName = '车载低轨卫星通信系统EMC性能正向设计技术研究 | V7.0';
    cfg.General.RngSeed = 7;
    cfg.General.StartupMode = 'default';

    %% Time / scenario
    cfg.Time = struct();
    cfg.Time.Epoch = datetime(2026,1,23,0,0,0,'TimeZone','UTC');
    cfg.Time.SampleTime_s = 10;
    cfg.Time.SimDuration_s = 0;  % 0 -> one orbital period

    %% Constellation
    cfg.Constellation = struct();
    cfg.Constellation.Altitude_m = 1200e3;
    cfg.Constellation.Eccentricity = 0.0;
    cfg.Constellation.Inclination_deg = 53;
    cfg.Constellation.NumPlanes = 12;
    cfg.Constellation.SatsPerPlane = 8;
    cfg.Constellation.FPhasing = 1;
    cfg.Constellation.ReuseK = 4;
    cfg.Constellation.ElMask_deg = 5;

    %% Ground
    cfg.Ground = struct();
    cfg.Ground.UserLat = 36.06;
    cfg.Ground.UserLon = 120.38;
    cfg.Ground.GWLat = 39.90;
    cfg.Ground.GWLon = 116.40;

    %% Downlink: satellite -> vehicle terminal
    cfg.Downlink = struct();
    cfg.Downlink.Fc_Hz = 1.5e9;
    cfg.Downlink.BW_Hz = 20e6;
    cfg.Downlink.Noise_dBm = -110;
    cfg.Downlink.TxEIRP_S_dBm = 55;
    cfg.Downlink.TxEIRP_I_dBm = 55;
    cfg.Downlink.RxGain_dB = 35;
    cfg.Downlink.InterfPenalty_dB = 10;

    %% Uplink: vehicle terminal -> satellite
    cfg.Uplink = struct();
    cfg.Uplink.Fc_Hz = 1.6e9;
    cfg.Uplink.BW_Hz = 20e6;
    cfg.Uplink.Noise_dBm = -110;
    cfg.Uplink.TxEIRP_S_dBm = 40;
    cfg.Uplink.TxEIRP_I_dBm = 40;
    cfg.Uplink.RxGain_dB = 25;
    cfg.Uplink.InterfPenalty_dB = 10;
    cfg.Uplink.CCI_Mode = 'fixed';          % none | fixed | reuseProxy
    cfg.Uplink.CCI_Fixed_dBm = -118;        % aggregate co-channel interference at satellite Rx port
    cfg.Uplink.JamProxyMode = 'reuseDL';    % off | fixed | reuseDL
    cfg.Uplink.JamProxyFixed_dBm = -115;    % used when JamProxyMode='fixed'
    cfg.Uplink.JamReuseBias_dB = -3;        % UL jammer proxy = DL jammer proxy + bias

    %% Jammer / EMC disturbance envelope
    cfg.Jammer = struct();
    cfg.Jammer.NumJammers = 6;
    cfg.Jammer.TxEIRP_Base_dBm = 88;
    cfg.Jammer.RxGain_dB = 10;
    cfg.Jammer.MainLobe_deg = 3;
    cfg.Jammer.SideLobe_deg = 20;
    cfg.Jammer.MainGain_dB = 0;
    cfg.Jammer.SideGain_dB = -8;
    cfg.Jammer.FloorGain_dB = -12;

    %% Anti-jam
    cfg.AntiJam = struct();
    cfg.AntiJam.Delay_s = 30;
    cfg.AntiJam.NullDepth_dB = 40;

    %% Worst-case search
    cfg.WorstCase = struct();
    cfg.WorstCase.Enable = true;
    cfg.WorstCase.Target = 'e2e';           % downlink | uplink | e2e
    cfg.WorstCase.GAN_seqLen = 128;
    cfg.WorstCase.GAN_zDim = 16;
    cfg.WorstCase.GAN_cDim = 2;
    cfg.WorstCase.GAN_infoLambda = 1.0;
    cfg.WorstCase.GAN_trainIters = 250;
    cfg.WorstCase.GAN_modelFile = 'InfoGAN_Jammer_R2021a.mat';
    cfg.WorstCase.GA_PopSize = 16;
    cfg.WorstCase.GA_Generations = 10;
    cfg.WorstCase.JamScale_lb_dB = 0;
    cfg.WorstCase.JamScale_ub_dB = 35;
    cfg.WorstCase.z_lb = -2;
    cfg.WorstCase.z_ub = 2;
    cfg.WorstCase.W_outage = 300;
    cfg.WorstCase.W_bler = 80;
    cfg.WorstCase.W_energy = 80;
    cfg.WorstCase.OutageThr_Mbps = 20;

    %% Display / UI
    cfg.Display = struct();
    cfg.Display.SINR_YLIM = [-20 50];
    cfg.Display.THR_YLIM = [0 200];
    cfg.Display.SINR_YLIM_UL = [-20 50];
    cfg.Display.THR_YLIM_UL = [0 200];
    cfg.Display.E2E_YLIM = [0 200];
    cfg.Display.ViewerSpeed = 60;

    %% Interference classifier
    cfg.Classifier = struct();
    cfg.Classifier.Enable = true;
    cfg.Classifier.ModelFile = 'lenet_stft_model_r2021a.mat';
    cfg.Classifier.DatasetRoot = 'dataset_stft_r2021a';
    cfg.Classifier.TrainIfMissing = true;
    cfg.Classifier.ForceRegenDataset = false;
    cfg.Classifier.ExportImages = true;

    %% Requirements / compliance
    cfg.Requirements = struct();
    cfg.Requirements.MinSINR_dB = 1;
    cfg.Requirements.MinThr_Mbps = 20;
    cfg.Requirements.RxSensitivity_dBm = -120;

    % Route-test signal strength is often a converted front-end value rather than raw air-interface Rx power.
    % Default: disabled until the partner provides AF / cable loss / conversion coefficients.
    cfg.Requirements.EnableConvertedStrengthCheck = false;
    cfg.Requirements.MinSignalStrength_dBm = 1;
    cfg.Requirements.SignalStrengthOffset_dB = 0;

    cfg.Requirements.MaxDopplerRate_Hzps = 300;
    cfg.Requirements.MinVoiceRate_kbps = 2.4;
    cfg.Requirements.MinBroadband_Mbps = 20;
    cfg.Requirements.RouteSNR_Min_dB = 1;
    cfg.Requirements.RouteStrength_Min_dBm = 1;

    cfg.Requirements.KuEIRP_Min_dBw = 1.0;
    cfg.Requirements.KuGT_Min_dBperK = -27.5;
    cfg.Requirements.KuEIRP_Current_dBw = 1.2;
    cfg.Requirements.KuGT_Current_dBperK = -27.0;

    cfg.Requirements.JA3700_TargetLevel = 5;
    cfg.Requirements.JA3700_CurrentLevel = 5;

    %% Measurement / conversion placeholders for later docking
    cfg.Measurement = struct();
    cfg.Measurement.EnableFieldStrengthConversion = false;
    cfg.Measurement.AntennaFactor_dBpm = 0;
    cfg.Measurement.CableLoss_dB = 0;
    cfg.Measurement.PreAmpGain_dB = 0;
    cfg.Measurement.OtherOffset_dB = 0;
    cfg.Measurement.RefImpedance_Ohm = 50;

    %% Outputs
    cfg.Output = struct();
    cfg.Output.Enable3DViewer = true;
    cfg.Output.ExportFolder = 'outputs_v7';
    cfg.Output.AutoSaveResolvedConfig = true;
    cfg.Output.AutoSaveResultMat = true;
    cfg.Output.ResolvedConfigFile = 'cfg_resolved_v7.mat';
    cfg.Output.ResultMatFile = 'result_v7.mat';
    cfg.Output.SummaryTextFile = 'summary_v7.txt';
end
