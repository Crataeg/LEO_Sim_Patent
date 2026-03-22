function result = LEO_StarNet_EMC_V7_0_Engineering(varargin)
%LEO_STARNET_EMC_V7_0_ENGINEERING
% V7 工程交付版（MATLAB R2021a）
%
% 修正版说明：
%   1) Dashboard 右下角恢复为二维 Plane-Slot 星座图
%   2) 去掉星座图下方的链路名称显示
%   3) satelliteScenarioViewer 保持正常卫星视图，不动态改链路
%   4) Sky View 修复名称残留、错位、重影问题
%   5) Sky View 底部两行文字栏显示当前链路
%   6) 当前链路即使部分不在天空中，也会做地平线裁剪显示
%   7) GA 使用更常见的停止条件
%
% 调用方式：
%   LEO_StarNet_EMC_V7_0_Engineering
%   LEO_StarNet_EMC_V7_0_Engineering(cfgStruct)
%   LEO_StarNet_EMC_V7_0_Engineering('cfg_v7.mat')

    close all;
    clc;

    thisDir = fileparts(mfilename('fullpath'));
    if ~isempty(thisDir)
        addpath(thisDir);
    end

    cfgDefault = emcDefaultConfig();
    cfg = resolveInputCfg(cfgDefault, varargin{:});
    rng(cfg.General.RngSeed);

    if ~exist(cfg.Output.ExportFolder, 'dir')
        mkdir(cfg.Output.ExportFolder);
    end

    if cfg.Output.AutoSaveResolvedConfig
        saveCfg = cfg; %#ok<NASGU>
        cfg = saveCfg; %#ok<NASGU>
        save(fullfile(cfg.Output.ExportFolder, cfg.Output.ResolvedConfigFile), 'cfg');
        cfg = saveCfg;
    end

    fprintf('============================================================\n');
    fprintf('  LEO StarNet EMC V7.0 | Engineering Delivery Version\n');
    fprintf('============================================================\n');
    fprintf('Startup Mode : %s\n', cfg.General.StartupMode);
    fprintf('Project      : %s\n', cfg.General.ProjectName);
    fprintf('ExportFolder : %s\n', cfg.Output.ExportFolder);

    %% =========================
    % PART 0: basic parameters
    % ==========================
    Epoch = cfg.Time.Epoch;
    sample_time = cfg.Time.SampleTime_s;

    Re = 6371e3;
    mu = 3.986004418e14;

    h = cfg.Constellation.Altitude_m;
    a = Re + h;
    ecc = cfg.Constellation.Eccentricity;
    incDeg = cfg.Constellation.Inclination_deg;
    numPlanes = cfg.Constellation.NumPlanes;
    satsPerPlane = cfg.Constellation.SatsPerPlane;
    F_phasing = cfg.Constellation.FPhasing;
    Nsat = numPlanes * satsPerPlane;
    reuseK = cfg.Constellation.ReuseK;
    elMaskDeg = cfg.Constellation.ElMask_deg;

    T_orbit = 2*pi*sqrt(a^3/mu);
    sim_start = Epoch;
    if cfg.Time.SimDuration_s > 0
        sim_stop = sim_start + seconds(cfg.Time.SimDuration_s);
    else
        sim_stop = sim_start + seconds(T_orbit);
    end

    %% =========================
    % PART 1: scenario and constellation
    % ==========================
    fprintf('Step 1: 构建 satelliteScenario...\n');
    sc = satelliteScenario(sim_start, sim_stop, sample_time);

    gsUser = groundStation(sc, cfg.Ground.UserLat, cfg.Ground.UserLon, 'Name', 'Vehicle_User');
    gsGW   = groundStation(sc, cfg.Ground.GWLat, cfg.Ground.GWLon, 'Name', 'Gateway');

    fprintf('Step 2: 生成星座：%d x %d = %d satellites ...\n', numPlanes, satsPerPlane, Nsat);
    satConst = cell(1, Nsat);
    satName  = strings(1, Nsat);
    satPlane = zeros(1, Nsat);
    satSlot  = zeros(1, Nsat);

    idx = 0;
    for p = 1:numPlanes
        raan = (p-1) * (360/numPlanes);
        for s = 1:satsPerPlane
            idx = idx + 1;
            ta = (s-1)*(360/satsPerPlane) + (p-1)*F_phasing*(360/Nsat);
            nm = sprintf('SAT_P%02d_S%02d', p, s);
            satConst{idx} = satellite(sc, a, ecc, incDeg, raan, 0, ta, 'Name', nm);
            satName(idx) = nm;
            satPlane(idx) = p;
            satSlot(idx) = s;
        end
    end

    numJam = cfg.Jammer.NumJammers;
    satJam = cell(1, numJam);
    for j = 1:numJam
        raanJ = (j-1)*(360/max(1,numJam));
        taJ = 180 + 10*j;
        satJam{j} = satellite(sc, a, ecc, incDeg, raanJ, 0, taJ, 'Name', sprintf('JAMMER_%d', j));
        try
            satJam{j}.MarkerColor = [0.95 0.15 0.15];
        catch
        end
    end

    %% =========================
    % PART 2: geometry precompute
    % ==========================
    fprintf('Step 3: 预计算几何...\n');
    timeVec = 0:sample_time:seconds(sim_stop - sim_start);
    numSteps_guess = numel(timeVec);

    [~, el0, ~] = aer(gsUser, satConst{1});
    L = min(numSteps_guess, numel(el0));
    timeVec = timeVec(1:L);
    numSteps = L;
    t_axis_min = timeVec/60;

    azU = nan(numSteps, Nsat);
    elU = nan(numSteps, Nsat);
    rU  = nan(numSteps, Nsat);
    for i = 1:Nsat
        [az, el, r] = aer(gsUser, satConst{i});
        Li = min(numSteps, numel(el));
        azU(1:Li,i) = az(1:Li);
        elU(1:Li,i) = el(1:Li);
        rU(1:Li,i)  = r(1:Li);
    end

    azG = nan(numSteps, Nsat);
    elG = nan(numSteps, Nsat);
    rG  = nan(numSteps, Nsat);
    for i = 1:Nsat
        [az, el, r] = aer(gsGW, satConst{i});
        Li = min(numSteps, numel(el));
        azG(1:Li,i) = az(1:Li);
        elG(1:Li,i) = el(1:Li);
        rG(1:Li,i)  = r(1:Li);
    end

    azJ = nan(numSteps, numJam);
    elJ = nan(numSteps, numJam);
    rJ  = nan(numSteps, numJam);
    for j = 1:numJam
        [az, el, r] = aer(gsUser, satJam{j});
        Lj = min(numSteps, numel(el));
        azJ(1:Lj,j) = az(1:Lj);
        elJ(1:Lj,j) = el(1:Lj);
        rJ(1:Lj,j)  = r(1:Lj);
    end

    rrU = nan(numSteps, Nsat);
    for i = 1:Nsat
        ri = rU(:,i);
        dri = [diff(ri)/sample_time; 0];
        rrU(:,i) = movmean(dri, 5);
    end

    %% =========================
    % PART 3: reuse plan and ISL graph
    % ==========================
    fprintf('Step 4: 构建频率复用与简化 ISL 图...\n');
    satChan = zeros(1, Nsat);
    for i = 1:Nsat
        satChan(i) = mod((satPlane(i)-1) + (satSlot(i)-1), reuseK) + 1;
    end

    toIdx = @(p,s) (p-1)*satsPerPlane + s;
    sList = [];
    tList = [];
    wList = [];
    d_inplane = 2*a*sin(pi/satsPerPlane);
    d_cross   = 2*a*sin(pi/numPlanes);

    for p = 1:numPlanes
        for s = 1:satsPerPlane
            u = toIdx(p,s);
            s_fwd = s+1; if s_fwd>satsPerPlane, s_fwd = 1; end
            s_bwd = s-1; if s_bwd<1, s_bwd = satsPerPlane; end
            v1 = toIdx(p,s_fwd);
            v2 = toIdx(p,s_bwd);

            p_r = p+1; if p_r>numPlanes, p_r = 1; end
            p_l = p-1; if p_l<1, p_l = numPlanes; end
            v3 = toIdx(p_r,s);
            v4 = toIdx(p_l,s);

            sList = [sList u u u u]; %#ok<AGROW>
            tList = [tList v1 v2 v3 v4]; %#ok<AGROW>
            wList = [wList d_inplane d_inplane d_cross d_cross]; %#ok<AGROW>
        end
    end
    Gisl = graph(sList, tList, wList, Nsat);

    %% =========================
    % PART 4: link power precompute (DL + UL)
    % ==========================
    fprintf('Step 5: 预计算功率项（DL + UL）...\n');
    visU_all = elU > elMaskDeg;
    visG_all = elG > elMaskDeg;
    visJ_all = elJ > elMaskDeg;

    [pS_DL_mW, pIself_DL_mW, totCCI_DL] = emcPrecomputeLinkPower( ...
        rU, visU_all, cfg.Downlink.Fc_Hz, cfg.Downlink.TxEIRP_S_dBm, cfg.Downlink.TxEIRP_I_dBm, ...
        cfg.Downlink.RxGain_dB, cfg.Downlink.InterfPenalty_dB, satChan, reuseK);

    [pS_UL_mW, pIself_UL_proxy_mW, totCCI_UL_proxy] = emcPrecomputeLinkPower( ...
        rU, visU_all, cfg.Uplink.Fc_Hz, cfg.Uplink.TxEIRP_S_dBm, cfg.Uplink.TxEIRP_I_dBm, ...
        cfg.Uplink.RxGain_dB, cfg.Uplink.InterfPenalty_dB, satChan, reuseK);

    switch lower(strtrim(cfg.Uplink.CCI_Mode))
        case 'none'
            pIself_UL_mW = zeros(size(pIself_UL_proxy_mW));
            totCCI_UL = zeros(size(totCCI_UL_proxy));
        case 'reuseproxy'
            pIself_UL_mW = pIself_UL_proxy_mW;
            totCCI_UL = totCCI_UL_proxy;
        otherwise
            pIself_UL_mW = zeros(size(pIself_UL_proxy_mW));
            totCCI_UL = repmat(10.^(cfg.Uplink.CCI_Fixed_dBm/10), numSteps, reuseK);
    end

    pJ_base_mW = zeros(numSteps, numJam);
    for j = 1:numJam
        PLj = fspl_dB_vec(rJ(:,j), cfg.Downlink.Fc_Hz/1e6);
        PJdBm = cfg.Jammer.TxEIRP_Base_dBm - PLj + cfg.Jammer.RxGain_dB;
        tmp = 10.^(PJdBm/10);
        tmp(~visJ_all(:,j)) = 0;
        pJ_base_mW(:,j) = tmp;
    end

    pJsum_base_mW_DL = zeros(numSteps, Nsat);
    for j = 1:numJam
        azj = azJ(:,j);
        elj = elJ(:,j);
        for i = 1:Nsat
            daz = wrap180_vec(azU(:,i) - azj);
            del = elU(:,i) - elj;
            offAxis = sqrt(daz.^2 + del.^2);
            active = visJ_all(:,j) & visU_all(:,i);
            if any(active)
                patLin = jammerPatternLin(offAxis(active), ...
                    cfg.Jammer.MainLobe_deg, cfg.Jammer.SideLobe_deg, ...
                    cfg.Jammer.MainGain_dB, cfg.Jammer.SideGain_dB, cfg.Jammer.FloorGain_dB);
                pJsum_base_mW_DL(active,i) = pJsum_base_mW_DL(active,i) + pJ_base_mW(active,j) .* patLin;
            end
        end
    end

    pJsum_base_mW_UL = emcBuildUplinkJammerBase( ...
        visU_all, pJsum_base_mW_DL, cfg.Uplink.JamProxyMode, cfg.Uplink.JamProxyFixed_dBm, cfg.Uplink.JamReuseBias_dB);

    p_n_DL = 10.^(cfg.Downlink.Noise_dBm/10);
    p_n_UL = 10.^(cfg.Uplink.Noise_dBm/10);

    modelDL = emcBuildLinkModel('DL', numSteps, sample_time, ...
        visU_all, visG_all, azU, elU, rU, rrU, elG, rG, ...
        pS_DL_mW, totCCI_DL, satChan, pIself_DL_mW, pJsum_base_mW_DL, ...
        cfg.Jammer.TxEIRP_Base_dBm, cfg.AntiJam.Delay_s, cfg.AntiJam.NullDepth_dB, ...
        cfg.Downlink.Fc_Hz, cfg.Downlink.BW_Hz, p_n_DL, Gisl, cfg.WorstCase.OutageThr_Mbps, cfg.Requirements.SignalStrengthOffset_dB);

    modelUL = emcBuildLinkModel('UL', numSteps, sample_time, ...
        visU_all, visG_all, azU, elU, rU, rrU, elG, rG, ...
        pS_UL_mW, totCCI_UL, satChan, pIself_UL_mW, pJsum_base_mW_UL, ...
        cfg.Jammer.TxEIRP_Base_dBm, cfg.AntiJam.Delay_s, cfg.AntiJam.NullDepth_dB, ...
        cfg.Uplink.Fc_Hz, cfg.Uplink.BW_Hz, p_n_UL, Gisl, cfg.WorstCase.OutageThr_Mbps, cfg.Requirements.SignalStrengthOffset_dB);

    %% =========================
    % PART 5: baseline and worst-case search
    % ==========================
    fprintf('Step 6: 基线仿真 + 最劣搜索...\n');
    jamAgg_zero = zeros(numSteps,1);
    TxP_J_off_dBm = -200;
    JamScale0_dB = 0;

    simDL_Base = simulateStarNetV7(modelDL, TxP_J_off_dBm, jamAgg_zero, JamScale0_dB, struct());
    simUL_Base = simulateStarNetV7(modelUL, TxP_J_off_dBm, jamAgg_zero, JamScale0_dB, ...
        struct('ForceServing', simDL_Base.Serving, 'ForceGateway', simDL_Base.Gateway));
    simE2E_Base = emcCombineE2E(simDL_Base, simUL_Base, cfg.Requirements.MinThr_Mbps);

    cBest = 0.5*ones(1, cfg.WorstCase.GAN_cDim);
    if cfg.WorstCase.Enable
        [netG, ~, ~] = trainOrLoadJammerGAN( ...
            cfg.WorstCase.GAN_seqLen, cfg.WorstCase.GAN_zDim, cfg.WorstCase.GAN_cDim, ...
            cfg.WorstCase.GAN_trainIters, cfg.WorstCase.GAN_modelFile, cfg.WorstCase.GAN_infoLambda);

        nvars = cfg.WorstCase.GAN_zDim + cfg.WorstCase.GAN_cDim + 1;
        lb = [cfg.WorstCase.z_lb*ones(cfg.WorstCase.GAN_zDim,1); zeros(cfg.WorstCase.GAN_cDim,1); cfg.WorstCase.JamScale_lb_dB];
        ub = [cfg.WorstCase.z_ub*ones(cfg.WorstCase.GAN_zDim,1); ones(cfg.WorstCase.GAN_cDim,1); cfg.WorstCase.JamScale_ub_dB];

        obj = @(x) worstCaseObjectiveV7( ...
            x, netG, cfg.WorstCase.GAN_seqLen, cfg.WorstCase.GAN_zDim, cfg.WorstCase.GAN_cDim, ...
            modelDL, modelUL, cfg.WorstCase.Target, ...
            cfg.WorstCase.W_outage, cfg.WorstCase.W_bler, cfg.WorstCase.W_energy);

        gaMaxGen = 60;
        if isfield(cfg.WorstCase, 'GA_Generations')
            gaMaxGen = max(cfg.WorstCase.GA_Generations, 60);
        end

        gaMaxStall = 12;
        if isfield(cfg.WorstCase, 'GA_MaxStallGenerations')
            gaMaxStall = cfg.WorstCase.GA_MaxStallGenerations;
        end

        gaFuncTol = 1e-4;
        if isfield(cfg.WorstCase, 'GA_FunctionTolerance')
            gaFuncTol = cfg.WorstCase.GA_FunctionTolerance;
        end

        gaOpts = optimoptions('ga', ...
            'PopulationSize', cfg.WorstCase.GA_PopSize, ...
            'MaxGenerations', gaMaxGen, ...
            'MaxStallGenerations', gaMaxStall, ...
            'FunctionTolerance', gaFuncTol, ...
            'Display', 'iter', ...
            'UseParallel', false);

        try
            [xBest, ~] = ga(obj, nvars, [], [], [], [], lb, ub, [], gaOpts);
        catch ME
            fprintf('[警告] GA 失败：%s\n', ME.message);
            xBest = [zeros(cfg.WorstCase.GAN_zDim,1); 0.5*ones(cfg.WorstCase.GAN_cDim,1); 20];
        end

        zBest = xBest(1:cfg.WorstCase.GAN_zDim);
        cBest = xBest(cfg.WorstCase.GAN_zDim + (1:cfg.WorstCase.GAN_cDim));
        JamScaleBest_dB = xBest(end);
        jamAggWorst = genJamAggFromG(netG, zBest, cBest, cfg.WorstCase.GAN_seqLen, numSteps);
    else
        JamScaleBest_dB = 0;
        jamAggWorst = zeros(numSteps,1);
    end

    simDL_Worst = simulateStarNetV7(modelDL, cfg.Jammer.TxEIRP_Base_dBm, jamAggWorst, JamScaleBest_dB, struct());
    simUL_Worst = simulateStarNetV7(modelUL, cfg.Jammer.TxEIRP_Base_dBm, jamAggWorst, JamScaleBest_dB, ...
        struct('ForceServing', simDL_Worst.Serving, 'ForceGateway', simDL_Worst.Gateway));
    simE2E_Worst = emcCombineE2E(simDL_Worst, simUL_Worst, cfg.Requirements.MinThr_Mbps);

    simDL_Worst.JamInfoCode = cBest(:).';

    fprintf('\n===================== SUMMARY =====================\n');
    fprintf('DL Base   : meanThr=%.2f Mbps, outage=%.2f %%\n', simDL_Base.meanThr, 100*simDL_Base.outageFrac);
    fprintf('DL Worst  : meanThr=%.2f Mbps, outage=%.2f %%\n', simDL_Worst.meanThr, 100*simDL_Worst.outageFrac);
    fprintf('UL Base   : meanThr=%.2f Mbps, outage=%.2f %%\n', simUL_Base.meanThr, 100*simUL_Base.outageFrac);
    fprintf('UL Worst  : meanThr=%.2f Mbps, outage=%.2f %%\n', simUL_Worst.meanThr, 100*simUL_Worst.outageFrac);
    fprintf('E2E Base  : meanThr=%.2f Mbps, outage=%.2f %%\n', simE2E_Base.meanThr, 100*simE2E_Base.outageFrac);
    fprintf('E2E Worst : meanThr=%.2f Mbps, outage=%.2f %% | JamScale=%.1f dB | Target=%s\n', ...
        simE2E_Worst.meanThr, 100*simE2E_Worst.outageFrac, JamScaleBest_dB, cfg.WorstCase.Target);
    fprintf('===================================================\n\n');

    %% =========================
    % PART 5.5: interference classification (DL based)
    % ==========================
    intfClasses = [];
    if cfg.Classifier.Enable
        fprintf('Step 6.5: STFT+LeNet 干扰分类（基于 DL 最劣场景）...\n');
        try
            IntfExportDir = fullfile(cfg.Classifier.DatasetRoot, '_exports');
            if cfg.Classifier.ForceRegenDataset && exist(cfg.Classifier.DatasetRoot, 'dir')
                try, rmdir(cfg.Classifier.DatasetRoot, 's'); catch, end
            end

            [netIntf, intfClasses] = getOrTrainLeNetSTFT( ...
                cfg.Classifier.ModelFile, cfg.Classifier.DatasetRoot, cfg.Classifier.TrainIfMissing);

            if cfg.Classifier.ExportImages
                if ~exist(IntfExportDir, 'dir'), mkdir(IntfExportDir); end
                exportDatasetMontage(cfg.Classifier.DatasetRoot, IntfExportDir);
                exportTestConfusion(netIntf, cfg.Classifier.DatasetRoot, IntfExportDir, intfClasses);
            end

            SamplerCfg = defaultPowerAlignedSamplerCfg();
            SamplerCfg.UsePostAJPower = true;
            SamplerCfg.Ns = 2048;
            if isfield(simDL_Worst, 'JamInfoCode')
                SamplerCfg.InfoCode = simDL_Worst.JamInfoCode;
            end

            [intfTrue, intfPred, intfScore, snapInfo] = classifyInterferenceTimeline_powerSampler( ...
                netIntf, intfClasses, numSteps, sample_time, simDL_Worst, p_n_DL, SamplerCfg);

            simDL_Worst.IntfTrue = intfTrue;
            simDL_Worst.IntfPred = intfPred;
            simDL_Worst.IntfScore = intfScore;

            if cfg.Classifier.ExportImages
                exportSimKeyframeSTFT(snapInfo, IntfExportDir);
            end
        catch ME
            fprintf('[警告] STFT+LeNet 分类模块跳过：%s\n', ME.message);
        end
    end

    %% =========================
    % PART 6: normal 3D Viewer
    % ==========================
    fprintf('Step 7: 恢复正常 3D Viewer 视图...\n');
    acUser = gobjects(0,1);
    acGW   = gobjects(0,1);
    acISL  = gobjects(0,1);
    acJamUser = gobjects(0,1);

    islMap = zeros(Nsat, Nsat, 'uint16');
    islEnds = Gisl.Edges.EndNodes;
    nISL = size(islEnds,1);
    for e = 1:nISL
        u = islEnds(e,1);
        v2 = islEnds(e,2);
        islMap(u,v2) = e;
        islMap(v2,u) = e;
    end

    %% =========================
    % PART 7: dashboard
    % ==========================
    fprintf('Step 8: 构建 Dashboard ...\n');
    dash = uifigure('Name', 'LEO StarNet EMC V7 | Engineering Delivery Dashboard', ...
        'Color', 'w', 'Position', [40 30 1680 940]);

    gl = uigridlayout(dash, [2 2]);
    gl.RowHeight = {38, '1x'};
    gl.ColumnWidth = {'1x', 430};
    gl.Padding = [10 10 10 10];
    gl.RowSpacing = 8;
    gl.ColumnSpacing = 10;

    titleLbl = uilabel(gl, 'Text', sprintf('LEO StarNet EMC V7 | %d sats | %.1f min | WorstTarget=%s', ...
        Nsat, seconds(sim_stop-sim_start)/60, cfg.WorstCase.Target), ...
        'FontSize', 16, 'FontWeight', 'bold');
    titleLbl.Layout.Row = 1;
    titleLbl.Layout.Column = [1 2];

    tabs = uitabgroup(gl);
    tabs.Layout.Row = 2;
    tabs.Layout.Column = 1;

    % ---- Overview tab ----
    tabOv = uitab(tabs, 'Title', '概览');
    glOv = uigridlayout(tabOv, [2 2]);
    glOv.RowHeight = {'1x','1x'};
    glOv.ColumnWidth = {'1x','1x'};

    axE2E   = uiaxes(glOv); axE2E.Layout.Row = 1; axE2E.Layout.Column = 1;
    axOvThr = uiaxes(glOv); axOvThr.Layout.Row = 1; axOvThr.Layout.Column = 2;
    axOvJam = uiaxes(glOv); axOvJam.Layout.Row = 2; axOvJam.Layout.Column = 1;
    axOvDop = uiaxes(glOv); axOvDop.Layout.Row = 2; axOvDop.Layout.Column = 2;

    plot(axE2E, t_axis_min, simE2E_Base.THR, '--', 'LineWidth', 1.3); hold(axE2E, 'on');
    plot(axE2E, t_axis_min, simE2E_Worst.THR, '-', 'LineWidth', 2.0);
    grid(axE2E, 'on'); title(axE2E, 'E2E Throughput (min{DL,UL})');
    ylabel(axE2E, 'Mbps'); xlabel(axE2E, 'Time (min)');
    xlim(axE2E, [0 max(t_axis_min)]); ylim(axE2E, cfg.Display.E2E_YLIM);
    yline(axE2E, cfg.Requirements.MinThr_Mbps, 'r--', 'Req');
    curE2E = line(axE2E, [0 0], cfg.Display.E2E_YLIM, 'Color', 'k', 'LineWidth', 1.8);

    plot(axOvThr, t_axis_min, simDL_Worst.THR, 'LineWidth', 1.7); hold(axOvThr, 'on');
    plot(axOvThr, t_axis_min, simUL_Worst.THR, 'LineWidth', 1.7);
    grid(axOvThr, 'on'); title(axOvThr, 'Worst-case Throughput | DL vs UL');
    ylabel(axOvThr, 'Mbps'); xlabel(axOvThr, 'Time (min)');
    xlim(axOvThr, [0 max(t_axis_min)]); ylim(axOvThr, [0 max(cfg.Display.THR_YLIM(2), cfg.Display.THR_YLIM_UL(2))]);
    curOvThr = line(axOvThr, [0 0], ylim(axOvThr), 'Color', 'k', 'LineWidth', 1.8);
    legend(axOvThr, {'DL Worst','UL Worst'}, 'Location', 'best');

    plot(axOvJam, t_axis_min, jamAggWorst, 'LineWidth', 2.0);
    grid(axOvJam, 'on'); title(axOvJam, sprintf('Worst-case Jam Envelope | JamScale=%.1f dB', JamScaleBest_dB));
    ylabel(axOvJam, 'jamAgg'); xlabel(axOvJam, 'Time (min)');
    xlim(axOvJam, [0 max(t_axis_min)]); ylim(axOvJam, [0 1.05]);
    curOvJam = line(axOvJam, [0 0], [0 1.05], 'Color', 'k', 'LineWidth', 1.8);

    plot(axOvDop, t_axis_min, simDL_Worst.DopRate_Hzps, 'LineWidth', 1.7); hold(axOvDop, 'on');
    plot(axOvDop, t_axis_min, simUL_Worst.DopRate_Hzps, 'LineWidth', 1.7);
    grid(axOvDop, 'on'); title(axOvDop, 'Doppler Rate | DL vs UL');
    ylabel(axOvDop, 'Hz/s'); xlabel(axOvDop, 'Time (min)');
    xlim(axOvDop, [0 max(t_axis_min)]);
    yline(axOvDop, cfg.Requirements.MaxDopplerRate_Hzps, 'r--', 'Req');
    curOvDop = line(axOvDop, [0 0], ylim(axOvDop), 'Color', 'k', 'LineWidth', 1.8);
    legend(axOvDop, {'DL DopRate','UL DopRate'}, 'Location', 'best');

    % ---- DL tab ----
    tabDL = uitab(tabs, 'Title', '下行链路 DL');
    glDL = uigridlayout(tabDL, [2 2]);
    glDL.RowHeight = {'1x','1x'}; glDL.ColumnWidth = {'1x','1x'};
    axDLSINR = uiaxes(glDL); axDLSINR.Layout.Row = 1; axDLSINR.Layout.Column = 1;
    axDLBER  = uiaxes(glDL); axDLBER.Layout.Row = 1; axDLBER.Layout.Column = 2;
    axDLTHR  = uiaxes(glDL); axDLTHR.Layout.Row = 2; axDLTHR.Layout.Column = 1;
    axDLDOP  = uiaxes(glDL); axDLDOP.Layout.Row = 2; axDLDOP.Layout.Column = 2;

    hDLSINR1 = plot(axDLSINR, t_axis_min, simDL_Base.SINR, '--', 'LineWidth', 1.3); hold(axDLSINR, 'on');
    hDLSINR2 = plot(axDLSINR, t_axis_min, simDL_Worst.SINR, '-', 'LineWidth', 2.0);
    grid(axDLSINR, 'on'); title(axDLSINR, 'DL SINR'); ylabel(axDLSINR, 'dB'); xlabel(axDLSINR, 'Time (min)');
    xlim(axDLSINR, [0 max(t_axis_min)]); ylim(axDLSINR, cfg.Display.SINR_YLIM); yline(axDLSINR, 0, 'r--', 'Disconnect');
    shadeTagRegions(axDLSINR, t_axis_min, simDL_Worst.Event, "JAMMING!!!", cfg.Display.SINR_YLIM, [1 0 0], 'Jamming', -10);
    shadeTagRegions(axDLSINR, t_axis_min, simDL_Worst.Event, "Protected", cfg.Display.SINR_YLIM, [0 1 0], 'Protected', 40);
    shadeTagRegions(axDLSINR, t_axis_min, simDL_Worst.Event, "CoChannel", cfg.Display.SINR_YLIM, [1 0.5 0], 'Co-Channel', 20);
    hDLJam = patch(axDLSINR, nan, nan, [1 0 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    hDLProt = patch(axDLSINR, nan, nan, [0 1 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    hDLCo = patch(axDLSINR, nan, nan, [1 0.5 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    legend(axDLSINR, [hDLSINR1 hDLSINR2 hDLJam hDLProt hDLCo], {'DL Base','DL Worst','Jamming','Protected','Co-Channel'}, 'Location', 'northeast');
    curDLSINR = line(axDLSINR, [0 0], cfg.Display.SINR_YLIM, 'Color', 'k', 'LineWidth', 1.8);
    dotDLSINR = plot(axDLSINR, 0, cfg.Display.SINR_YLIM(1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    semilogy(axDLBER, t_axis_min, simDL_Base.BER, '--', 'LineWidth', 1.3); hold(axDLBER, 'on');
    semilogy(axDLBER, t_axis_min, simDL_Worst.BER, '-', 'LineWidth', 2.0);
    grid(axDLBER, 'on'); title(axDLBER, 'DL BER (log)'); ylabel(axDLBER, 'log'); xlabel(axDLBER, 'Time (min)');
    xlim(axDLBER, [0 max(t_axis_min)]); ylim(axDLBER, [1e-9 1]);
    curDLBER = line(axDLBER, [0 0], [1e-9 1], 'Color', 'k', 'LineWidth', 1.8);
    dotDLBER = plot(axDLBER, 0, 1, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    plot(axDLTHR, t_axis_min, simDL_Base.THR, '--', 'LineWidth', 1.3); hold(axDLTHR, 'on');
    plot(axDLTHR, t_axis_min, simDL_Worst.THR, '-', 'LineWidth', 2.0);
    grid(axDLTHR, 'on'); title(axDLTHR, 'DL Throughput'); ylabel(axDLTHR, 'Mbps'); xlabel(axDLTHR, 'Time (min)');
    xlim(axDLTHR, [0 max(t_axis_min)]); ylim(axDLTHR, cfg.Display.THR_YLIM); yline(axDLTHR, cfg.Requirements.MinThr_Mbps, 'r--', 'Req');
    curDLTHR = line(axDLTHR, [0 0], cfg.Display.THR_YLIM, 'Color', 'k', 'LineWidth', 1.8);
    dotDLTHR = plot(axDLTHR, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    plot(axDLDOP, t_axis_min, simDL_Worst.DOPkHz, 'LineWidth', 2.0);
    grid(axDLDOP, 'on'); title(axDLDOP, 'DL Doppler'); ylabel(axDLDOP, 'kHz'); xlabel(axDLDOP, 'Time (min)');
    xlim(axDLDOP, [0 max(t_axis_min)]);
    curDLDOP = line(axDLDOP, [0 0], ylim(axDLDOP), 'Color', 'k', 'LineWidth', 1.8);

    % ---- UL tab ----
    tabUL = uitab(tabs, 'Title', '上行链路 UL');
    glUL = uigridlayout(tabUL, [2 2]);
    glUL.RowHeight = {'1x','1x'}; glUL.ColumnWidth = {'1x','1x'};
    axULSINR = uiaxes(glUL); axULSINR.Layout.Row = 1; axULSINR.Layout.Column = 1;
    axULBER  = uiaxes(glUL); axULBER.Layout.Row = 1; axULBER.Layout.Column = 2;
    axULTHR  = uiaxes(glUL); axULTHR.Layout.Row = 2; axULTHR.Layout.Column = 1;
    axULDOP  = uiaxes(glUL); axULDOP.Layout.Row = 2; axULDOP.Layout.Column = 2;

    hULSINR1 = plot(axULSINR, t_axis_min, simUL_Base.SINR, '--', 'LineWidth', 1.3); hold(axULSINR, 'on');
    hULSINR2 = plot(axULSINR, t_axis_min, simUL_Worst.SINR, '-', 'LineWidth', 2.0);
    grid(axULSINR, 'on'); title(axULSINR, 'UL SINR'); ylabel(axULSINR, 'dB'); xlabel(axULSINR, 'Time (min)');
    xlim(axULSINR, [0 max(t_axis_min)]); ylim(axULSINR, cfg.Display.SINR_YLIM_UL); yline(axULSINR, 0, 'r--', 'Disconnect');
    shadeTagRegions(axULSINR, t_axis_min, simUL_Worst.Event, "JAMMING!!!", cfg.Display.SINR_YLIM_UL, [1 0 0], 'Jamming', -10);
    shadeTagRegions(axULSINR, t_axis_min, simUL_Worst.Event, "Protected", cfg.Display.SINR_YLIM_UL, [0 1 0], 'Protected', 40);
    shadeTagRegions(axULSINR, t_axis_min, simUL_Worst.Event, "CoChannel", cfg.Display.SINR_YLIM_UL, [1 0.5 0], 'Co-Channel', 20);
    hULJam = patch(axULSINR, nan, nan, [1 0 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    hULProt = patch(axULSINR, nan, nan, [0 1 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    hULCo = patch(axULSINR, nan, nan, [1 0.5 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    legend(axULSINR, [hULSINR1 hULSINR2 hULJam hULProt hULCo], {'UL Base','UL Worst','Jamming','Protected','Co-Channel'}, 'Location', 'northeast');
    curULSINR = line(axULSINR, [0 0], cfg.Display.SINR_YLIM_UL, 'Color', 'k', 'LineWidth', 1.8);
    dotULSINR = plot(axULSINR, 0, cfg.Display.SINR_YLIM_UL(1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    semilogy(axULBER, t_axis_min, simUL_Base.BER, '--', 'LineWidth', 1.3); hold(axULBER, 'on');
    semilogy(axULBER, t_axis_min, simUL_Worst.BER, '-', 'LineWidth', 2.0);
    grid(axULBER, 'on'); title(axULBER, 'UL BER (log)'); ylabel(axULBER, 'log'); xlabel(axULBER, 'Time (min)');
    xlim(axULBER, [0 max(t_axis_min)]); ylim(axULBER, [1e-9 1]);
    curULBER = line(axULBER, [0 0], [1e-9 1], 'Color', 'k', 'LineWidth', 1.8);
    dotULBER = plot(axULBER, 0, 1, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    plot(axULTHR, t_axis_min, simUL_Base.THR, '--', 'LineWidth', 1.3); hold(axULTHR, 'on');
    plot(axULTHR, t_axis_min, simUL_Worst.THR, '-', 'LineWidth', 2.0);
    grid(axULTHR, 'on'); title(axULTHR, 'UL Throughput'); ylabel(axULTHR, 'Mbps'); xlabel(axULTHR, 'Time (min)');
    xlim(axULTHR, [0 max(t_axis_min)]); ylim(axULTHR, cfg.Display.THR_YLIM_UL); yline(axULTHR, cfg.Requirements.MinThr_Mbps, 'r--', 'Req');
    curULTHR = line(axULTHR, [0 0], cfg.Display.THR_YLIM_UL, 'Color', 'k', 'LineWidth', 1.8);
    dotULTHR = plot(axULTHR, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    plot(axULDOP, t_axis_min, simUL_Worst.DOPkHz, 'LineWidth', 2.0);
    grid(axULDOP, 'on'); title(axULDOP, 'UL Doppler'); ylabel(axULDOP, 'kHz'); xlabel(axULDOP, 'Time (min)');
    xlim(axULDOP, [0 max(t_axis_min)]);
    curULDOP = line(axULDOP, [0 0], ylim(axULDOP), 'Color', 'k', 'LineWidth', 1.8);

    % ---- Right information panel ----
    right = uigridlayout(gl, [16 1]);
    right.Layout.Row = 2;
    right.Layout.Column = 2;
    right.RowHeight = {24,24,24,24,24,24,24,24,24,24,110,22,24,260,160,'1x'};
    right.Padding = [0 0 0 0];

    lblTime  = uilabel(right, 'Text', 'Current Time: -', 'FontWeight', 'bold');
    lblSpeed = uilabel(right, 'Text', 'Viewer Speed: -');
    lblServ  = uilabel(right, 'Text', 'Serving Sat: -');
    lblGW    = uilabel(right, 'Text', 'Gateway Sat: -');
    lblHops  = uilabel(right, 'Text', 'ISL Hops: -');
    lblVis   = uilabel(right, 'Text', 'Visible(User/GW): - / -');
    lblDL    = uilabel(right, 'Text', 'DL | -');
    lblUL    = uilabel(right, 'Text', 'UL | -');
    lblE2E   = uilabel(right, 'Text', 'E2E | -');
    lblIntf  = uilabel(right, 'Text', 'Interference Class (DL STFT+LeNet): -', 'FontWeight', 'bold');

    axIntf = uiaxes(right); axIntf.Toolbar.Visible = 'off'; axIntf.Interactions = [];
    title(axIntf, 'Interference Class Scores'); ylabel(axIntf, 'Score');
    xticks(axIntf, 1:4); xticklabels(axIntf, {'none','tone','pbnj','mod'});
    ylim(axIntf, [0 1]); grid(axIntf, 'on');
    barIntf = bar(axIntf, nan(1,4));

    uilabel(right, 'Text', 'Overall Compliance Lamp:');
    lamp = uilamp(right); lamp.Color = [0 1 0];

    tblComp = uitable(right, 'Data', cell(0,4), ...
        'ColumnName', {'指标','当前值','门限','状态'}, ...
        'ColumnEditable', [false false false false]);

    axGrid = uiaxes(right);
    axGrid.Toolbar.Visible = 'off';
    axGrid.Interactions = [];
    title(axGrid, 'Constellation Grid + Route');
    xlabel(axGrid, 'Plane');
    ylabel(axGrid, 'Slot');
    xlim(axGrid, [0.5 numPlanes+0.5]);
    ylim(axGrid, [0.5 satsPerPlane+0.5]);
    grid(axGrid, 'on');
    hold(axGrid, 'on');
    axGrid.DataAspectRatioMode = 'auto';
    axGrid.PlotBoxAspectRatioMode = 'auto';
    axGrid.XDir = 'normal';
    axGrid.YDir = 'normal';
    scatter(axGrid, satPlane(:), satSlot(:), 18, [0.55 0.55 0.55], 'filled');
    gridServ = scatter(axGrid, nan, nan, 90, [0.62 0.24 0.12], 'filled', 'MarkerEdgeColor', [0.35 0.10 0.05]);
    gridGW   = scatter(axGrid, nan, nan, 90, [1.00 0.85 0.15], 'filled', 'MarkerEdgeColor', [0.55 0.40 0]);
    pathLine = plot(axGrid, nan, nan, '-', 'LineWidth', 2.8, 'Color', [1.00 0.45 0.05]);

    uilabel(right, 'Text', '事件窗口已独立显示', 'FontAngle', 'italic');

    evtFig = uifigure('Name', 'DL / UL Event Log', 'Color', 'w', 'Position', [1735 730 620 260]);
    tblEvt = uitable(evtFig, ...
        'Data', cell(0,4), ...
        'ColumnName', {'Time','DL Event','UL Event','Combined'}, ...
        'ColumnEditable', [false false false false], ...
        'Position', [10 10 600 240]);

    % ---- Separate Sky View 3D window ----
    skyFig = uifigure('Name', 'Sky View 3D', 'Color', 'w', 'Position', [1735 80 720 740]);

    axSky3D = uiaxes(skyFig);
    axSky3D.Position = [20 90 680 620];
    title(axSky3D, 'Sky View 3D');
    xlabel(axSky3D, 'East');
    ylabel(axSky3D, 'North');
    zlabel(axSky3D, 'Up');
    grid(axSky3D, 'on');
    view(axSky3D, 36, 28);
    axis(axSky3D, [-1 1 -1 1 0 1]);
    axis(axSky3D, 'equal');
    hold(axSky3D, 'on');

    th = linspace(0, 2*pi, 181);
    for rr = [0.25 0.5 0.75 1.0]
        plot3(axSky3D, rr*cos(th), rr*sin(th), zeros(size(th)), '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
    end
    for elc = [15 30 45 60 75]
        rxy = cosd(elc);
        zc  = sind(elc);
        plot3(axSky3D, rxy*cos(th), rxy*sin(th), zc*ones(size(th)), ':', ...
            'Color', [0.75 0.75 0.75], 'LineWidth', 0.6);
    end
    for azc = 0:45:315
        plot3(axSky3D, [0 cosd(azc)], [0 sind(azc)], [0 0], ':', ...
            'Color', [0.80 0.80 0.80], 'LineWidth', 0.6);
    end
    text(axSky3D, 0, 0, 1.05, 'Zenith', 'HorizontalAlignment', 'center');
    text(axSky3D, 1.05, 0, 0, 'E');
    text(axSky3D, -1.08, 0, 0, 'W');
    text(axSky3D, 0, 1.05, 0, 'N');
    text(axSky3D, 0, -1.08, 0, 'S');

    sky3All = scatter3(axSky3D, nan, nan, nan, 18, [0.30 0.50 0.95], 'filled');
    sky3Serv = scatter3(axSky3D, nan, nan, nan, 90, [0.62 0.24 0.12], 'filled');
    sky3GW = scatter3(axSky3D, nan, nan, nan, 90, [1.00 0.85 0.15], 'filled');
    sky3UserLink = plot3(axSky3D, nan, nan, nan, '-', 'Color', [1.00 0.45 0.05], 'LineWidth', 2.6);
    sky3ISLPath  = plot3(axSky3D, nan, nan, nan, '-', 'Color', [0.00 0.95 0.95], 'LineWidth', 2.4);
    sky3Labels = gobjects(0,1);

    txtSkyRoute = uitextarea(skyFig, ...
        'Value', {'Current Route: -',' '}, ...
        'Editable', 'off', ...
        'FontWeight', 'bold', ...
        'FontSize', 11, ...
        'Position', [20 18 680 56]);

    legend(axSky3D, [sky3All, sky3Serv, sky3GW, sky3UserLink, sky3ISLPath], ...
        {'Visible Sats','Serving Sat','Gateway Sat','User Link','ISL Route'}, ...
        'Location', 'northeast');

    %% =========================
    % PART 8: viewer + timer link
    % ==========================
    fprintf('Step 9: 启动正常 3D Viewer ...\n');
    v = [];
    if cfg.Output.Enable3DViewer
        try
            v = satelliteScenarioViewer(sc, 'Basemap', 'none', ...
                'PlaybackSpeedMultiplier', cfg.Display.ViewerSpeed, 'Dimension', '3D');
        catch
            try
                v = satelliteScenarioViewer(sc, ...
                    'PlaybackSpeedMultiplier', cfg.Display.ViewerSpeed, 'Dimension', '3D');
            catch
                try
                    v = satelliteScenarioViewer(sc);
                    try, v.PlaybackSpeedMultiplier = cfg.Display.ViewerSpeed; catch, end
                catch
                    v = [];
                end
            end
        end
        try, showAll(v); catch, end
    end

    app = struct();
    app.cfg = cfg;
    app.sim_start = sim_start;
    app.sample_time = sample_time;
    app.numSteps = numSteps;
    app.t_axis_min = t_axis_min;

    app.baseDL = simDL_Base;
    app.worstDL = simDL_Worst;
    app.baseUL = simUL_Base;
    app.worstUL = simUL_Worst;
    app.e2eBase = simE2E_Base;
    app.e2eWorst = simE2E_Worst;

    app.Gisl = Gisl;
    app.azU = azU;
    app.elU = elU;
    app.azG = azG;
    app.elG = elG;
    app.satPlane = satPlane;
    app.satSlot = satSlot;
    app.satName = satName;

    app.curDLSINR = curDLSINR;
    app.curDLBER  = curDLBER;
    app.curDLTHR  = curDLTHR;
    app.curDLDOP  = curDLDOP;
    app.curULSINR = curULSINR;
    app.curULBER  = curULBER;
    app.curULTHR  = curULTHR;
    app.curULDOP  = curULDOP;
    app.curE2E    = curE2E;
    app.curOvThr  = curOvThr;
    app.curOvJam  = curOvJam;
    app.curOvDop  = curOvDop;

    app.dotDLSINR = dotDLSINR;
    app.dotDLBER  = dotDLBER;
    app.dotDLTHR  = dotDLTHR;
    app.dotULSINR = dotULSINR;
    app.dotULBER  = dotULBER;
    app.dotULTHR  = dotULTHR;

    app.lblTime = lblTime;
    app.lblSpeed = lblSpeed;
    app.lblServ = lblServ;
    app.lblGW = lblGW;
    app.lblHops = lblHops;
    app.lblVis = lblVis;
    app.lblDL = lblDL;
    app.lblUL = lblUL;
    app.lblE2E = lblE2E;
    app.lblIntf = lblIntf;

    app.axIntf = axIntf;
    app.barIntf = barIntf;
    app.lamp = lamp;
    app.tblComp = tblComp;
    app.tblEvt = tblEvt;
    app.evtFig = evtFig;

    app.skyFig = skyFig;
    app.axSky3D = axSky3D;
    app.sky3All = sky3All;
    app.sky3Serv = sky3Serv;
    app.sky3GW = sky3GW;
    app.sky3UserLink = sky3UserLink;
    app.sky3ISLPath = sky3ISLPath;
    app.sky3Labels = sky3Labels;
    app.txtSkyRoute = txtSkyRoute;

    app.gridServ = gridServ;
    app.gridGW = gridGW;
    app.pathLine = pathLine;

    app.activeLinkColor = [0.00 0.95 0.95];
    app.servSatColor    = [0.62 0.24 0.12];

    app.satConst = satConst;
    app.gsUser = gsUser;
    app.gsGW   = gsGW;

    app.acUser = acUser;
    app.acGW   = acGW;
    app.acISL  = acISL;
    app.islMap = islMap;
    app.acJamUser = acJamUser;

    app.lastRoute = [];
    app.lastServ = 0;
    app.lastGW = 0;
    app.lastEvent = "";
    app.v = v;

    if isfield(simDL_Worst, 'IntfPred')
        app.intfPred = simDL_Worst.IntfPred;
        app.intfTrue = simDL_Worst.IntfTrue;
        app.intfScore = simDL_Worst.IntfScore;
    else
        app.intfPred = [];
        app.intfTrue = [];
        app.intfScore = [];
    end

    guidata(dash, app);

    tmr = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.20, 'BusyMode', 'drop', ...
        'TimerFcn', @(~,~)onTickV7(dash));
    dash.CloseRequestFcn = @(src,evt)onCloseV7(src,evt,tmr);
    start(tmr);

    try
        if ~isempty(v), play(v); end
    catch
        try, play(sc); catch, end
    end

    onTickV7(dash);

    %% save result artifacts
    result = struct();
    result.cfg = cfg;
    result.simDL_Base = simDL_Base;
    result.simDL_Worst = simDL_Worst;
    result.simUL_Base = simUL_Base;
    result.simUL_Worst = simUL_Worst;
    result.simE2E_Base = simE2E_Base;
    result.simE2E_Worst = simE2E_Worst;
    result.JamScaleBest_dB = JamScaleBest_dB;
    result.jamAggWorst = jamAggWorst;
    result.Dashboard = dash;
    result.Viewer = v;

    if cfg.Output.AutoSaveResultMat
        resultSave = result; %#ok<NASGU>
        resultSave = rmfield(resultSave, {'Dashboard','Viewer'});
        save(fullfile(cfg.Output.ExportFolder, cfg.Output.ResultMatFile), 'resultSave', '-v7.3');
    end

    emcWriteSummaryTextV7(fullfile(cfg.Output.ExportFolder, cfg.Output.SummaryTextFile), ...
        cfg, simDL_Base, simDL_Worst, simUL_Base, simUL_Worst, simE2E_Base, simE2E_Worst);
end

function cfg = resolveInputCfg(cfgDefault, varargin)
    cfg = cfgDefault;
    if nargin < 2 || isempty(varargin)
        cfg = emcLaunchConfigUI(cfgDefault);
        return;
    end

    arg1 = varargin{1};
    if isstruct(arg1)
        cfg = emcMergeStruct(cfgDefault, arg1);
        cfg.General.StartupMode = 'input-struct';
        return;
    end

    if ischar(arg1) || isstring(arg1)
        arg1 = char(arg1);
        if exist(arg1, 'file') == 2
            try
                S = load(arg1);
                if isfield(S, 'cfg') && isstruct(S.cfg)
                    cfg = emcMergeStruct(cfgDefault, S.cfg);
                else
                    f = fieldnames(S);
                    for i = 1:numel(f)
                        if isstruct(S.(f{i}))
                            cfg = emcMergeStruct(cfgDefault, S.(f{i}));
                            break;
                        end
                    end
                end
                cfg.General.StartupMode = 'input-mat';
            catch
                cfg = cfgDefault;
                cfg.General.StartupMode = 'default';
            end
        else
            cfg = emcLaunchConfigUI(cfgDefault);
        end
        return;
    end

    cfg = emcLaunchConfigUI(cfgDefault);
end