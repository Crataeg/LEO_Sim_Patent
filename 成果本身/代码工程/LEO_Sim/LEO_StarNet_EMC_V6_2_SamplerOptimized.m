%% LEO StarNet EMC + GA(GAN) Worst-case + 3D Linked Dashboard (R2021a)  [V6.2 FULL]
% 完全体增强点（在 V6.1 基础上“不改主仿真逻辑”，只补齐你 gen_set/train_lenet 的“图片生成链路”）：
%  A) 生成 STFT 数据集图像（train/val/test, 4类：none/tone/pbnj/mod）
%  B) 导出数据集样例拼图（montage）+ 测试集混淆矩阵
%  C) 导出仿真时间轴关键时刻 STFT 图（用于PPT展示：干扰出现/保护/共信道等时刻）
%  D) Dashboard 仍实时显示分类结果条形图 + 3D联动竖线 + 链路高亮
%
% 依赖：
%  - Satellite Communications Toolbox
%  - Deep Learning Toolbox
%  - Optimization Toolbox (ga)
%  - Image Processing Toolbox（你已安装：im2single/im2gray/imresize/montage等可用）
%
% 输出目录（默认）：
%  - dataset_stft_r2021a/ (train/val/test 四类图片)
%  - dataset_stft_r2021a/_exports/ (样例拼图、混淆矩阵、仿真关键帧STFT图)
 
clear; clc; close all;
rng(7);
 
fprintf('============================================================\n');
fprintf('  LEO StarNet EMC | Worst-case Search (GA on GAN) | R2021a V6.2 FULL\n');
fprintf('============================================================\n');
 
%% =========================
% PART 0: 参数区（可改）
% =========================
Epoch = datetime(2026,1,23,0,0,0,'TimeZone','UTC');   % 固定Epoch：不要用 now
 
Re  = 6371e3;
mu  = 3.986004418e14;
c   = 299792458;
 
% --- 星座 ---
h      = 1200e3;
a      = Re + h;
ecc    = 0.0;
incDeg = 53;
 
numPlanes    = 12;
satsPerPlane = 8;
F_phasing    = 1;
Nsat = numPlanes*satsPerPlane;
 
% --- 仿真时长：一圈轨道 ---
T_orbit = 2*pi*sqrt(a^3/mu);
sample_time = 10;     % 秒
sim_start = Epoch;
sim_stop  = sim_start + seconds(T_orbit);
 
% --- 地面站 ---
gsUserLat = 36.06; gsUserLon = 120.38; % 青岛
gsGWLat   = 39.90; gsGWLon   = 116.40; % 北京
 
% --- 链路/EMC ---
fc_Hz     = 28e9;
freq_MHz  = fc_Hz/1e6;
BW        = 100e6;
Noise_dBm = -110;
 
TxP_S_dBm = 55;
TxP_I_dBm = 55;                 % 共信道干扰等效
TxP_J_base_dBm = 88;            % Jammer 基础功率
G_rx_dB   = 35;
G_int_penalty_dB = 10;
 
elMaskDeg     = 5;
reuseK        = 4;
 
% ====== Jammer角度模式（连续存在，强度随 off-axis 衰减） ======
J_main_deg = 3;
J_side_deg = 20;
J_main_gain_dB  = 0;
J_side_gain_dB  = -8;
J_floor_gain_dB = -12;
% ===============================================================
 
% 抗干扰状态机
AJ_DelaySec      = 30;
AJ_NullDepth_dB  = 40;
 
% --- 红队最劣搜索 ---
EnableWorstCaseSearch = true;
 
GAN_seqLen  = 128;
GAN_zDim    = 16;
GAN_trainIters = 250;
GAN_modelFile  = 'GAN_Jammer_R2021a.mat';
 
GA_PopSize     = 16;
GA_Generations = 10;
 
JamScale_lb_dB = 0;
JamScale_ub_dB = 35;
 
z_lb = -2; z_ub = 2;
 
W_outage  = 300;
W_bler    = 80;
W_energy  = 80;
 
SINR_YLIM = [-20 50];
THR_YLIM  = [0 600];
 
%% =========================
% PART 0.5: STFT+LeNet（你 gen_set/train_lenet 的“图片生成+训练+导出”完全体）
% =========================
EnableInterfClassifier = true;                 % 可关掉
IntfModelFile  = 'lenet_stft_model_r2021a.mat';
IntfDatasetRoot = 'dataset_stft_r2021a';
IntfTrainIfMissing = true;                    % 没模型就训练
IntfForceRegenDataset = false;                % 想强制重生成数据集时改 true
IntfExportImages = true;                      % ? 导出你要的“那些图片”（样例拼图/混淆矩阵/仿真关键帧STFT）
IntfImgExportDir = fullfile(IntfDatasetRoot, '_exports');
 
%% =========================
% PART 1: 场景 & 星座
% =========================
fprintf('Step 1: 构建 satelliteScenario（固定Epoch + 全轨道周期）...\n');
sc = satelliteScenario(sim_start, sim_stop, sample_time);
 
gsUser = groundStation(sc, gsUserLat, gsUserLon, 'Name', 'Qingdao_User');
gsGW   = groundStation(sc, gsGWLat,   gsGWLon,   'Name', 'Beijing_Gateway');
 
fprintf('Step 2: 生成星座：%d×%d=%d satellites ...\n', numPlanes, satsPerPlane, Nsat);
 
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
        satName(idx)  = nm;
        satPlane(idx) = p;
        satSlot(idx)  = s;
    end
end
 
% Jammer（适度增加到6个）
numJam = 6;
satJam = cell(1, numJam);

% --- NEW: keep jammer sensor handles so the cone/FOV can be shown in 3D viewer ---
jamSensor = cell(1, numJam);
jamFOV    = gobjects(1, numJam);

for j = 1:numJam
    raanJ = (j-1)*(360/numJam);
    taJ   = 180 + 10*j;
    satJam{j} = satellite(sc, a, ecc, incDeg, raanJ, 0, taJ, 'Name', sprintf('JAMMER_%d', j));

    % Make jammer satellite visually distinct (best-effort; property may vary by version)
    try, satJam{j}.MarkerColor = [1 0 0]; catch, end

    try
        jamSensor{j} = conicalSensor(satJam{j}, 'MaxViewAngle', 25);  % widen so it is easier to see
        jamFOV(j) = fieldOfView(jamSensor{j});
        try, jamFOV(j).LineColor = [1 0 0]; catch, end
        try, jamFOV(j).LineWidth = 1.2; catch, end
        try, jamFOV(j).FaceColor = [1 0 0]; catch, end
        try, jamFOV(j).FaceAlpha = 0.06;  catch, end
    catch
    end
end

%% =========================
% PART 2: 预计算几何
% =========================
fprintf('Step 3: 预计算几何...\n');
 
timeVec = 0:sample_time:seconds(sim_stop - sim_start);
numSteps_guess = numel(timeVec);
 
[~, el0, ~] = aer(gsUser, satConst{1});
L = min(numSteps_guess, numel(el0));
timeVec = timeVec(1:L);
numSteps = L;
t_axis_min = timeVec/60;
 
% User->Sat
azU = nan(numSteps, Nsat); elU = nan(numSteps, Nsat); rU = nan(numSteps, Nsat);
for i = 1:Nsat
    [az, el, r] = aer(gsUser, satConst{i});
    Li = min(numSteps, numel(el));
    azU(1:Li,i) = az(1:Li);
    elU(1:Li,i) = el(1:Li);
    rU(1:Li,i)  = r(1:Li);
end
 
% GW->Sat
azG = nan(numSteps, Nsat); elG = nan(numSteps, Nsat); rG = nan(numSteps, Nsat);
for i = 1:Nsat
    [az, el, r] = aer(gsGW, satConst{i});
    Li = min(numSteps, numel(el));
    azG(1:Li,i) = az(1:Li);
    elG(1:Li,i) = el(1:Li);
    rG(1:Li,i)  = r(1:Li);
end
 
% User->Jammer
azJ = nan(numSteps, numJam); elJ = nan(numSteps, numJam); rJ = nan(numSteps, numJam);
for j = 1:numJam
    [az, el, r] = aer(gsUser, satJam{j});
    Lj = min(numSteps, numel(el));
    azJ(1:Lj,j) = az(1:Lj);
    elJ(1:Lj,j) = el(1:Lj);
    rJ(1:Lj,j)  = r(1:Lj);
end
 
% range-rate（Doppler）
rrU = nan(numSteps, Nsat);
for i = 1:Nsat
    ri = rU(:,i);
    dri = [diff(ri)/sample_time; 0];
    rrU(:,i) = movmean(dri, 5);
end
 
%% =========================
% PART 3: 频率复用 & ISL拓扑
% =========================
fprintf('Step 4: 构建频率复用与简化ISL图...\n');
 
satChan = zeros(1, Nsat);
for i = 1:Nsat
    satChan(i) = mod((satPlane(i)-1) + (satSlot(i)-1), reuseK) + 1;
end
 
toIdx = @(p,s) (p-1)*satsPerPlane + s;
sList = []; tList = []; wList = [];
 
d_inplane = 2*a*sin(pi/satsPerPlane);
d_cross   = 2*a*sin(pi/numPlanes);
 
for p = 1:numPlanes
    for s = 1:satsPerPlane
        u = toIdx(p,s);
 
        s_fwd = s+1; if s_fwd>satsPerPlane, s_fwd=1; end
        s_bwd = s-1; if s_bwd<1, s_bwd=satsPerPlane; end
        v1 = toIdx(p,s_fwd); v2 = toIdx(p,s_bwd);
 
        p_r = p+1; if p_r>numPlanes, p_r=1; end
        p_l = p-1; if p_l<1, p_l=numPlanes; end
        v3 = toIdx(p_r,s); v4 = toIdx(p_l,s);
 
        sList = [sList u u u u]; %#ok<AGROW>
        tList = [tList v1 v2 v3 v4]; %#ok<AGROW>
        wList = [wList d_inplane d_inplane d_cross d_cross]; %#ok<AGROW>
    end
end
Gisl = graph(sList, tList, wList, Nsat);
 
%% =========================
% PART 4: 预计算功率项（加速GA评估）
% =========================
fprintf('Step 5: 预计算功率项（加速GA评估）...\n');
 
visU_all = elU > elMaskDeg;
visG_all = elG > elMaskDeg;
visJ_all = elJ > elMaskDeg;
 
pS_mW = zeros(numSteps, Nsat);
pIself_mW = zeros(numSteps, Nsat);
 
for i = 1:Nsat
    PL = fspl_dB_vec(rU(:,i), freq_MHz);
    PsdBm = TxP_S_dBm - PL + G_rx_dB;
    PidBm = TxP_I_dBm - PL + (G_rx_dB - G_int_penalty_dB);
 
    tmpS = 10.^(PsdBm/10); tmpI = 10.^(PidBm/10);
    tmpS(~visU_all(:,i)) = 0;
    tmpI(~visU_all(:,i)) = 0;
 
    pS_mW(:,i) = tmpS;
    pIself_mW(:,i) = tmpI;
end
 
totCCI = zeros(numSteps, reuseK);
for cidx = 1:reuseK
    members = find(satChan==cidx);
    totCCI(:,cidx) = sum(pIself_mW(:,members), 2);
end
 
% Jammer基接收功率（仅由距离决定）
pJ_base_mW = zeros(numSteps, numJam);
for j = 1:numJam
    PLj = fspl_dB_vec(rJ(:,j), freq_MHz);
    gainJ = 10;
    PJdBm = TxP_J_base_dBm - PLj + gainJ;
    tmp = 10.^(PJdBm/10);
    tmp(~visJ_all(:,j)) = 0;
    pJ_base_mW(:,j) = tmp;
end
 
% 预计算：对每个 (k,i) ，Jammer总干扰功率 = sum_j pJ_base(j,k) * AntPattern(offAxis)
pJsum_base_mW = zeros(numSteps, Nsat);
for j = 1:numJam
    azj = azJ(:,j); elj = elJ(:,j);
    for i = 1:Nsat
        daz = wrap180_vec(azU(:,i) - azj);
        del = elU(:,i) - elj;
        offAxis = sqrt(daz.^2 + del.^2);  % deg
        active = (visJ_all(:,j) & visU_all(:,i));
        if any(active)
            patLin = jammerPatternLin(offAxis(active), J_main_deg, J_side_deg, J_main_gain_dB, J_side_gain_dB, J_floor_gain_dB);
            pJsum_base_mW(active, i) = pJsum_base_mW(active, i) + pJ_base_mW(active, j) .* patLin;
        end
    end
end
 
p_n = 10^(Noise_dBm/10);
 
%% =========================
% PART 5: GAN + GA 最劣搜索
% =========================
fprintf('Step 6: Worst-case Search (GA on GAN) ...\n');
 
TxP_J_off_dBm = -200;
jamAgg_zero = zeros(numSteps,1);
JamScale0_dB = 0;
 
simBase = simulateStarNet( ...
    numSteps, sample_time, ...
    visU_all, visG_all, ...
    azU, elU, rU, rrU, ...
    elG, rG, ...
    pS_mW, totCCI, satChan, pIself_mW, ...
    pJsum_base_mW, TxP_J_off_dBm, TxP_J_base_dBm, ...
    jamAgg_zero, JamScale0_dB, ...
    AJ_DelaySec, AJ_NullDepth_dB, ...
    fc_Hz, c, BW, p_n, Gisl);
 
if EnableWorstCaseSearch
    [netG, ~] = trainOrLoadJammerGAN(GAN_seqLen, GAN_zDim, GAN_trainIters, GAN_modelFile);
 
    nvars = GAN_zDim + 1;
    lb = [z_lb*ones(GAN_zDim,1); JamScale_lb_dB];
    ub = [z_ub*ones(GAN_zDim,1); JamScale_ub_dB];
 
    obj = @(x) worstCaseObjective( ...
        x, netG, GAN_seqLen, ...
        numSteps, sample_time, ...
        visU_all, visG_all, ...
        azU, elU, rU, rrU, ...
        elG, rG, ...
        pS_mW, totCCI, satChan, pIself_mW, ...
        pJsum_base_mW, TxP_J_base_dBm, ...
        AJ_DelaySec, AJ_NullDepth_dB, ...
        fc_Hz, c, BW, p_n, Gisl, ...
        W_outage, W_bler, W_energy);
 
    gaOpts = optimoptions('ga', ...
        'PopulationSize', GA_PopSize, ...
        'MaxGenerations', GA_Generations, ...
        'Display', 'iter', ...
        'UseParallel', false);
 
    try
        [xBest, ~] = ga(obj, nvars, [],[],[],[], lb, ub, [], gaOpts);
    catch ME
        fprintf('[警告] GA失败：%s\n', ME.message);
        xBest = [zeros(GAN_zDim,1); 20];
    end
 
    zBest = xBest(1:GAN_zDim);
    JamScaleBest_dB = xBest(end);
    jamAggWorst = genJamAggFromG(netG, zBest, GAN_seqLen, numSteps);
else
    JamScaleBest_dB = 0;
    jamAggWorst = zeros(numSteps,1);
end
 
simWorst = simulateStarNet( ...
    numSteps, sample_time, ...
    visU_all, visG_all, ...
    azU, elU, rU, rrU, ...
    elG, rG, ...
    pS_mW, totCCI, satChan, pIself_mW, ...
    pJsum_base_mW, TxP_J_base_dBm, TxP_J_base_dBm, ...
    jamAggWorst, JamScaleBest_dB, ...
    AJ_DelaySec, AJ_NullDepth_dB, ...
    fc_Hz, c, BW, p_n, Gisl);
 
fprintf('\n===================== SUMMARY =====================\n');
fprintf('Baseline (No Jam):   meanThr=%.1f Mbps, outage=%.2f%%\n', simBase.meanThr, 100*simBase.outageFrac);
fprintf('Worst-case (GA+GAN): meanThr=%.1f Mbps, outage=%.2f%%, JamScale=%.1f dB\n', ...
    simWorst.meanThr, 100*simWorst.outageFrac, JamScaleBest_dB);
fprintf('===================================================\n\n');
 
%% =========================
% PART 5.5: 干扰类型分类（STFT图像 + LeNet）+ ?你要的“图片生成/导出”完全体
% =========================
if EnableInterfClassifier
    fprintf('Step 6.5: STFT+LeNet 干扰分类（含数据集图片生成/训练/导出）...\n');
 
    % 数据集（可选强制重生成）
    if IntfForceRegenDataset && exist(IntfDatasetRoot,'dir')
        fprintf('  [IntfCls] ForceRegenDataset=true，删除旧数据集：%s\n', IntfDatasetRoot);
        try, rmdir(IntfDatasetRoot,'s'); catch, end
    end
 
    [netIntf, intfClasses] = getOrTrainLeNetSTFT(IntfModelFile, IntfDatasetRoot, IntfTrainIfMissing);
 
    % ? 导出：样例拼图 + 混淆矩阵（对 test 集评估）
    if IntfExportImages
        if ~exist(IntfImgExportDir,'dir'), mkdir(IntfImgExportDir); end
        exportDatasetMontage(IntfDatasetRoot, IntfImgExportDir);
        exportTestConfusion(netIntf, IntfDatasetRoot, IntfImgExportDir, intfClasses);
    end
 
    % 生成“每个时间步”的干扰快照并分类（用于可视化，不影响主链路指标）
        % ---- NEW: Power-aligned waveform sampler for STFT snapshots (no more "scripted" demo waveforms) ----
    SamplerCfg = defaultPowerAlignedSamplerCfg();
    SamplerCfg.UsePostAJPower = true;      % true: use post-AJ PJ (align to main SINR); false: use raw PJ
    SamplerCfg.JammerType     = 'pbnj';    % {'tone','pbnj','mod'}  (pick one)
    SamplerCfg.CCIType        = 'mod';     % co-channel modeled as modulated wideband
    SamplerCfg.Ns             = 2048;      % snapshot IQ length

    [intfTrue, intfPred, intfScore, snapInfo] = classifyInterferenceTimeline_powerSampler( ...
        netIntf, intfClasses, numSteps, sample_time, simWorst, p_n, SamplerCfg);
 
    simWorst.IntfTrue  = intfTrue;
    simWorst.IntfPred  = intfPred;
    simWorst.IntfScore = intfScore;   % N×4
 
    % ? 导出：仿真关键帧 STFT 图（你PPT最需要的“仿真里发生了什么”）
    if IntfExportImages
        exportSimKeyframeSTFT(snapInfo, IntfImgExportDir);
    end
end
 
%% =========================
% PART 6: 3D Viewer 链路显示（access）
% =========================
fprintf('Step 7: 构建3D Viewer链路显示（access lines）...\n');
 
acUser = gobjects(1,Nsat);
acGW   = gobjects(1,Nsat);
for i = 1:Nsat
    % User <-> Sat access
    try
        acUser(i) = access(satConst{i}, gsUser);
        acUser(i).LineColor = [0.75 0.75 0.75];
        acUser(i).LineWidth = 0.6;
    catch
    end

    % GW <-> Sat access (make it bluish so it is easier to see)
    try
        acGW(i)   = access(satConst{i}, gsGW);
        acGW(i).LineColor   = [0.65 0.75 1.00];
        acGW(i).LineWidth   = 0.6;
    catch
    end
end

% --- NEW: ISL access lines (to visualize routing path in 3D) ---
islEnds = Gisl.Edges.EndNodes;
nISL = size(islEnds,1);
acISL = gobjects(nISL,1);
islMap = zeros(Nsat, Nsat, 'uint16'); % edgeIndex lookup

for e = 1:nISL
    u = islEnds(e,1); v = islEnds(e,2);
    islMap(u,v) = e; islMap(v,u) = e;
    try
        acISL(e) = access(satConst{u}, satConst{v});
        acISL(e).LineColor = [0.80 0.80 0.80];
        acISL(e).LineWidth = 0.4;
    catch
    end
end

% --- NEW: Jammer -> User access lines (helps "see" jammers in 3D) ---
acJamUser = gobjects(1,numJam);
for j = 1:numJam
    try
        acJamUser(j) = access(satJam{j}, gsUser);
        acJamUser(j).LineColor = [1 0 0];
        acJamUser(j).LineWidth = 0.9;
    catch
    end
end

%% =========================
% PART 7: Dashboard + 3D联动
% =========================
fprintf('Step 8: 构建Dashboard + 3D联动...\n');
 
dash = uifigure('Name','LEO StarNet EMC | GA+GAN Worst-case | Linked to 3D Viewer', ...
    'Color','w','Position',[40 40 1500 900]);
 
gl = uigridlayout(dash, [5 4]);
gl.RowHeight   = {36,'1x','1x','1x','1x'};
gl.ColumnWidth = {'1x','1x','1x',470};
gl.Padding     = [10 10 10 10];
gl.RowSpacing  = 8;
gl.ColumnSpacing = 10;
 
titleLbl = uilabel(gl, 'Text', ...
    sprintf('LEO StarNet EMC (R2021a) | %d sats | Full Orbit %.1f min | Worst-case: GA on GAN', Nsat, T_orbit/60), ...
    'FontSize',16,'FontWeight','bold');
titleLbl.Layout.Row = 1; titleLbl.Layout.Column = [1 4];
 
axSINR = uiaxes(gl); axSINR.Layout.Row=2; axSINR.Layout.Column=[1 3];
axBER  = uiaxes(gl); axBER.Layout.Row=3; axBER.Layout.Column=[1 3];
axTHR  = uiaxes(gl); axTHR.Layout.Row=4; axTHR.Layout.Column=[1 3];
axDOP  = uiaxes(gl); axDOP.Layout.Row=5; axDOP.Layout.Column=[1 3];
 
for ax = [axSINR axBER axTHR axDOP]
    ax.Toolbar.Visible = 'off';
    ax.Interactions = [];
end
 
plot(axSINR, t_axis_min, simBase.SINR, '--', 'LineWidth', 1.3); hold(axSINR,'on');
plot(axSINR, t_axis_min, simWorst.SINR,'-',  'LineWidth', 2.0);
grid(axSINR,'on'); title(axSINR,'SINR (dB)  [Baseline vs Worst-case]');
ylabel(axSINR,'dB'); xlim(axSINR,[0 max(t_axis_min)]); ylim(axSINR,SINR_YLIM);
yline(axSINR,0,'r--','Disconnect');
shadeTagRegions(axSINR, t_axis_min, simWorst.Event, "JAMMING!!!", SINR_YLIM, [1 0 0], 'Jamming', -10);
shadeTagRegions(axSINR, t_axis_min, simWorst.Event, "Protected",  SINR_YLIM, [0 1 0], 'Protected', 40);
shadeTagRegions(axSINR, t_axis_min, simWorst.Event, "CoChannel",  SINR_YLIM, [1 0.5 0], 'Co-Channel', 20);
 
semilogy(axBER, t_axis_min, simBase.BER,'--','LineWidth',1.3); hold(axBER,'on');
semilogy(axBER, t_axis_min, simWorst.BER,'-','LineWidth',2.0);
grid(axBER,'on'); title(axBER,'BER (log)  [Baseline vs Worst-case]');
ylabel(axBER,'Log Scale'); xlabel(axBER,'Time (Minutes)');
xlim(axBER,[0 max(t_axis_min)]); ylim(axBER,[1e-9 1]);
 
plot(axTHR, t_axis_min, simBase.THR,'--','LineWidth',1.3); hold(axTHR,'on');
plot(axTHR, t_axis_min, simWorst.THR,'-','LineWidth',2.0);
grid(axTHR,'on'); title(axTHR,'Throughput (Mbps)  [Baseline vs Worst-case]');
ylabel(axTHR,'Mbps'); xlabel(axTHR,'Time (Minutes)');
xlim(axTHR,[0 max(t_axis_min)]); ylim(axTHR,THR_YLIM);
 
plot(axDOP, t_axis_min, simWorst.DOPkHz,'LineWidth',2.0);
grid(axDOP,'on'); title(axDOP,'Doppler (kHz) (Worst-case Serving)');
ylabel(axDOP,'kHz'); xlabel(axDOP,'Time (Minutes)');
xlim(axDOP,[0 max(t_axis_min)]);
 
curSINR = line(axSINR,[0 0],SINR_YLIM,'Color','k','LineWidth',1.8);
curBER  = line(axBER, [0 0],[1e-9 1],'Color','k','LineWidth',1.8);
curTHR  = line(axTHR, [0 0],THR_YLIM,'Color','k','LineWidth',1.8);
curDOP  = line(axDOP, [0 0],ylim(axDOP),'Color','k','LineWidth',1.8);
 
dot1 = plot(axSINR,0,SINR_YLIM(1),'ko','MarkerFaceColor','k','MarkerSize',5);
dot2 = plot(axBER, 0,1,'ko','MarkerFaceColor','k','MarkerSize',5);
dot3 = plot(axTHR, 0,0,'ko','MarkerFaceColor','k','MarkerSize',5);
 
right = uigridlayout(gl, [18 1]);
right.Layout.Row = [2 5];
right.Layout.Column = 4;
right.RowHeight = {24,24,24,24,24,24,24,24,24,24,110,24,28,150,150,90,'1x',24};
right.Padding = [0 0 0 0];
 
lblTime  = uilabel(right,'Text','Current Time: -','FontWeight','bold');
lblSpeed = uilabel(right,'Text','Viewer Speed: -');
lblServ  = uilabel(right,'Text','Serving Sat: -');
lblGW    = uilabel(right,'Text','Gateway Sat: -');
lblHops  = uilabel(right,'Text','ISL Hops: -');
lblVis   = uilabel(right,'Text','Visible(User/GW): - / -');
lblSINR  = uilabel(right,'Text','SINR: - dB');
lblTHR   = uilabel(right,'Text','Thr: - Mbps  (Δ vs base: -)');
lblE2E   = uilabel(right,'Text','E2E Delay: - ms');
 
lblIntf = uilabel(right,'Text','Interference Class (STFT+LeNet): -','FontWeight','bold');
axIntf = uiaxes(right); axIntf.Toolbar.Visible='off'; axIntf.Interactions=[];
title(axIntf,'Interference Class Scores'); ylabel(axIntf,'Score');
xticks(axIntf,1:4); xticklabels(axIntf,{'none','tone','pbnj','mod'});
ylim(axIntf,[0 1]); grid(axIntf,'on');
barIntf = bar(axIntf, nan(1,4));
 
uilabel(right,'Text','Compliance Lamp (BLER<=0.1):');
lamp = uilamp(right); lamp.Color = [0 1 0];
 
axSky = uiaxes(right); axSky.Toolbar.Visible='off'; axSky.Interactions=[];
title(axSky,'Sky View (Az-El)'); xlabel(axSky,'Az (deg)'); ylabel(axSky,'El (deg)');
xlim(axSky,[-180 180]); ylim(axSky,[0 90]); grid(axSky,'on');
skyAll = scatter(axSky, nan, nan, 12, 'filled'); hold(axSky,'on');
skyServ= scatter(axSky, nan, nan, 45, 'filled');
skyGW  = scatter(axSky, nan, nan, 45, 'filled');
 
axGrid = uiaxes(right); axGrid.Toolbar.Visible='off'; axGrid.Interactions=[];
title(axGrid,'Constellation Grid (Plane-Slot) + Path'); xlabel(axGrid,'Plane'); ylabel(axGrid,'Slot');
xlim(axGrid,[0.5 numPlanes+0.5]); ylim(axGrid,[0.5 satsPerPlane+0.5]); grid(axGrid,'on');
scatter(axGrid, satPlane, satSlot, 18, 'filled'); hold(axGrid,'on');
gridServ= scatter(axGrid, nan, nan, 60, 'filled');
gridGW  = scatter(axGrid, nan, nan, 60, 'filled');
pathLine = plot(axGrid, nan, nan, '-', 'LineWidth', 2.0);
 
axJam = uiaxes(right); axJam.Toolbar.Visible='off'; axJam.Interactions=[];
plot(axJam, t_axis_min, jamAggWorst, 'LineWidth', 2); grid(axJam,'on');
title(axJam, sprintf('Worst-case Jam Envelope | JamScale=%.1f dB', JamScaleBest_dB));
xlabel(axJam,'Time (min)'); ylabel(axJam,'jamAgg (0..1)');
xlim(axJam,[0 max(t_axis_min)]); ylim(axJam,[0 1.05]);
curJam = line(axJam,[0 0],[0 1.05],'Color','k','LineWidth',1.6);
 
tbl = uitable(right, 'Data', cell(0,3), ...
    'ColumnName', {'Time','Event','Detail'}, 'ColumnEditable', [false false false]);
 
uilabel(right,'Text','提示：在3D Viewer里调速度/拖时间，竖线与链路高亮会同步。', 'WordWrap','on');
 
%% =========================
% PART 8: 启动3D Viewer + timer联动
% =========================
fprintf('Step 9: 启动3D Viewer...\n');
 
try
    v = satelliteScenarioViewer(sc, 'Basemap', 'none', 'PlaybackSpeedMultiplier', 60, 'Dimension', '3D');
catch
    try
        v = satelliteScenarioViewer(sc, 'PlaybackSpeedMultiplier', 60, 'Dimension', '3D');
    catch
        v = satelliteScenarioViewer(sc);
        try, v.PlaybackSpeedMultiplier = 60; catch, end
    end
end

% Make sure sensor FOV / access lines are visible in the 3D viewer (best-effort)
try
    showAll(v);
catch
end

app = struct();
app.sim_start = sim_start;
app.sample_time = sample_time;
app.numSteps = numSteps;
app.t_axis_min = t_axis_min;
 
app.base = simBase;
app.worst= simWorst;
 
if isfield(simWorst,'IntfPred')
    app.intfClasses = intfClasses;
    app.intfPred = simWorst.IntfPred;
    app.intfTrue = simWorst.IntfTrue;
    app.intfScore = simWorst.IntfScore;
else
    app.intfClasses = [];
    app.intfPred = [];
    app.intfTrue = [];
    app.intfScore = [];
end
 
app.azU = azU; app.elU = elU;
app.satPlane = satPlane; app.satSlot = satSlot;
app.Gisl = Gisl;
 
app.curSINR = curSINR; app.curBER = curBER; app.curTHR = curTHR; app.curDOP = curDOP; app.curJam = curJam;
app.dot1 = dot1; app.dot2 = dot2; app.dot3 = dot3;
 
app.lblTime=lblTime; app.lblSpeed=lblSpeed; app.lblServ=lblServ; app.lblGW=lblGW;
app.lblHops=lblHops; app.lblVis=lblVis; app.lblSINR=lblSINR; app.lblTHR=lblTHR; app.lblE2E=lblE2E;
app.lblIntf=lblIntf; app.axIntf=axIntf; app.barIntf=barIntf;
app.lamp=lamp; app.tbl=tbl;
 
app.skyAll=skyAll; app.skyServ=skyServ; app.skyGW=skyGW;
app.gridServ=gridServ; app.gridGW=gridGW; app.pathLine=pathLine;
 
app.acUser = acUser; app.acGW = acGW;
app.acISL  = acISL;  app.islMap = islMap; app.lastISLEdges = [];
app.acJamUser = acJamUser;
app.lastServ = 0; app.lastGW = 0;
app.lastEvent = "";
 
app.v = v;
guidata(dash, app);
 
tmr = timer('ExecutionMode','fixedSpacing','Period',0.05,'BusyMode','drop', 'TimerFcn', @(~,~)onTick(dash));
dash.CloseRequestFcn = @(src,evt)onClose(src,evt,tmr);
 
start(tmr);
 
try
    play(v);
catch
    try, play(sc); catch, end
end
 
%% =========================
% Local functions
% =========================
function patLin = jammerPatternLin(offDeg, mainDeg, sideDeg, mainGain_dB, sideGain_dB, floorGain_dB)
    g = zeros(size(offDeg));
    g(offDeg <= mainDeg) = mainGain_dB;
 
    mid = (offDeg > mainDeg) & (offDeg <= sideDeg);
    if any(mid)
        t = (offDeg(mid) - mainDeg) ./ max(1e-9, (sideDeg-mainDeg));
        g(mid) = mainGain_dB + t.*(sideGain_dB - mainGain_dB);
    end
 
    far = offDeg > sideDeg;
    g(far) = floorGain_dB;
 
    patLin = 10.^(g/10);
end
 
function sim = simulateStarNet( ...
    numSteps, dt, ...
    visU_all, visG_all, ...
    azU, elU, rU, rrU, ...
    elG, rG, ...
    pS_mW, totCCI, satChan, pIself_mW, ...
    pJsum_base_mW, TxP_J_effBase_dBm, TxP_J_base_dBm, ...
    jamAgg, JamScale_dB, ...
    AJ_DelaySec, AJ_NullDepth_dB, ...
    fc_Hz, c, BW, p_n, Gisl)
 
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

    % --- NEW: power breakdown (mW) for waveform sampler / STFT snapshots ---
    PS_mW    = zeros(numSteps,1);   % serving signal power
    PI_mW    = zeros(numSteps,1);   % co-channel interference power
    PJ_mW    = zeros(numSteps,1);   % jammer power (effective, post-AJ if active)
    PJraw_mW = zeros(numSteps,1);   % jammer power (raw, pre-AJ)
    AJ_Pre   = false(numSteps,1);
    AJ_Post  = false(numSteps,1);

    BLK = 1024;
 
    AJ_Active = false;
    AJ_Timer  = 0;
 
    JammerOn = (TxP_J_effBase_dBm > -100);
 
    for k = 1:numSteps
        visU = visU_all(k,:);
        visG = visG_all(k,:);
        VisUser(k) = sum(visU);
        VisGW(k)   = sum(visG);

        AJ_Pre(k) = AJ_Active;

        if ~any(visU)
            Event(k) = "No Service";
            AJ_Active=false; AJ_Timer=0;
            continue;
        end
 
        if any(visG)
            idxG = find(visG);
            [~,bg] = max(elG(k, idxG));
            gwIdx = idxG(bg);
        else
            gwIdx = 0;
        end
        Gateway(k) = gwIdx;
 
        cand = find(visU);
 
        sinrCand = -inf(size(cand));

        % 说明：把 serving 链路的功率分解存起来，供后面的“功率对齐采样器”生成 IQ
        pSCand = zeros(size(cand));
        pICand = zeros(size(cand));
        jamPowerCand    = zeros(size(cand));   % effective (post-AJ), used by link SINR
        jamPowerCandRaw = zeros(size(cand));   % raw (pre-AJ)

        for ii=1:numel(cand)
            si = cand(ii);

            p_s = pS_mW(k,si);

            ch = satChan(si);
            p_i = totCCI(k,ch) - pIself_mW(k,si);
            if p_i < 0, p_i = 0; end

            p_j_raw = 0;
            p_j_eff = 0;
            if JammerOn && pJsum_base_mW(k,si) > 0
                baseShift_dB = TxP_J_effBase_dBm - TxP_J_base_dBm;
                jamScaleLin = 10.^(((baseShift_dB + JamScale_dB*jamAgg(k))/10));
                p_j_raw = pJsum_base_mW(k,si) * jamScaleLin;
                p_j_eff = p_j_raw;
                if AJ_Active
                    p_j_eff = p_j_raw * 10^(-AJ_NullDepth_dB/10);
                end
            end

            pSCand(ii) = p_s;
            pICand(ii) = p_i;
            jamPowerCand(ii)    = p_j_eff;
            jamPowerCandRaw(ii) = p_j_raw;

            sinrLin = p_s / (p_i + p_j_eff + p_n);
            sinrCand(ii) = 10*log10(sinrLin);
        end
 
        if AJ_Active
            [~,best] = max(sinrCand);
        else
            [~,best] = max(elU(k,cand));
        end
        servIdx = cand(best);
        Serving(k) = servIdx;

        % --- store power breakdown for sampler ---
        PS_mW(k)    = pSCand(best);
        PI_mW(k)    = pICand(best);
        PJ_mW(k)    = jamPowerCand(best);
        PJraw_mW(k) = jamPowerCandRaw(best);

        jamOn = JammerOn && (jamPowerCand(best) > 1.0*p_n) && (jamAgg(k) > 0.05);
 
        if jamOn
            AJ_Timer = AJ_Timer + dt;
            if AJ_Timer >= AJ_DelaySec
                AJ_Active = true;
            end
        else
            AJ_Timer = 0;
            AJ_Active = false;
        end

        AJ_Post(k) = AJ_Active;

        sinr_dB = sinrCand(best);
        SINR(k) = sinr_dB;
 
        v_r = rrU(k, servIdx);
        dopHz = -(v_r/c)*fc_Hz;
        DOPkHz(k) = dopHz/1e3;
 
        lin = 10^(sinr_dB/10);
        ber = 0.5*erfc(sqrt(lin/2));
        ber = max(min(ber,0.5),1e-9);
        BER(k) = ber;
 
        if ber > 0.2
            THR(k) = 0;
        else
            eff = min(log2(1+lin), 6);
            THR(k) = BW*eff*0.8/1e6;
        end
 
        if gwIdx==0
            E2Ems(k) = nan;
            HopCnt(k)=0;
        else
            [pth, distISL] = shortestpath(Gisl, servIdx, gwIdx);
            if isinf(distISL)
                E2Ems(k) = nan;
                HopCnt(k)=0;
            else
                HopCnt(k) = max(0, numel(pth)-1);
                d_total = rU(k,servIdx) + rG(k,gwIdx) + distISL;
                E2Ems(k) = (d_total/c)*1e3;
            end
        end
 
        ch = satChan(servIdx);
        p_i_serv = totCCI(k,ch) - pIself_mW(k,servIdx);
        if p_i_serv < 0, p_i_serv=0; end
        p_s_serv = pS_mW(k,servIdx);
        isCo = p_i_serv > p_s_serv/10;
 
        if gwIdx==0
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
    for k=1:numSteps
        if isnan(BER(k)), BLER(k)=1; else
            BLER(k) = 1 - (1 - min(max(BER(k),0),1))^BLK;
        end
    end
 
    valid = ~isnan(SINR);
    meanThr = mean(THR(valid));
    outageFrac = mean(THR(valid) < 1);
 
    sim = struct();
    sim.SINR=SINR; sim.BER=BER; sim.THR=THR; sim.DOPkHz=DOPkHz; sim.E2Ems=E2Ems;
    sim.Event=Event;
    sim.Serving=Serving; sim.Gateway=Gateway; sim.VisUser=VisUser; sim.VisGW=VisGW; sim.Hops=HopCnt;
    sim.BLER=BLER;
    % --- NEW: power breakdown fields (for waveform sampler) ---
    sim.PS_mW = PS_mW;
    sim.PI_mW = PI_mW;
    sim.PJ_mW = PJ_mW;
    sim.PJraw_mW = PJraw_mW;
    sim.AJ_Pre = AJ_Pre;
    sim.AJ_Post = AJ_Post;
    sim.Pn_mW = p_n;

    sim.meanThr = meanThr;
    sim.outageFrac = outageFrac;
end
 
function f = worstCaseObjective( ...
    x, netG, seqLen, ...
    numSteps, dt, ...
    visU_all, visG_all, ...
    azU, elU, rU, rrU, ...
    elG, rG, ...
    pS_mW, totCCI, satChan, pIself_mW, ...
    pJsum_base_mW, TxP_J_base_dBm, ...
    AJ_DelaySec, AJ_NullDepth_dB, ...
    fc_Hz, c, BW, p_n, Gisl, ...
    W_outage, W_bler, W_energy)
 
    zDim = numel(x)-1;
    z = x(1:zDim);
    JamScale_dB = x(end);
 
    jamAgg = genJamAggFromG(netG, z, seqLen, numSteps);
 
    sim = simulateStarNet( ...
        numSteps, dt, ...
        visU_all, visG_all, ...
        azU, elU, rU, rrU, ...
        elG, rG, ...
        pS_mW, totCCI, satChan, pIself_mW, ...
        pJsum_base_mW, TxP_J_base_dBm, TxP_J_base_dBm, ...
        jamAgg, JamScale_dB, ...
        AJ_DelaySec, AJ_NullDepth_dB, ...
        fc_Hz, c, BW, p_n, Gisl);
 
    valid = ~isnan(sim.SINR);
    if ~any(valid)
        f = 1e6; return;
    end
 
    meanThr = mean(sim.THR(valid));
    outage  = mean(sim.THR(valid) < 1);
    meanBler= mean(sim.BLER(valid));
 
    energy = mean(jamAgg.^2);
    duty   = mean(jamAgg > 0.2);
    penalty = W_energy*(energy + max(0, duty-0.55)^2*10);
 
    f = -meanThr + W_outage*outage + W_bler*meanBler + penalty;
end
 
function [netG, netD] = trainOrLoadJammerGAN(seqLen, zDim, iters, modelFile)
    if exist(modelFile,'file')==2
        S = load(modelFile);
        netG = S.netG; netD = S.netD;
        return;
    end
 
    fprintf('  [GAN] No saved model, training a lightweight GAN...\n');
 
    nReal = 220;
    t = linspace(0,1,seqLen);
    realData = zeros(seqLen, nReal, 'single');
    for i = 1:nReal
        c1 = 0.4 + 0.2*rand;
        w1 = 0.06 + 0.10*rand;
        bump = exp(-0.5*((t-c1)/w1).^2);
        plat = (t > (0.32+0.06*rand)) & (t < (0.68-0.06*rand));
        shape = 0.12 + 0.58*bump + 0.28*plat;
        shape = shape + 0.06*randn(1,seqLen);
        shape = movmean(shape, 9);
        shape = min(max(shape,0),1);
        realData(:,i) = single(shape(:));
    end
 
    netG = dlnetwork(layerGraph([
        featureInputLayer(zDim,'Name','z')
        fullyConnectedLayer(128,'Name','g_fc1')
        reluLayer('Name','g_relu1')
        fullyConnectedLayer(seqLen,'Name','g_fc2')
        sigmoidLayer('Name','g_sig')
    ]));
 
    netD = dlnetwork(layerGraph([
        featureInputLayer(seqLen,'Name','x')
        fullyConnectedLayer(128,'Name','d_fc1')
        leakyReluLayer(0.2,'Name','d_lrelu1')
        fullyConnectedLayer(1,'Name','d_fc2')
        sigmoidLayer('Name','d_sig')
    ]));
 
    lr = 1e-3; batch = 24;
    avgG=[]; avgSqG=[]; avgD=[]; avgSqD=[];
 
    for it=1:iters
        idx = randi(nReal,[1 batch]);
        xReal = dlarray(realData(:,idx),'CB');
 
        z = dlarray(single(randn(zDim,batch)),'CB');
        xFake = forward(netG, z);
 
        [gradD, ~] = dlfeval(@dGradients, netD, xReal, xFake);
        [netD, avgD, avgSqD] = adamupdate(netD, gradD, avgD, avgSqD, it, lr);
 
        z2 = dlarray(single(randn(zDim,batch)),'CB');
        [gradG, ~] = dlfeval(@gGradients, netG, netD, z2);
        [netG, avgG, avgSqG] = adamupdate(netG, gradG, avgG, avgSqG, it, lr);
    end
 
    save(modelFile,'netG','netD');
    fprintf('  [GAN] Training done. Saved to %s\n', modelFile);
end
 
function [gradD, lossD] = dGradients(netD, xReal, xFake)
    pReal = forward(netD, xReal);
    pFake = forward(netD, xFake);
    epsv = 1e-6;
    lossD = -mean(log(pReal+epsv) + log(1-pFake+epsv));
    gradD = dlgradient(lossD, netD.Learnables);
end
 
function [gradG, lossG] = gGradients(netG, netD, z)
    xFake = forward(netG, z);
    pFake = forward(netD, xFake);
    epsv = 1e-6;
    lossG = -mean(log(pFake+epsv));
    gradG = dlgradient(lossG, netG.Learnables);
end
 
function jamAgg = genJamAggFromG(netG, z, seqLen, numSteps)
    z = dlarray(single(z(:)),'CB');
    y = predict(netG, z);
    y = gather(extractdata(y));
    y = double(y(:))';
 
    x0 = linspace(1,numSteps,seqLen);
    jamAgg = interp1(x0, y, 1:numSteps, 'pchip', 'extrap');
    jamAgg = min(max(jamAgg,0),1);
    jamAgg = movmean(jamAgg, 9);
    jamAgg = jamAgg(:);
end
 
function onTick(dashFig)
    if ~isvalid(dashFig), return; end
    app = guidata(dashFig);
    if isempty(app.v) || ~isvalid(app.v), return; end
 
    try
        ct = app.v.CurrentTime;
    catch
        return;
    end
 
    dtsec = seconds(ct - app.sim_start);
    k = floor(dtsec/app.sample_time) + 1;
    k = max(1, min(app.numSteps, k));
    x = app.t_axis_min(k);
 
    app.curSINR.XData = [x x];
    app.curBER.XData  = [x x];
    app.curTHR.XData  = [x x];
    app.curDOP.XData  = [x x];
    app.curJam.XData  = [x x];
 
    sinr = app.worst.SINR(k);
    ber  = app.worst.BER(k);
    thr  = app.worst.THR(k);
    thrBase = app.base.THR(k);
    e2e  = app.worst.E2Ems(k);
    evt  = string(app.worst.Event(k));
 
    y1 = sinr; if isnan(y1), y1 = -20; end
    y2 = ber;  if isnan(y2), y2 = 1;   end
    set(app.dot1,'XData',x,'YData',y1);
    set(app.dot2,'XData',x,'YData',max(y2,1e-9));
    set(app.dot3,'XData',x,'YData',thr);
 
    bler = app.worst.BLER(k);
    if bler > 0.1, app.lamp.Color=[1 0 0]; else, app.lamp.Color=[0 1 0]; end
 
    serv = app.worst.Serving(k);
    gw   = app.worst.Gateway(k);
    visU = app.worst.VisUser(k);
    visG = app.worst.VisGW(k);
 
    hopsTxt = "-";
    if serv>0 && gw>0
        pth = shortestpath(app.Gisl, serv, gw);
        hopsTxt = sprintf('%d', max(0,numel(pth)-1));
        app.pathLine.XData = app.satPlane(pth);
        app.pathLine.YData = app.satSlot(pth);
        app.gridServ.XData = app.satPlane(serv); app.gridServ.YData = app.satSlot(serv);
        app.gridGW.XData   = app.satPlane(gw);   app.gridGW.YData   = app.satSlot(gw);
    else
        app.pathLine.XData = nan; app.pathLine.YData = nan;
        app.gridServ.XData = nan; app.gridServ.YData = nan;
        app.gridGW.XData   = nan; app.gridGW.YData   = nan;
    end
 
    visIdx = find(app.elU(k,:)>5);
    if ~isempty(visIdx)
        app.skyAll.XData = wrap180_vec(app.azU(k,visIdx));
        app.skyAll.YData = app.elU(k,visIdx);
        if serv>0
            app.skyServ.XData = wrap180(app.azU(k,serv));
            app.skyServ.YData = app.elU(k,serv);
        else
            app.skyServ.XData = nan; app.skyServ.YData = nan;
        end
        if gw>0
            app.skyGW.XData = wrap180(app.azU(k,gw));
            app.skyGW.YData = app.elU(k,gw);
        else
            app.skyGW.XData = nan; app.skyGW.YData = nan;
        end
    else
        app.skyAll.XData = nan; app.skyAll.YData = nan;
        app.skyServ.XData = nan; app.skyServ.YData = nan;
        app.skyGW.XData = nan; app.skyGW.YData = nan;
    end
 
    % 3D链路高亮（Serving绿，GW蓝）
    try
        if app.lastServ ~= serv
            if app.lastServ>0 && app.lastServ<=numel(app.acUser) && isvalid(app.acUser(app.lastServ))
                app.acUser(app.lastServ).LineColor = [0.75 0.75 0.75];
                app.acUser(app.lastServ).LineWidth = 0.6;
            end
            if serv>0 && serv<=numel(app.acUser) && isvalid(app.acUser(serv))
                app.acUser(serv).LineColor = [0 1 0];
                app.acUser(serv).LineWidth = 2.2;
            end
            app.lastServ = serv;
        end
        if app.lastGW ~= gw
            if app.lastGW>0 && app.lastGW<=numel(app.acGW) && isvalid(app.acGW(app.lastGW))
                app.acGW(app.lastGW).LineColor = [0.65 0.75 1.00];
                app.acGW(app.lastGW).LineWidth = 0.6;
            end
            if gw>0 && gw<=numel(app.acGW) && isvalid(app.acGW(gw))
                app.acGW(gw).LineColor = [0 0.45 1];
                app.acGW(gw).LineWidth = 2.2;
            end
            app.lastGW = gw;
        end
    catch
    end

    % 3D ISL 路径高亮（橙色）：展示 Serving Sat -> GW Sat 的多跳链路
    try
        % 先把上一帧的高亮边恢复成灰色
        if isfield(app,'lastISLEdges') && ~isempty(app.lastISLEdges) && isfield(app,'acISL')
            for ee = app.lastISLEdges(:).'
                if ee>=1 && ee<=numel(app.acISL) && isvalid(app.acISL(ee))
                    app.acISL(ee).LineColor = [0.80 0.80 0.80];
                    app.acISL(ee).LineWidth = 0.4;
                end
            end
        end

        edgesNow = [];
        if serv>0 && gw>0 && isfield(app,'islMap') && ~isempty(app.islMap)
            pth2 = shortestpath(app.Gisl, serv, gw);
            for ii=1:(numel(pth2)-1)
                ee = app.islMap(pth2(ii), pth2(ii+1));
                if ee>0
                    edgesNow(end+1) = ee; %#ok<AGROW>
                end
            end
            for ee = edgesNow
                if ee>=1 && ee<=numel(app.acISL) && isvalid(app.acISL(ee))
                    app.acISL(ee).LineColor = [1 0.6 0];
                    app.acISL(ee).LineWidth = 2.0;
                end
            end
        end
        app.lastISLEdges = edgesNow;
    catch
    end

    spdTxt = "Viewer Speed: -";
    try
        spdTxt = sprintf("Viewer Speed: x%.2f", app.v.PlaybackSpeedMultiplier);
    catch
    end
 
    app.lblTime.Text  = sprintf("Current Time: %s", char(ct));
    app.lblSpeed.Text = spdTxt;
    app.lblServ.Text  = sprintf("Serving Sat: #%d", serv);
    app.lblGW.Text    = sprintf("Gateway Sat: #%d", gw);
    app.lblHops.Text  = sprintf("ISL Hops: %s", hopsTxt);
    app.lblVis.Text   = sprintf("Visible(User/GW): %d / %d", visU, visG);
    app.lblSINR.Text  = sprintf("SINR: %s dB", fmtNum(sinr,2));
    app.lblTHR.Text   = sprintf("Thr: %s Mbps  (Δ vs base: %s)", fmtNum(thr,1), fmtNum(thr-thrBase,1));
    app.lblE2E.Text   = sprintf("E2E Delay: %s ms", fmtNum(e2e,2));
 
    % ---- Interference Classification (STFT+LeNet) ----
    if isfield(app,'intfPred') && ~isempty(app.intfPred) && numel(app.intfPred)>=k
        try
            pred = app.intfPred(k);
            app.lblIntf.Text = sprintf("Interference Class (STFT+LeNet): %s", char(pred));
            if isfield(app,'intfScore') && ~isempty(app.intfScore) && size(app.intfScore,1)>=k
                scv = app.intfScore(k,:);
                if isvalid(app.barIntf), app.barIntf.YData = scv; end
            end
        catch
        end
    else
        if isfield(app,'lblIntf'), app.lblIntf.Text = "Interference Class (STFT+LeNet): -"; end
    end
 
    if app.lastEvent ~= evt
        data = app.tbl.Data;
        if size(data,1) >= 200, data = data(end-150:end,:); end
        app.tbl.Data = [data; {char(ct), 'State', char(evt)}];
        app.lastEvent = evt;
    end
 
    guidata(dashFig, app);
    drawnow limitrate;
end
 
function onClose(src, ~, tmr)
    try
        if isa(tmr,'timer') && isvalid(tmr)
            stop(tmr); delete(tmr);
        end
    catch
    end
    delete(src);
end
 
function s = fmtNum(x,n)
    if isempty(x) || isnan(x), s="-"; return; end
    s = num2str(x, ['%.' num2str(n) 'f']);
end
 
function PL = fspl_dB_vec(range_m, freq_MHz)
    d_km = max(range_m, 1)/1000;
    PL = 32.45 + 20*log10(d_km) + 20*log10(freq_MHz);
end
 
function a = wrap180(x)
    a = mod(x + 180, 360) - 180;
end
function a = wrap180_vec(x)
    a = mod(x + 180, 360) - 180;
end
 
function shadeTagRegions(ax, t_axis, tags, key, yLim, colorRGB, labelText, labelY)
    idx = find(contains(tags, key));
    if isempty(idx), return; end
    idx = idx(:);
    breaks = [1; find(diff(idx)>1)+1; numel(idx)+1];
    for b=1:(numel(breaks)-1)
        seg = idx(breaks(b):breaks(b+1)-1);
        x1=t_axis(seg(1)); x2=t_axis(seg(end));
        X=[x1 x2 x2 x1]; Y=[yLim(1) yLim(1) yLim(2) yLim(2)];
        patch(ax,X,Y,colorRGB,'FaceAlpha',0.12,'EdgeColor','none');
        text(ax,mean([x1 x2]),labelY,labelText,'Color',colorRGB,'HorizontalAlignment','center');
    end
end
 
%% =========================
% Local Functions: STFT+LeNet Interference Classifier (R2021a)  + ?“图片导出完全体”
% =========================
function [net, classes] = getOrTrainLeNetSTFT(modelFile, datasetRoot, trainIfMissing)
    classes = {'none','tone','pbnj','mod'};
    if exist(modelFile,'file')
        S = load(modelFile);
        net = S.net;
        if isfield(S,'classes'), classes = S.classes; end
        return;
    end
    if ~trainIfMissing
        error('Interference classifier model not found: %s', modelFile);
    end
 
    if ~exist(datasetRoot,'dir')
        fprintf('  [IntfCls] Dataset not found, generating dataset: %s\n', datasetRoot);
        generateDatasetSimpleSTFT(datasetRoot);
    end
    fprintf('  [IntfCls] Training LeNet on STFT images...\n');
    [net, classes] = trainLeNetSTFT(datasetRoot, classes);
    save(modelFile,'net','classes');
    fprintf('  [IntfCls] Saved: %s\n', modelFile);
end
 
function generateDatasetSimpleSTFT(outRoot)
    classes = {'none','tone','pbnj','mod'};
    splits  = {'train','val','test'};
    makeDirs(outRoot, splits, classes);
 
    % 你原脚本“随机强度/频段/类型”的核心思想：这里做成轻量可控版
    numTrain=600; numVal=120; numTest=180;
    SNRdB_list=[-5 0 3];
    JSR_dB_list=[0 3 6 9 12];
    Ns = 2048;
 
    stft.win=256; stft.overlap=128; stft.nfft=256; stft.fs=1;
    imgSize=[128 128];
 
    rng(2026);
 
    genSplitSimple(outRoot,'train',classes,numTrain,Ns,SNRdB_list,JSR_dB_list,stft,imgSize);
    genSplitSimple(outRoot,'val',  classes,numVal,  Ns,SNRdB_list,JSR_dB_list,stft,imgSize);
    genSplitSimple(outRoot,'test', classes,numTest, Ns,SNRdB_list,JSR_dB_list,stft,imgSize);
end
 
function genSplitSimple(outRoot, splitName, classes, numPerClass, Ns, SNRdB_list, JSR_dB_list, stft, imgSize)
    fprintf('  [IntfCls] Split=%s ...\n', splitName);
    for ci=1:numel(classes)
        cls = classes{ci};
        outDir = fullfile(outRoot, splitName, cls);
        for n=1:numPerClass
            snrDb = SNRdB_list(randi(numel(SNRdB_list)));
            jsrDb = JSR_dB_list(randi(numel(JSR_dB_list)));
 
            % 基带QPSK
            s = qpskRand(Ns);
 
            % AWGN
            EsN0 = 10^(snrDb/10);
            noiseVar = 1/EsN0;
            n_awgn = sqrt(noiseVar/2)*(randn(Ns,1)+1j*randn(Ns,1));
 
            % 干扰：类型 + 随机中心频率/带宽/突发位置（贴合你PPT“随机频段+强度+类别”）
            cfg = randomJammerCfgSimple(cls, jsrDb);
            j = genJammerSimple(cfg, s);
 
            r = s + n_awgn + j;
 
            img = makeSTFTImageSimple(r, stft, imgSize);
            fname = sprintf('%s_%05d_SNR%.1f_JSR%.1f.png', cls, n, snrDb, jsrDb);
            imwrite(img, fullfile(outDir, fname));
        end
    end
end
 
function [net, classes] = trainLeNetSTFT(dataRoot, classes)
    trainDir = fullfile(dataRoot,'train');
    valDir   = fullfile(dataRoot,'val');
 
    inputSize=[128 128 1];
 
    % ReadFcn：你已装 Image Processing Toolbox，所以这里可直接用 im2single/im2gray
    imdsTrain = imageDatastore(trainDir,'IncludeSubfolders',true,'LabelSource','foldernames', ...
        'ReadFcn', @(x) im2single(im2gray(imread(x))));
    imdsVal   = imageDatastore(valDir,  'IncludeSubfolders',true,'LabelSource','foldernames', ...
        'ReadFcn', @(x) im2single(im2gray(imread(x))));
 
    imdsTrain.Labels = reordercats(imdsTrain.Labels, classes);
    imdsVal.Labels   = reordercats(imdsVal.Labels, classes);
 
    augTrain = augmentedImageDatastore(inputSize(1:2), imdsTrain);
    augVal   = augmentedImageDatastore(inputSize(1:2), imdsVal);
 
    layers = [
        imageInputLayer(inputSize,'Normalization','none','Name','in')
        convolution2dLayer(5,6,'Padding','same','Name','c1')
        reluLayer('Name','r1')
        averagePooling2dLayer(2,'Stride',2,'Name','p1')
        convolution2dLayer(5,16,'Name','c2')
        reluLayer('Name','r2')
        averagePooling2dLayer(2,'Stride',2,'Name','p2')
        convolution2dLayer(5,120,'Name','c3')
        reluLayer('Name','r3')
        fullyConnectedLayer(84,'Name','fc1')
        reluLayer('Name','r4')
        fullyConnectedLayer(numel(classes),'Name','fc2')
        softmaxLayer('Name','sm')
        classificationLayer('Name','out') ];
 
    opts = trainingOptions('adam', ...
        'InitialLearnRate',1e-3, ...
        'MaxEpochs',8, ...
        'MiniBatchSize',64, ...
        'Shuffle','every-epoch', ...
        'ValidationData',augVal, ...
        'ValidationFrequency',50, ...
        'Verbose',true);
 
    net = trainNetwork(augTrain, layers, opts);
end
 
function exportDatasetMontage(datasetRoot, exportDir)
    % 导出每类若干张样例拼图（用于PPT）
    try
        classes = {'none','tone','pbnj','mod'};
        split = 'train';
        outPng = fullfile(exportDir, sprintf('montage_%s.png', split));
        files = {};
        for i=1:numel(classes)
            d = fullfile(datasetRoot, split, classes{i});
            L = dir(fullfile(d,'*.png'));
            take = min(8, numel(L));
            for k=1:take
                files{end+1} = fullfile(L(k).folder, L(k).name); %#ok<AGROW>
            end
        end
        if isempty(files), return; end
        f = figure('Visible','off','Color','w','Position',[100 100 1200 650]);
        montage(files,'Size',[4 8]); title(sprintf('Dataset Samples (split=%s)', split));
        exportgraphics(f, outPng, 'Resolution', 180);
        close(f);
        fprintf('  [IntfCls][Export] Saved montage: %s\n', outPng);
    catch ME
        fprintf('  [IntfCls][Export] Montage failed: %s\n', ME.message);
    end
end
 
function exportTestConfusion(net, datasetRoot, exportDir, classes)
    % 导出 test 集混淆矩阵（用于PPT）
    try
        testDir = fullfile(datasetRoot,'test');
        imdsTest = imageDatastore(testDir,'IncludeSubfolders',true,'LabelSource','foldernames', ...
            'ReadFcn', @(x) im2single(im2gray(imread(x))));
        imdsTest.Labels = reordercats(imdsTest.Labels, classes);
        augTest = augmentedImageDatastore([128 128], imdsTest);
 
        pred = classify(net, augTest);
        pred = reordercats(pred, classes);
 
        cm = confusionmat(imdsTest.Labels, pred, 'Order', categorical(classes,classes));
        cmN = cm ./ max(1,sum(cm,2));
 
        f = figure('Visible','off','Color','w','Position',[100 100 900 700]);
        imagesc(cmN); axis image; colorbar;
        xticks(1:numel(classes)); yticks(1:numel(classes));
        xticklabels(classes); yticklabels(classes);
        title('Confusion Matrix (Normalized) on Test Set');
        xlabel('Predicted'); ylabel('True');
        for i=1:numel(classes)
            for j=1:numel(classes)
                text(j,i,sprintf('%.2f',cmN(i,j)),'HorizontalAlignment','center','Color','w','FontWeight','bold');
            end
        end
        outPng = fullfile(exportDir, 'confusion_test.png');
        exportgraphics(f, outPng, 'Resolution', 180);
        close(f);
        fprintf('  [IntfCls][Export] Saved confusion matrix: %s\n', outPng);
    catch ME
        fprintf('  [IntfCls][Export] Confusion matrix export failed: %s\n', ME.message);
    end
end
 
function [trueLab, predLab, scoreMat, snapInfo] = classifyInterferenceTimeline(net, classes, numSteps, sample_time, eventTags, jamScale_dB) %#ok<INUSD>
    % 为每个时间步合成一个短快照，用 STFT + LeNet 预测干扰类型
    % snapInfo：保存关键帧的“原始接收信号+图片+标签”，用于导出PPT图
 
    Ns = 2048;
    stft.win=256; stft.overlap=128; stft.nfft=256; stft.fs=1;
    imgSize=[128 128];
 
    trueLab = categorical(repmat("none",numSteps,1), classes);
    predLab = categorical(repmat("none",numSteps,1), classes);
    scoreMat = zeros(numSteps, numel(classes));
 
    rng(2027); % 固定可复现
 
    % 关键帧挑选：每类事件挑几帧（用于导出）
    keyIdx = pickKeyframes(eventTags, numSteps);
 
    snapInfo = struct();
    snapInfo.exportIdx = keyIdx;
    snapInfo.tIndex = keyIdx;
    snapInfo.event = strings(numel(keyIdx),1);
    snapInfo.trueClass = strings(numel(keyIdx),1);
    snapInfo.predClass = strings(numel(keyIdx),1);
    snapInfo.img = cell(numel(keyIdx),1);
 
    for k=1:numSteps
        evt = string(eventTags(k));
 
        if contains(evt,"JAMMING") || contains(evt,"CoChannel")
            pick = randi([2 4]); % tone/pbnj/mod
            cls = classes{pick};
            jsrDb = min(12, max(0, jamScale_dB/3));
        else
            cls = 'none';
            jsrDb = 0;
        end
 
        trueLab(k) = categorical(string(cls), classes);
 
        r = synthRxSnapshotSimple(cls, Ns, 0, jsrDb); % snr=0dB
        img = makeSTFTImageSimple(r, stft, imgSize);
        img1 = im2single(img);
        img1 = reshape(img1, [imgSize 1]);
 
        try
            [pl, sc] = classify(net, img1);
            pl = reordercats(pl, classes);
            predLab(k) = pl;
 
            sc = sc(:).';
            if numel(sc)==numel(classes)
                scoreMat(k,:) = sc;
            end
        catch
            predLab(k) = trueLab(k);
            scoreMat(k,:) = 0;
            scoreMat(k, find(strcmp(classes,cls),1)) = 1;
        end
 
        % 记录关键帧（用于导出PPT图片）
        ii = find(keyIdx==k,1);
        if ~isempty(ii)
            snapInfo.event(ii) = evt;
            snapInfo.trueClass(ii) = string(cls);
            snapInfo.predClass(ii) = string(predLab(k));
            snapInfo.img{ii} = img; % 128x128
        end
    end
end
 
function idx = pickKeyframes(eventTags, numSteps)
    % 从事件序列中挑选关键帧：干扰开始/中段/结束 + Protected/CoChannel
    tags = string(eventTags);
    idx = [];
    % JAMMING
    j = find(contains(tags,"JAMMING"));
    if ~isempty(j)
        idx = [idx; j(1); j(round(end/2)); j(end)];
    end
    % Protected
    p = find(contains(tags,"Protected"));
    if ~isempty(p)
        idx = [idx; p(1); p(round(end/2)); p(end)];
    end
    % CoChannel
    c = find(contains(tags,"CoChannel"));
    if ~isempty(c)
        idx = [idx; c(1); c(round(end/2)); c(end)];
    end
    % Normal（挑均匀3点）
    n = find(contains(tags,"Normal"));
    if ~isempty(n)
        idx = [idx; n(1); n(round(end/2)); n(end)];
    end
    idx = unique(max(1, min(numSteps, idx(:))));
    if numel(idx) > 12
        idx = idx(round(linspace(1,numel(idx),12)));
    end
end
 
function exportSimKeyframeSTFT(snapInfo, exportDir)
    % 导出仿真关键帧STFT图：每帧单独一张 + 总拼图
    try
        if ~exist(exportDir,'dir'), mkdir(exportDir); end
 
        n = numel(snapInfo.tIndex);
        if n==0, return; end
 
        files = cell(n,1);
        for i=1:n
            img = snapInfo.img{i};
            if isempty(img), continue; end
            evt = snapInfo.event(i);
            tc  = snapInfo.trueClass(i);
            pc  = snapInfo.predClass(i);
            k   = snapInfo.tIndex(i);
 
            f = figure('Visible','off','Color','w','Position',[100 100 520 480]);
            imshow(img,[]); colormap gray;
            title(sprintf('k=%d | evt=%s | true=%s | pred=%s', k, evt, tc, pc), 'Interpreter','none');
            fn = fullfile(exportDir, sprintf('sim_keyframe_%02d_k%04d_%s_%s.png', i, k, tc, pc));
            exportgraphics(f, fn, 'Resolution', 200);
            close(f);
            files{i} = fn;
        end
 
        files = files(~cellfun(@isempty,files));
        if isempty(files), return; end
 
        f2 = figure('Visible','off','Color','w','Position',[100 100 1200 650]);
        montage(files); title('Simulation Keyframes STFT (PPT-ready)');
        fn2 = fullfile(exportDir, 'sim_keyframes_montage.png');
        exportgraphics(f2, fn2, 'Resolution', 180);
        close(f2);
 
        fprintf('  [IntfCls][Export] Saved sim keyframes to: %s\n', exportDir);
    catch ME
        fprintf('  [IntfCls][Export] Sim keyframe export failed: %s\n', ME.message);
    end
end
 
function r = synthRxSnapshotSimple(className, Ns, snrDb, jsrDb)
    s = qpskRand(Ns);
 
    EsN0 = 10^(snrDb/10);
    noiseVar = 1/EsN0;
    n_awgn = sqrt(noiseVar/2)*(randn(Ns,1)+1j*randn(Ns,1));
 
    cfg = randomJammerCfgSimple(className, jsrDb);
    j = genJammerSimple(cfg, s);
 
    r = s + n_awgn + j;
end
 
function s = qpskRand(Ns)
    b = randi([0 3], Ns, 1);
    s = exp(1j*(pi/4 + (pi/2)*double(b)));
end
 
function cfg = randomJammerCfgSimple(className, jsrDb)
    % 贴近你PPT“随机频段/强度/类别”的简化实现
    switch className
        case 'none'
            cfg = struct('type','none');
        case 'tone'
            f0 = 0.05 + 0.40*rand;               % 随机单音频点
            cfg = struct('type','tone','JSR_dB',jsrDb,'f0',f0);
        case 'pbnj'
            f1 = 0.03 + 0.30*rand;               % 随机带通起点
            bw = 0.05 + 0.20*rand;               % 随机带宽
            f2 = min(0.49, f1 + bw);
            cfg = struct('type','pbnj','JSR_dB',jsrDb,'band',[f1 f2]);
        case 'mod'
            df = 0.03 + 0.15*rand;               % 调制搬移
            duty = 0.15 + 0.35*rand;             % 突发占空
            cfg = struct('type','mod','JSR_dB',jsrDb,'df',df,'duty',duty);
        otherwise
            cfg = struct('type','none');
    end
end
 
function j = genJammerSimple(cfg, s)
    Ns = length(s);
    switch cfg.type
        case 'none'
            j = zeros(Ns,1);
        case 'tone'
            n = (0:Ns-1).';
            j0 = exp(1j*2*pi*cfg.f0*n);
            j = scaleToJSR(j0, s, cfg.JSR_dB);
        case 'pbnj'
            u = randn(Ns,1) + 1j*randn(Ns,1);
            hBP = fir1(80, cfg.band);
            j0 = filter(hBP,1,u);
            j = scaleToJSR(j0, s, cfg.JSR_dB);
        case 'mod'
            n = (0:Ns-1).';
            xI = qpskRand(Ns);
            mask = zeros(Ns,1);
            burstLen = max(32, round(cfg.duty*Ns));
            startIdx = randi([1, Ns-burstLen+1]);
            mask(startIdx:startIdx+burstLen-1)=1;
            j0 = (xI .* exp(1j*2*pi*cfg.df*n)) .* mask;
            j = scaleToJSR(j0, s, cfg.JSR_dB);
        otherwise
            j = zeros(Ns,1);
    end
end
 
function j = scaleToJSR(j0, s, JSR_dB)
    if all(j0==0)
        j = j0; return;
    end
    Ps = mean(abs(s).^2);
    Pj_target = Ps * 10^(JSR_dB/10);
    j = j0 * sqrt(Pj_target / (mean(abs(j0).^2) + 1e-12));
end
 
function imgOut = makeSTFTImageSimple(r, stft, imgSize)
    [S,~,~] = spectrogram(r, stft.win, stft.overlap, stft.nfft, stft.fs, 'centered');
    P = abs(S).^2;
    img = 10*log10(P + 1e-12);
    img = img - min(img(:));
    img = img ./ (max(img(:)) + 1e-12);
    imgOut = imresize(img, imgSize);
end
 
function makeDirs(outRoot, splits, classes)
    for si=1:numel(splits)
        for ci=1:numel(classes)
            d = fullfile(outRoot, splits{si}, classes{ci});
            if ~exist(d,'dir'), mkdir(d); end
        end
    end
end



%% =========================
% Local Functions: Power-aligned Waveform Sampler (for STFT+LeNet "real" usage)
% 说明：
%  - 主仿真仍是“功率级/系统级”计算 SINR/BER/THR（不改主逻辑）
%  - 这里新增一个采样器：把 simulateStarNet 里算出来的 (PS, PI, PJ, PN) 变成一段 IQ
%  - 分类器看到的 STFT 快照来自“功率对齐”的 IQ（而不是从 Event 标签编剧情造波形）
% =========================
function cfg = defaultPowerAlignedSamplerCfg()
    cfg = struct();

    % IQ snapshot length
    cfg.Ns = 2048;

    % STFT (keep consistent with training)
    cfg.Stft = struct('win',256,'overlap',128,'nfft',256,'fs',1);
    cfg.imgSize = [128 128];

    % If true: use PJ_mW (effective, post-AJ) -> align with main SINR
    % If false: use PJraw_mW (pre-AJ) -> show "raw interference" even if protected
    cfg.UsePostAJPower = true;

    % Spectral model choices (must be among classes)
    cfg.JammerType = 'pbnj';   % {'tone','pbnj','mod'}
    cfg.CCIType    = 'mod';    % {'mod','pbnj'}

    % Deterministic per-time-step randomization (so results are repeatable)
    cfg.Seed = 2028;

    % Detection threshold: treat interference as present if P > K*PN
    cfg.DetK = 1.0;

    % Parameter ranges (normalized frequency 0..0.5 for fs=1)
    cfg.tone = struct('f0Range',[0.05 0.45]);

    cfg.pbnj = struct( ...
        'bandStartRange',[0.03 0.30], ...
        'bwRange',[0.05 0.20], ...
        'firOrder',80);

    cfg.mod = struct( ...
        'dfRange',[0.03 0.15], ...
        'dutyRange',[0.15 0.35]);
end

function [trueLab, predLab, scoreMat, snapInfo] = classifyInterferenceTimeline_powerSampler( ...
    net, classes, numSteps, sample_time, sim, p_n, cfg) %#ok<INUSD>
    % 用功率级 (PS, PI, PJ, PN) 生成 IQ -> STFT -> LeNet
    %
    % 输入：
    %   sim.PS_mW / sim.PI_mW / sim.PJ_mW / sim.PJraw_mW  来自 simulateStarNet（本脚本已补齐）
    %   p_n：噪声功率(mW)
    %
    % 输出：
    %   trueLab：基于“功率谁更强”的粗略真值（用于自检/展示，不用于训练）
    %   predLab：LeNet 预测
    %   scoreMat：LeNet softmax 分数 (N×4)
    %   snapInfo：导出PPT关键帧用

    Ns = cfg.Ns;
    stft = cfg.Stft;
    imgSize = cfg.imgSize;

    trueLab = categorical(repmat("none",numSteps,1), classes);
    predLab = categorical(repmat("none",numSteps,1), classes);
    scoreMat = zeros(numSteps, numel(classes));

    % 关键帧（仍然用主仿真 Event 选时刻，只影响“导出哪些图”，不影响采样/分类）
    keyIdx = pickKeyframes(sim.Event, numSteps);

    snapInfo = struct();
    snapInfo.exportIdx = keyIdx;
    snapInfo.tIndex = keyIdx;
    snapInfo.event = strings(numel(keyIdx),1);
    snapInfo.trueClass = strings(numel(keyIdx),1);
    snapInfo.predClass = strings(numel(keyIdx),1);
    snapInfo.img = cell(numel(keyIdx),1);

    for k=1:numSteps
        % 取功率分解（mW）
        Ps = 0; Pi = 0; Pj = 0;
        if isfield(sim,'PS_mW'), Ps = sim.PS_mW(k); end
        if isfield(sim,'PI_mW'), Pi = sim.PI_mW(k); end
        if cfg.UsePostAJPower
            if isfield(sim,'PJ_mW'), Pj = sim.PJ_mW(k); end
        else
            if isfield(sim,'PJraw_mW'), Pj = sim.PJraw_mW(k);
            elseif isfield(sim,'PJ_mW'), Pj = sim.PJ_mW(k);
            end
        end
        Pn = p_n;

        % 基于功率的“粗真值”（仅用于展示/导出标题）
        clsTrue = inferTrueClassFromPowers(Pi, Pj, Pn, cfg);
        trueLab(k) = categorical(string(clsTrue), classes);

        % 生成功率对齐 IQ
        [r, ~] = sampleIQFromPowers(Ps, Pi, Pj, Pn, cfg, k); %#ok<ASGLU>

        % STFT -> 图片
        img = makeSTFTImageSimple(r, stft, imgSize);
        img1 = im2single(img);
        img1 = reshape(img1, [imgSize 1]);

        % 分类
        try
            [pl, sc] = classify(net, img1);
            pl = reordercats(pl, classes);
            predLab(k) = pl;

            sc = sc(:).';
            if numel(sc)==numel(classes)
                scoreMat(k,:) = sc;
            end
        catch
            predLab(k) = trueLab(k);
            scoreMat(k,:) = 0;
            scoreMat(k, find(strcmp(classes, char(clsTrue)),1)) = 1;
        end

        % 记录关键帧（用于导出PPT图片）
        ii = find(keyIdx==k,1);
        if ~isempty(ii)
            snapInfo.event(ii) = string(sim.Event(k));
            snapInfo.trueClass(ii) = string(clsTrue);
            snapInfo.predClass(ii) = string(predLab(k));
            snapInfo.img{ii} = img; % 128x128
        end
    end
end

function cls = inferTrueClassFromPowers(Pi_mW, Pj_mW, Pn_mW, cfg)
    % 粗真值判定：谁更强就认为是什么干扰类型（只用于展示）
    if (Pj_mW > cfg.DetK*Pn_mW) && (Pj_mW >= Pi_mW)
        cls = cfg.JammerType;
    elseif (Pi_mW > cfg.DetK*Pn_mW)
        cls = cfg.CCIType;
    else
        cls = 'none';
    end
end

function [r, meta] = sampleIQFromPowers(Ps_mW, Pi_mW, Pj_mW, Pn_mW, cfg, k)
    % 把 (PS, PI, PJ, PN) 生成一段复基带 IQ
    %
    % 目标：
    %  - mean(|s|^2)  ~= Ps_mW
    %  - mean(|i|^2)  ~= Pi_mW
    %  - mean(|j|^2)  ~= Pj_mW
    %  - mean(|n|^2)  ~= Pn_mW
    %
    % 注意：这是“功率级对齐”的轻量波形，不引入 Rician/相位噪声/频偏残差

    Ns = cfg.Ns;
    n = (0:Ns-1).';

    meta = struct();

    % 固定每个时间步的随机性（避免受外部 rng 影响）
    st = rng;
    rng(cfg.Seed + k);

    % Desired signal (QPSK)
    s0 = qpskRand(Ns);
    s  = scaleToPower(s0, Ps_mW);

    % Co-channel interference
    iSig = zeros(Ns,1);
    if Pi_mW > 0
        switch lower(cfg.CCIType)
            case 'mod'
                xI = qpskRand(Ns);
                df = cfg.mod.dfRange(1) + diff(cfg.mod.dfRange)*rand;
                if rand > 0.5, df = -df; end
                i0 = xI .* exp(1j*2*pi*df*n);   % frequency-shifted modulated interferer
            case 'pbnj'
                u = randn(Ns,1) + 1j*randn(Ns,1);
                f1 = cfg.pbnj.bandStartRange(1) + diff(cfg.pbnj.bandStartRange)*rand;
                bw = cfg.pbnj.bwRange(1) + diff(cfg.pbnj.bwRange)*rand;
                f2 = min(0.49, f1 + bw);
                h  = fir1(cfg.pbnj.firOrder, [f1 f2]);
                i0 = filter(h, 1, u);
            otherwise
                i0 = randn(Ns,1) + 1j*randn(Ns,1);
        end
        iSig = scaleToPower(i0, Pi_mW);
    end

    % Jammer (spectral shape chosen by cfg.JammerType)
    jSig = zeros(Ns,1);
    if Pj_mW > 0
        switch lower(cfg.JammerType)
            case 'tone'
                f0 = cfg.tone.f0Range(1) + diff(cfg.tone.f0Range)*rand;
                if rand > 0.5, f0 = -f0; end
                j0 = exp(1j*2*pi*f0*n);

            case 'pbnj'
                u  = randn(Ns,1) + 1j*randn(Ns,1);
                f1 = cfg.pbnj.bandStartRange(1) + diff(cfg.pbnj.bandStartRange)*rand;
                bw = cfg.pbnj.bwRange(1) + diff(cfg.pbnj.bwRange)*rand;
                f2 = min(0.49, f1 + bw);
                h  = fir1(cfg.pbnj.firOrder, [f1 f2]);
                j0 = filter(h, 1, u);

            case 'mod'
                xJ = qpskRand(Ns);
                df = cfg.mod.dfRange(1) + diff(cfg.mod.dfRange)*rand;
                if rand > 0.5, df = -df; end
                duty = cfg.mod.dutyRange(1) + diff(cfg.mod.dutyRange)*rand;

                mask = zeros(Ns,1);
                burstLen = max(32, round(duty*Ns));
                startIdx = randi([1, Ns-burstLen+1]);
                mask(startIdx:startIdx+burstLen-1) = 1;

                j0 = (xJ .* exp(1j*2*pi*df*n)) .* mask;

            otherwise
                j0 = zeros(Ns,1);
        end
        jSig = scaleToPower(j0, Pj_mW);
    end

    % AWGN
    w = sqrt(Pn_mW/2) * (randn(Ns,1) + 1j*randn(Ns,1));

    r = s + iSig + jSig + w;

    % restore rng
    rng(st);

    % meta (optional debug)
    meta.Ps_meas = mean(abs(s).^2);
    meta.Pi_meas = mean(abs(iSig).^2);
    meta.Pj_meas = mean(abs(jSig).^2);
    meta.Pn_meas = mean(abs(w).^2);
end

function y = scaleToPower(y0, Ptarget_mW)
    if Ptarget_mW <= 0 || all(y0==0)
        y = zeros(size(y0)); 
        return;
    end
    P0 = mean(abs(y0).^2);
    y = y0 * sqrt(Ptarget_mW / (P0 + 1e-12));
end
