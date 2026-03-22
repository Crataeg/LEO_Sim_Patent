%% LEO StarNet EMC + GA(GAN) Worst-case + 3D Linked Dashboard (R2021a)  [V6.1]
% 修复/增强：
%  1) 修复 shadeTagRegions horzcat 维度不一致（idx行列向量不一致导致）
%  2) 加强Jammer有效性：提高TxP_J_base、提高旁瓣/远离主瓣干扰水平、提高JamScale上限
%  3) Dashboard + 3D联动：只读 v.CurrentTime，竖线同步；高亮Serving/GW链路

clear; clc; close all;
rng(7);

fprintf('============================================================\n');
fprintf('  LEO StarNet EMC | Worst-case Search (GA on GAN) | R2021a V6.1\n');
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
TxP_J_base_dBm = 88;            % ★提高 Jammer 基础功率（原75 -> 88）
G_rx_dB   = 35;
G_int_penalty_dB = 10;

elMaskDeg     = 5;
reuseK        = 4;

% ====== Jammer角度模式（连续存在，强度随 off-axis 衰减） ======
J_main_deg = 3;
J_side_deg = 20;
J_main_gain_dB  = 0;
J_side_gain_dB  = -8;           % ★提高旁瓣水平（原-18 -> -8）
J_floor_gain_dB = -12;          % ★提高远离主瓣底噪（原-28 -> -12）
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
JamScale_ub_dB = 35;            % ★提高上限（原25 -> 35）

z_lb = -2; z_ub = 2;

W_outage  = 300;
W_bler    = 80;
W_energy  = 80;

SINR_YLIM = [-20 50];
THR_YLIM  = [0 600];

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

% Jammer（增加数量可更容易打出最坏情况；这里适度增加到6个）
numJam = 6;   % ★原3 -> 6
satJam = cell(1, numJam);
for j = 1:numJam
    raanJ = (j-1)*(360/numJam);
    taJ   = 180 + 10*j;
    satJam{j} = satellite(sc, a, ecc, incDeg, raanJ, 0, taJ, 'Name', sprintf('JAMMER_%d', j));
    try
        sj = conicalSensor(satJam{j}, 'MaxViewAngle', 15);
        fov = fieldOfView(sj); fov.LineColor = [1 0 0];
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
% PART 6: 3D Viewer 链路显示（access）
% =========================
fprintf('Step 7: 构建3D Viewer链路显示（access lines）...\n');

acUser = gobjects(1,Nsat);
acGW   = gobjects(1,Nsat);
for i = 1:Nsat
    try
        acUser(i) = access(satConst{i}, gsUser); acUser(i).LineColor = [0.78 0.78 0.78]; acUser(i).LineWidth = 0.5;
        acGW(i)   = access(satConst{i}, gsGW);   acGW(i).LineColor   = [0.82 0.82 0.82]; acGW(i).LineWidth   = 0.5;
    catch
    end
end

%% =========================
% PART 7: Dashboard + 3D联动
% =========================
fprintf('Step 8: 构建Dashboard + 3D联动...\n');

dash = uifigure('Name','LEO StarNet EMC | GA+GAN Worst-case | Linked to 3D Viewer', ...
    'Color','w','Position',[40 40 1500 900]);

gl = uigridlayout(dash, [4 4]);
gl.RowHeight   = {36,'1x','1x','1x'};
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
axTHR  = uiaxes(gl); axTHR.Layout.Row=4; axTHR.Layout.Column=[1 2];
axDOP  = uiaxes(gl); axDOP.Layout.Row=4; axDOP.Layout.Column=3;

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

right = uigridlayout(gl, [16 1]);
right.Layout.Row = [2 4];
right.Layout.Column = 4;
right.RowHeight = {24,24,24,24,24,24,24,24,28,150,150,90,'1x',24,24,24};
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

app = struct();
app.sim_start = sim_start;
app.sample_time = sample_time;
app.numSteps = numSteps;
app.t_axis_min = t_axis_min;

app.base = simBase;
app.worst= simWorst;

app.azU = azU; app.elU = elU;
app.satPlane = satPlane; app.satSlot = satSlot;
app.Gisl = Gisl;

app.curSINR = curSINR; app.curBER = curBER; app.curTHR = curTHR; app.curDOP = curDOP; app.curJam = curJam;
app.dot1 = dot1; app.dot2 = dot2; app.dot3 = dot3;

app.lblTime=lblTime; app.lblSpeed=lblSpeed; app.lblServ=lblServ; app.lblGW=lblGW;
app.lblHops=lblHops; app.lblVis=lblVis; app.lblSINR=lblSINR; app.lblTHR=lblTHR; app.lblE2E=lblE2E;
app.lamp=lamp; app.tbl=tbl;

app.skyAll=skyAll; app.skyServ=skyServ; app.skyGW=skyGW;
app.gridServ=gridServ; app.gridGW=gridGW; app.pathLine=pathLine;

app.acUser = acUser; app.acGW = acGW;
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

    BLK = 1024;

    AJ_Active = false;
    AJ_Timer  = 0;

    JammerOn = (TxP_J_effBase_dBm > -100);

    for k = 1:numSteps
        visU = visU_all(k,:);
        visG = visG_all(k,:);
        VisUser(k) = sum(visU);
        VisGW(k)   = sum(visG);

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
        jamPowerCand = zeros(size(cand));

        for ii=1:numel(cand)
            si = cand(ii);

            p_s = pS_mW(k,si);

            ch = satChan(si);
            p_i = totCCI(k,ch) - pIself_mW(k,si);
            if p_i < 0, p_i = 0; end

            p_j = 0;
            if JammerOn && pJsum_base_mW(k,si) > 0
                baseShift_dB = TxP_J_effBase_dBm - TxP_J_base_dBm;
                jamScaleLin = 10.^(((baseShift_dB + JamScale_dB*jamAgg(k))/10));
                p_j = pJsum_base_mW(k,si) * jamScaleLin;
                if AJ_Active
                    p_j = p_j * 10^(-AJ_NullDepth_dB/10);
                end
            end
            jamPowerCand(ii) = p_j;

            sinrLin = p_s / (p_i + p_j + p_n);
            sinrCand(ii) = 10*log10(sinrLin);
        end

        if AJ_Active
            [~,best] = max(sinrCand);
        else
            [~,best] = max(elU(k,cand));
        end
        servIdx = cand(best);
        Serving(k) = servIdx;

        jamOn = JammerOn && (jamPowerCand(best) > 1.0*p_n) && (jamAgg(k) > 0.05); % ★门限更严格，避免“弱干扰也触发”

        if jamOn
            AJ_Timer = AJ_Timer + dt;
            if AJ_Timer >= AJ_DelaySec
                AJ_Active = true;
            end
        else
            AJ_Timer = 0;
            AJ_Active = false;
        end

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
                app.acUser(app.lastServ).LineColor = [0.78 0.78 0.78];
                app.acUser(app.lastServ).LineWidth = 0.5;
            end
            if serv>0 && serv<=numel(app.acUser) && isvalid(app.acUser(serv))
                app.acUser(serv).LineColor = [0 1 0];
                app.acUser(serv).LineWidth = 2.2;
            end
            app.lastServ = serv;
        end
        if app.lastGW ~= gw
            if app.lastGW>0 && app.lastGW<=numel(app.acGW) && isvalid(app.acGW(app.lastGW))
                app.acGW(app.lastGW).LineColor = [0.82 0.82 0.82];
                app.acGW(app.lastGW).LineWidth = 0.5;
            end
            if gw>0 && gw<=numel(app.acGW) && isvalid(app.acGW(gw))
                app.acGW(gw).LineColor = [0 0.45 1];
                app.acGW(gw).LineWidth = 2.2;
            end
            app.lastGW = gw;
        end
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

    % ✅ 关键修复：统一为列向量，breaks 用列拼接，避免 horzcat 维度不一致
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
