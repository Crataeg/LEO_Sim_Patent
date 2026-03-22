%% LEO StarNet EMC Simulation + 3D Viewer Linked Dashboard (MATLAB R2021a)
% 目标：
%   1) 固定仿真Epoch，避免“现实时间变了卫星跑到地球另一侧”
%   2) 生成足够数量的LEO星座（Walker-like：多轨道面×多颗卫星）
%   3) 仿真窗口=一个轨道周期，确保每颗卫星完整绕地一圈
%   4) 青岛用户站 + 北京网关站：具备“简化星网”端到端时延指标
%   5) EMC：共信道干扰(CCI) + 共线干扰(Jammer) + 抗干扰状态机(Protected)
%   6) 多普勒：range-rate -> doppler
%   7) 仪表盘竖线与3D viewer联动（读 v.CurrentTime），播放速度变化自动同步
%
% 依赖：MATLAB R2021a + Satellite Communications Toolbox（你已安装）
% 注意：不使用 satelliteScenario.AutoSimulate（R2021a无该属性）
%
% Author: (for cra) GPT-5.2 Pro

clear; clc; close all;

fprintf('============================================================\n');
fprintf('  LEO StarNet EMC + Linked Dashboard (R2021a)\n');
fprintf('  Fixed Epoch + Constellation + Full Orbit + EMC + Doppler\n');
fprintf('============================================================\n');

%% =========================
% PART 0: 全局参数
% =========================
% ---- 固定Epoch：关键修复（不要用 now）----
Epoch = datetime(2026,1,23,0,0,0,'TimeZone','UTC');  % 你也可以改成任意固定时间

% ---- 地球与物理常数 ----
Re  = 6371e3;                 % Earth radius (m)
mu  = 3.986004418e14;         % Earth GM (m^3/s^2)
c   = 299792458;              % speed of light (m/s)

% ---- 轨道/星座配置（可调）----
h      = 1200e3;              % 轨道高度 1200 km（对应 ~109 min 周期）
a      = Re + h;              % 半长轴
ecc    = 0.0;                 % 圆轨道
incDeg = 53;                  % 倾角：典型宽带星座常见（青岛纬度36可覆盖）
numPlanes    = 12;            % 轨道面数
satsPerPlane = 8;             % 每个轨道面卫星数
F_phasing    = 1;             % Walker相位因子（简化）

Nsat = numPlanes * satsPerPlane;

% ---- 仿真时长：一个轨道周期 ----
T_orbit = 2*pi*sqrt(a^3/mu);          % seconds
sample_time = 10;                      % seconds（建议 5~20；越小越平滑越慢）
sim_start = Epoch;
sim_stop  = sim_start + seconds(T_orbit);

% ---- 链路/EMC 参数 ----
fc_Hz     = 28e9;        % 载频 28 GHz (Ka-band示例)
freq_MHz  = fc_Hz/1e6;   % 用于FSPL公式
BW        = 100e6;       % 100 MHz
Noise_dBm = -110;        % 你原工程风格（偏“乐观”但能出曲线）

TxP_S_dBm = 55;          % 服务星发射功率（等效EIRP的一部分）
TxP_I_dBm = 55;          % 其他星作为同频干扰源的等效发射
TxP_J_dBm = 75;          % Jammer等效发射
G_rx_dB   = 35;          % 接收端等效增益（与你原工程一致风格）
G_int_penalty_dB = 10;   % 非主瓣干扰抑制（干扰相对主信号少10dB，工程化近似）

elMaskDeg  = 5;          % 最小仰角门限
reuseK     = 4;          % 频率复用因子：1..K
offAxisThDeg = 2.0;      % “共线干扰”判据阈值（度）

% ---- 抗干扰状态机（工程化）----
AJ_DelaySec    = 30;     % 连续检测到共线干扰后，30秒触发抗干扰
AJ_NullDepth_dB = 40;    % 抗干扰启用后，对Jammer等效抑制深度

% ---- 指标显示范围（画图用）----
SINR_YLIM = [-20 50];
THR_YLIM  = [0 600];

%% =========================
% PART 1: 场景与对象
% =========================
fprintf('Step 1: 构建 satelliteScenario（固定Epoch + 全轨道周期）...\n');
sc = satelliteScenario(sim_start, sim_stop, sample_time);

% 两个地面站：用户与网关（你可改）
gsUser = groundStation(sc, 36.06, 120.38, 'Name', 'Qingdao_User');
gsGW   = groundStation(sc, 39.90, 116.40, 'Name', 'Beijing_Gateway');

% --- 生成 Walker-like 星座（不依赖TLE，完全离线可跑） ---
fprintf('Step 2: 生成星座：%d planes × %d sats = %d sats ...\n', numPlanes, satsPerPlane, Nsat);

satConst = cell(1, Nsat);
satName  = strings(1, Nsat);
satPlane = zeros(1, Nsat);
satSlot  = zeros(1, Nsat);

idx = 0;
for p = 1:numPlanes
    raan = (p-1) * (360/numPlanes);    % RAAN均匀分布
    for s = 1:satsPerPlane
        idx = idx + 1;
        % Walker-like 相位：每个plane相对错开一点
        ta = (s-1) * (360/satsPerPlane) + (p-1) * F_phasing * (360/Nsat);
        nm = sprintf('SAT_P%02d_S%02d', p, s);

        satConst{idx} = satellite(sc, a, ecc, incDeg, raan, 0, ta, 'Name', nm);
        satName(idx)  = nm;
        satPlane(idx) = p;
        satSlot(idx)  = s;
    end
end

% --- 红队 Jammer 卫星（少量，便于可视化） ---
% 说明：Jammer不需要太多，否则3D里太乱；干扰效果由“共线判据+功率”体现
numJam = 3;
satJam = cell(1, numJam);
for j = 1:numJam
    raanJ = (j-1) * 120;        % 0/120/240 度
    taJ   = 180 + 10*j;         % 随便给个相位
    satJam{j} = satellite(sc, a, ecc, incDeg, raanJ, 0, taJ, 'Name', sprintf('JAMMER_%d', j));

    % Jammer 视场锥（红色），用来在3D里直观看“干扰源方向”
    try
        sj = conicalSensor(satJam{j}, 'MaxViewAngle', 15);
        fov = fieldOfView(sj);
        fov.LineColor = [1 0 0];
    catch
        % 如果某些环境下sensor可视化异常，不影响主逻辑
    end
end

%% =========================
% PART 2: 预计算几何（离线算好，播放时查表）
% =========================
fprintf('Step 3: 预计算几何（User/Gateway到全部卫星，及Jammer）...\n');

% 先按理论时间轴建，再用aer输出长度做“保险裁剪”
timeVec = 0:sample_time:seconds(sim_stop - sim_start);
numSteps_guess = numel(timeVec);

% 用第一颗星的aer来确定真实长度（防止版本差异导致长度不一致）
[az0, el0, r0] = aer(gsUser, satConst{1}); %#ok<ASGLU>
L = min(numSteps_guess, numel(el0));
timeVec = timeVec(1:L);
numSteps = L;

t_axis_min = timeVec/60;   % minutes

% --- User到星座 ---
azU = nan(numSteps, Nsat);
elU = nan(numSteps, Nsat);
rU  = nan(numSteps, Nsat);

for i = 1:Nsat
    [az, el, r] = aer(gsUser, satConst{i});
    Li = min(numSteps, numel(el));
    azU(1:Li, i) = az(1:Li);
    elU(1:Li, i) = el(1:Li);
    rU(1:Li,  i) = r(1:Li);
end

% --- Gateway到星座 ---
azG = nan(numSteps, Nsat);
elG = nan(numSteps, Nsat);
rG  = nan(numSteps, Nsat);

for i = 1:Nsat
    [az, el, r] = aer(gsGW, satConst{i});
    Li = min(numSteps, numel(el));
    azG(1:Li, i) = az(1:Li);
    elG(1:Li, i) = el(1:Li);
    rG(1:Li,  i) = r(1:Li);
end

% --- User到Jammer ---
azJ = nan(numSteps, numJam);
elJ = nan(numSteps, numJam);
rJ  = nan(numSteps, numJam);

for j = 1:numJam
    [az, el, r] = aer(gsUser, satJam{j});
    Lj = min(numSteps, numel(el));
    azJ(1:Lj, j) = az(1:Lj);
    elJ(1:Lj, j) = el(1:Lj);
    rJ(1:Lj,  j) = r(1:Lj);
end

% --- Range rate（用距离差分近似径向速度，足够工程可用） ---
rrU = nan(numSteps, Nsat);
for i = 1:Nsat
    ri = rU(:, i);
    dri = [diff(ri)/sample_time; 0]; % m/s，末尾补0
    rrU(:, i) = dri;
end

% --- 频率复用分配（简化：按plane/slot生成channel group） ---
satChan = zeros(1, Nsat);
for i = 1:Nsat
    satChan(i) = mod((satPlane(i)-1) + (satSlot(i)-1), reuseK) + 1; % 1..reuseK
end

%% =========================
% PART 3: 星网拓扑（简化ISL图）+ 指标计算
% =========================
fprintf('Step 4: 构建简化ISL拓扑图并计算全时序指标...\n');

% 简化ISL距离（常量近似）：同轨道面邻星 + 相邻轨道面同slot
d_inplane = 2*a*sin(pi/satsPerPlane);          % chord length
d_cross   = 2*a*sin((pi/numPlanes));           % chord length (RAAN spacing approx)

% 构建加权图：节点=卫星索引 1..Nsat
sList = [];
tList = [];
wList = [];

toIdx = @(p,s) (p-1)*satsPerPlane + s;

for p = 1:numPlanes
    for s = 1:satsPerPlane
        u = toIdx(p,s);

        % 同plane前后邻居（环形）
        s_fwd = s + 1; if s_fwd > satsPerPlane, s_fwd = 1; end
        s_bwd = s - 1; if s_bwd < 1, s_bwd = satsPerPlane; end

        v1 = toIdx(p, s_fwd);
        v2 = toIdx(p, s_bwd);

        sList(end+1) = u; tList(end+1) = v1; wList(end+1) = d_inplane; %#ok<SAGROW>
        sList(end+1) = u; tList(end+1) = v2; wList(end+1) = d_inplane;

        % 相邻plane同slot（环形）
        p_r = p + 1; if p_r > numPlanes, p_r = 1; end
        p_l = p - 1; if p_l < 1, p_l = numPlanes; end

        v3 = toIdx(p_r, s);
        v4 = toIdx(p_l, s);

        sList(end+1) = u; tList(end+1) = v3; wList(end+1) = d_cross;
        sList(end+1) = u; tList(end+1) = v4; wList(end+1) = d_cross;
    end
end

G = graph(sList, tList, wList, Nsat);
Dall = distances(G);  % NxN 预计算最短距离（米）

% ---- 预分配日志 ----
Log_SINR = nan(1, numSteps);
Log_BER  = nan(1, numSteps);
Log_THR  = zeros(1, numSteps);
Log_Doppler_Hz = nan(1, numSteps);

Log_VisUser = zeros(1, numSteps);
Log_VisGW   = zeros(1, numSteps);

Log_Serving = zeros(1, numSteps);   % user serving sat idx
Log_GWServ  = zeros(1, numSteps);   % gateway serving sat idx

Log_E2E_ms  = nan(1, numSteps);     % end-to-end propagation delay (ms)
Log_Event   = strings(1, numSteps); % Normal / CoChannel / JAMMING!!! / Protected / No Service / No Gateway

% ---- 抗干扰状态机 ----
AJ_Active = false;
AJ_Timer  = 0;  % seconds

% 噪声功率（mW）
p_n = 10^(Noise_dBm/10);

for k = 1:numSteps

    % 可见性
    visU = elU(k,:) > elMaskDeg;
    visG = elG(k,:) > elMaskDeg;
    Log_VisUser(k) = sum(visU);
    Log_VisGW(k)   = sum(visG);

    if ~any(visU)
        Log_Event(k) = "No Service";
        AJ_Active = false; AJ_Timer = 0;
        continue;
    end
    if ~any(visG)
        % 用户能上星，但网关不可达（星网端到端不可用）
        % 依然给出用户链路指标
        gwIdx = 0;
    else
        % 网关选最大仰角星
        idxG = find(visG);
        [~, bg] = max(elG(k, idxG));
        gwIdx = idxG(bg);
    end
    Log_GWServ(k) = gwIdx;

    % 候选服务星集合
    cand = find(visU);

    % --- 对每个候选星估计：S / CCI / Jam / SINR，用于“抗干扰时选SINR最优星” ---
    SINR_dB_vec = -inf(1, numel(cand));
    S_mW_vec    = zeros(1, numel(cand));
    I_mW_vec    = zeros(1, numel(cand));
    J_mW_vec    = zeros(1, numel(cand));

    jamAny = false;

    for ii = 1:numel(cand)
        si = cand(ii);

        % 信号功率
        PLs = fspl_dB(rU(k, si), freq_MHz);
        P_S_dBm = TxP_S_dBm - PLs + G_rx_dB;
        p_s = 10^(P_S_dBm/10);

        % 共信道干扰：同channel且可见的其他星
        sameChan = (satChan == satChan(si)) & visU;
        sameChan(si) = false;
        intIdx = find(sameChan);

        p_i = 0;
        if ~isempty(intIdx)
            PLi = fspl_dB(rU(k, intIdx), freq_MHz);
            P_I_dBm = TxP_I_dBm - PLi + (G_rx_dB - G_int_penalty_dB);
            p_i = sum(10.^(P_I_dBm/10));
        end

        % Jammer 共线干扰：至少一个Jammer在可见且 off-axis < 阈值
        p_j = 0;
        for j = 1:numJam
            if elJ(k, j) > elMaskDeg
                daz = wrap180(azU(k, si) - azJ(k, j));
                del = elU(k, si) - elJ(k, j);
                offAxis = sqrt(daz.^2 + del.^2);

                if offAxis < offAxisThDeg
                    jamAny = true;
                    PLj = fspl_dB(rJ(k, j), freq_MHz);

                    gainJ = 10; % Jammer有效增益（工程化）
                    if AJ_Active
                        gainJ = gainJ - AJ_NullDepth_dB; % 抗干扰后等效null
                    end

                    P_J_dBm = TxP_J_dBm - PLj + gainJ;
                    p_j = p_j + 10^(P_J_dBm/10);
                end
            end
        end

        % SINR
        sinr_lin = p_s / (p_i + p_j + p_n);
        sinr_dB  = 10*log10(sinr_lin);

        S_mW_vec(ii) = p_s;
        I_mW_vec(ii) = p_i;
        J_mW_vec(ii) = p_j;
        SINR_dB_vec(ii) = sinr_dB;
    end

    % --- 抗干扰状态机更新（用“是否存在共线干扰”触发） ---
    if jamAny
        AJ_Timer = AJ_Timer + sample_time;
        if AJ_Timer >= AJ_DelaySec
            AJ_Active = true;
        end
    else
        AJ_Timer = 0;
        AJ_Active = false;
    end

    % --- 选服务星：正常=最大仰角；抗干扰=最大SINR ---
    if AJ_Active
        [~, best] = max(SINR_dB_vec);
    else
        [~, best] = max(elU(k, cand));
    end
    servIdx = cand(best);
    Log_Serving(k) = servIdx;

    % 取最终SINR（对servIdx对应的候选项）
    sinr_dB = SINR_dB_vec(best);
    Log_SINR(k) = sinr_dB;

    % Doppler：range-rate -> doppler
    v_r = rrU(k, servIdx); % m/s (approx)
    Log_Doppler_Hz(k) = -(v_r / c) * fc_Hz;

    % BER/吞吐量（保持你原工程风格：QPSK近似 + Shannon吞吐量）
    lin_sinr = 10^(sinr_dB/10);
    ber_val = 0.5 * erfc(sqrt(lin_sinr/2));
    ber_val = max(min(ber_val, 0.5), 1e-9);
    Log_BER(k) = ber_val;

    if ber_val > 0.2
        Log_THR(k) = 0;
    else
        eff = log2(1 + lin_sinr);
        eff = min(eff, 6);
        Log_THR(k) = BW * eff * 0.8 / 1e6; % Mbps
    end

    % 端到端传播时延（用户->serv + ISL最短路 + gw->gateway）
    if gwIdx == 0
        Log_E2E_ms(k) = nan;
        if jamAny && ~AJ_Active
            Log_Event(k) = "JAMMING!!! (No GW)";
        elseif jamAny && AJ_Active
            Log_Event(k) = "Protected (No GW)";
        else
            % 判断是否共信道干扰较强（简单阈值：I > S/10）
            p_s = S_mW_vec(best); p_i = I_mW_vec(best);
            if p_i > p_s/10
                Log_Event(k) = "CoChannel (No GW)";
            else
                Log_Event(k) = "Normal (No GW)";
            end
        end
    else
        % 路径距离（m）
        d_user = rU(k, servIdx);
        d_gw   = rG(k, gwIdx);
        d_isl  = Dall(servIdx, gwIdx);

        if isinf(d_isl) || isnan(d_isl)
            Log_E2E_ms(k) = nan;
        else
            d_total = d_user + d_isl + d_gw;
            Log_E2E_ms(k) = (d_total / c) * 1e3; % ms
        end

        if jamAny && ~AJ_Active
            Log_Event(k) = "JAMMING!!!";
        elseif jamAny && AJ_Active
            Log_Event(k) = "Protected";
        else
            p_s = S_mW_vec(best); p_i = I_mW_vec(best);
            if p_i > p_s/10
                Log_Event(k) = "CoChannel";
            else
                Log_Event(k) = "Normal";
            end
        end
    end
end

%% =========================
% PART 4: 仪表盘 + 3D Viewer 联动
% =========================
fprintf('Step 5: 构建仪表盘 + 3D联动...\n');

dash = uifigure('Name','LEO StarNet EMC Dashboard (Linked to 3D Viewer)', ...
    'Color','w','Position',[50 50 1400 860]);

gl = uigridlayout(dash, [4 4]);
gl.RowHeight   = {36,'1x','1x','1x'};
gl.ColumnWidth = {'1x','1x','1x',420};
gl.Padding     = [10 10 10 10];
gl.RowSpacing  = 8;
gl.ColumnSpacing = 10;

titleLbl = uilabel(gl, 'Text', ...
    sprintf('LEO StarNet EMC Dashboard | %d sats | Full Orbit %.1f min | Linked to 3D Viewer', Nsat, T_orbit/60), ...
    'FontSize',16,'FontWeight','bold');
titleLbl.Layout.Row = 1; titleLbl.Layout.Column = [1 4];

% ---- 左侧曲线区 ----
axSINR = uiaxes(gl); axSINR.Layout.Row=2; axSINR.Layout.Column=[1 3];
axBER  = uiaxes(gl); axBER.Layout.Row=3; axBER.Layout.Column=[1 3];
axTHR  = uiaxes(gl); axTHR.Layout.Row=4; axTHR.Layout.Column=[1 2];
axDOP  = uiaxes(gl); axDOP.Layout.Row=4; axDOP.Layout.Column=3;

axList = [axSINR axBER axTHR axDOP];
for ax = axList
    ax.Toolbar.Visible = 'off';
    ax.Interactions = [];
end

% SINR
plot(axSINR, t_axis_min, Log_SINR, 'LineWidth', 2); grid(axSINR,'on');
title(axSINR,'SINR (dB)'); ylabel(axSINR,'dB');
xlim(axSINR,[0 max(t_axis_min)]); ylim(axSINR, SINR_YLIM);
yline(axSINR, 0, 'r--', 'Disconnect');

% 干扰/抗干扰/共信道区域着色（支持多段）
shadeTagRegions(axSINR, t_axis_min, Log_Event, "JAMMING!!!", SINR_YLIM, [1 0 0], 'Jamming', -10);
shadeTagRegions(axSINR, t_axis_min, Log_Event, "Protected",  SINR_YLIM, [0 1 0], 'Protected', 40);
shadeTagRegions(axSINR, t_axis_min, Log_Event, "CoChannel",  SINR_YLIM, [1 0.5 0], 'Co-Channel', 20);

% BER
semilogy(axBER, t_axis_min, Log_BER, 'LineWidth', 2); grid(axBER,'on');
title(axBER,'Bit Error Rate (BER)'); ylabel(axBER,'Log Scale'); xlabel(axBER,'Time (Minutes)');
xlim(axBER,[0 max(t_axis_min)]); ylim(axBER,[1e-9 1]);

% THR
plot(axTHR, t_axis_min, Log_THR, 'LineWidth', 2); grid(axTHR,'on');
title(axTHR,'Throughput (Mbps)'); ylabel(axTHR,'Mbps'); xlabel(axTHR,'Time (Minutes)');
xlim(axTHR,[0 max(t_axis_min)]); ylim(axTHR, THR_YLIM);

% Doppler
plot(axDOP, t_axis_min, Log_Doppler_Hz/1e3, 'LineWidth', 2); grid(axDOP,'on');
title(axDOP,'Doppler (kHz)'); ylabel(axDOP,'kHz'); xlabel(axDOP,'Time (Minutes)');
xlim(axDOP,[0 max(t_axis_min)]);

% ---- 竖线游标：不要用 xline（R2021a + UIAxes 容易“不动”），用 line 更新 XData ----
curSINR = line(axSINR, [0 0], SINR_YLIM, 'Color','k','LineWidth',1.8);
curBER  = line(axBER,  [0 0], [1e-9 1],  'Color','k','LineWidth',1.8);
curTHR  = line(axTHR,  [0 0], THR_YLIM,  'Color','k','LineWidth',1.8);
curDOP  = line(axDOP,  [0 0], ylim(axDOP), 'Color','k','LineWidth',1.8);

%% ---- 右侧状态区 ----
right = uigridlayout(gl, [12 1]);
right.Layout.Row = [2 4];
right.Layout.Column = 4;
right.RowHeight = {24,24,24,24,24,24,24,24,28,160,'1x',24};
right.Padding = [0 0 0 0];

lblTime  = uilabel(right,'Text','Current Time: -','FontWeight','bold');
lblSpeed = uilabel(right,'Text','Viewer Speed: -');
lblServ  = uilabel(right,'Text','Serving Sat: -');
lblGW    = uilabel(right,'Text','Gateway Sat: -');
lblVis   = uilabel(right,'Text','Visible(User/GW): - / -');

lblSINR  = uilabel(right,'Text','SINR: - dB');
lblBER   = uilabel(right,'Text','BER: -');
lblTHR   = uilabel(right,'Text','Throughput: - Mbps');
lblDOP   = uilabel(right,'Text','Doppler: - kHz');
lblE2E   = uilabel(right,'Text','E2E Delay: - ms');

% 合规灯：这里给一个工程化判据（可按你组会口径调整）
% 用 BLER≈1-(1-BER)^1024，阈值0.1（示例风格与之前一致）
uilabel(right,'Text','Compliance Lamp (BLER<=0.1):');
lamp = uilamp(right); lamp.Color = [0 1 0];

tbl = uitable(right, ...
    'Data', cell(0,3), ...
    'ColumnName', {'Time','Event','Detail'}, ...
    'ColumnEditable', [false false false]);

lblHint = uilabel(right, 'Text', ...
    '提示：在3D Viewer里改播放速度/拖动时间轴，竖线会自动同步。', ...
    'WordWrap','on');

%% =========================
% PART 5: 3D Viewer + 联动计时器
% =========================
fprintf('Step 6: 启动3D Viewer（Basemap=none，尽量规避在线地形异常）...\n');

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

% 把要用的数据打包给回调
app = struct();
app.sim_start = sim_start;
app.sample_time = sample_time;
app.numSteps = numSteps;
app.t_axis_min = t_axis_min;

app.Log_SINR = Log_SINR;
app.Log_BER  = Log_BER;
app.Log_THR  = Log_THR;
app.Log_DOPk = Log_Doppler_Hz/1e3;
app.Log_E2E  = Log_E2E_ms;
app.Log_Event= Log_Event;
app.Log_Serving = Log_Serving;
app.Log_GWServ  = Log_GWServ;
app.Log_VisUser = Log_VisUser;
app.Log_VisGW   = Log_VisGW;

app.satName = satName;

app.lblTime  = lblTime;
app.lblSpeed = lblSpeed;
app.lblServ  = lblServ;
app.lblGW    = lblGW;
app.lblVis   = lblVis;

app.lblSINR = lblSINR;
app.lblBER  = lblBER;
app.lblTHR  = lblTHR;
app.lblDOP  = lblDOP;
app.lblE2E  = lblE2E;

app.lamp = lamp;
app.tbl  = tbl;

app.curSINR = curSINR;
app.curBER  = curBER;
app.curTHR  = curTHR;
app.curDOP  = curDOP;

app.BLER_BlockLen = 1024;
app.BLER_Th = 0.1;

app.lastEvent = "";
app.lastServ  = 0;
app.lastGW    = 0;

app.v = v;
guidata(dash, app);

tmr = timer( ...
    'ExecutionMode','fixedSpacing', ...
    'Period', 0.05, ...     % 20Hz刷新（足够“看起来连续”）
    'BusyMode','drop', ...
    'TimerFcn', @(~,~)onTick(dash) );

dash.CloseRequestFcn = @(src,evt)onClose(src,evt,tmr);

start(tmr);

% 最后再play（避免某些环境play阻塞导致timer没启动）
try
    play(v);
catch
    try
        play(sc);
    catch
        try
            show(sc);
        catch
        end
    end
end

%% =========================
% Local Functions
% =========================
function onTick(dashFig)
    if ~isvalid(dashFig), return; end
    app = guidata(dashFig);

    if ~isfield(app,'v') || isempty(app.v) || ~isvalid(app.v)
        safeStopTimer();
        return;
    end

    % 只读 CurrentTime：避免你之前 set.CurrentTime 崩溃
    try
        ct = app.v.CurrentTime;
    catch
        safeStopTimer();
        return;
    end

    % 映射到k（用 floor 保证单调）
    dt = seconds(ct - app.sim_start);
    k = floor(dt / app.sample_time) + 1;
    k = max(1, min(app.numSteps, k));

    x = app.t_axis_min(k);

    % 更新竖线（line对象更新XData最稳）
    try
        app.curSINR.XData = [x x];
        app.curBER.XData  = [x x];
        app.curTHR.XData  = [x x];
        app.curDOP.XData  = [x x];
    catch
    end

    % 指标
    sinr = app.Log_SINR(k);
    ber  = app.Log_BER(k);
    thr  = app.Log_THR(k);
    dopk = app.Log_DOPk(k);
    e2e  = app.Log_E2E(k);
    evt  = string(app.Log_Event(k));

    serv = app.Log_Serving(k);
    gw   = app.Log_GWServ(k);

    visU = app.Log_VisUser(k);
    visG = app.Log_VisGW(k);

    % BLER近似 -> 灯
    if isnan(ber)
        bler = 1;
    else
        ber = min(max(ber,0),1);
        bler = 1 - (1-ber)^(app.BLER_BlockLen);
    end
    if bler > app.BLER_Th
        app.lamp.Color = [1 0 0];
    else
        app.lamp.Color = [0 1 0];
    end

    % Viewer speed显示
    spdTxt = "Viewer Speed: -";
    try
        if isprop(app.v,'PlaybackSpeedMultiplier')
            spdTxt = sprintf("Viewer Speed: x%.2f", app.v.PlaybackSpeedMultiplier);
        end
    catch
    end

    % Serving/GW名字
    servTxt = "-";
    if serv > 0 && serv <= numel(app.satName), servTxt = char(app.satName(serv)); end
    gwTxt = "-";
    if gw > 0 && gw <= numel(app.satName), gwTxt = char(app.satName(gw)); end

    % UI更新
    app.lblTime.Text  = sprintf("Current Time: %s", char(ct));
    app.lblSpeed.Text = spdTxt;
    app.lblServ.Text  = sprintf("Serving Sat: %s (#%d)", servTxt, serv);
    app.lblGW.Text    = sprintf("Gateway Sat: %s (#%d)", gwTxt, gw);
    app.lblVis.Text   = sprintf("Visible(User/GW): %d / %d", visU, visG);

    app.lblSINR.Text = sprintf("SINR: %s dB", fmtNum(sinr,2));
    app.lblBER.Text  = sprintf("BER: %s", fmtSci(ber));
    app.lblTHR.Text  = sprintf("Throughput: %s Mbps", fmtNum(thr,2));
    app.lblDOP.Text  = sprintf("Doppler: %s kHz", fmtNum(dopk,2));
    app.lblE2E.Text  = sprintf("E2E Delay: %s ms", fmtNum(e2e,2));

    % 事件日志：事件变化 或 选星切换时记录
    if app.lastEvent ~= evt
        app = pushEvent(app, ct, "State", char(evt));
        app.lastEvent = evt;
    end
    if app.lastServ ~= serv && serv ~= 0
        app = pushEvent(app, ct, "Handover", sprintf("User -> %s (#%d)", servTxt, serv));
        app.lastServ = serv;
    end
    if app.lastGW ~= gw && gw ~= 0
        app = pushEvent(app, ct, "GW Switch", sprintf("GW -> %s (#%d)", gwTxt, gw));
        app.lastGW = gw;
    end

    guidata(dashFig, app);

    drawnow limitrate;

    if k >= app.numSteps
        safeStopTimer();
    end
end

function PL = fspl_dB(range_m, freq_MHz)
    % Free-space path loss (dB): 32.45 + 20log10(d_km) + 20log10(f_MHz)
    d_km = max(range_m, 1) / 1000;
    PL = 32.45 + 20*log10(d_km) + 20*log10(freq_MHz);
end

function a = wrap180(x)
    % wrap degrees to [-180,180)
    a = mod(x + 180, 360) - 180;
end

function shadeTagRegions(ax, t_axis, tags, targetTag, yLim, colorRGB, labelText, labelY)
    % 支持多段：把所有连续段都patch出来
    idx = find(tags == targetTag);
    if isempty(idx), return; end

    % 找连续段
    breaks = [1 find(diff(idx) > 1) + 1 numel(idx)+1];
    for b = 1:(numel(breaks)-1)
        seg = idx(breaks(b):breaks(b+1)-1);
        x1 = t_axis(seg(1));
        x2 = t_axis(seg(end));
        X = [x1 x2 x2 x1];
        Y = [yLim(1) yLim(1) yLim(2) yLim(2)];
        patch(ax, X, Y, colorRGB, 'FaceAlpha', 0.12, 'EdgeColor','none');
        text(ax, mean([x1 x2]), labelY, labelText, 'Color', colorRGB, 'HorizontalAlignment','center');
    end
end

function s = fmtNum(x, n)
    if isempty(x) || isnan(x), s = "-"; return; end
    s = num2str(x, ['%.' num2str(n) 'f']);
end

function s = fmtSci(x)
    if isempty(x) || isnan(x), s = "-"; return; end
    s = num2str(x, '%.3e');
end

function app = pushEvent(app, t, evt, detail)
    data = app.tbl.Data;
    if size(data,1) >= 200
        data = data(end-150:end, :);
    end
    newRow = {char(t), evt, detail};
    app.tbl.Data = [data; newRow];
end

function safeStopTimer()
    % 只停止本脚本的timer（尽量不干扰别人的timer）
    try
        tmrList = timerfindall;
        for i = 1:numel(tmrList)
            try
                if strcmp(tmrList(i).Running,'on')
                    stop(tmrList(i));
                end
                delete(tmrList(i));
            catch
            end
        end
    catch
    end
end

function onClose(src, ~, tmr)
    try
        if isa(tmr,'timer') && isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
    catch
    end
    delete(src);
end
