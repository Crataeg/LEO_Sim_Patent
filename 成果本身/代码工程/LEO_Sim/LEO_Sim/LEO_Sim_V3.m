%% Project: LEO EMC Final + 3D Viewer Linked Dashboard + Doppler + GA + GAN (R2021a)
%  Target:
%    1) 指标曲线与 3D satelliteScenarioViewer 播放“真实联动”（竖线跟随 Viewer.CurrentTime）
%    2) Viewer 播放速度变化时（PlaybackSpeedMultiplier/速度滑条），竖线推进速度自动同步
%    3) 在此工程骨架上补齐：多普勒频移 + GA 优化抗干扰参数 + GAN 生成智能干扰轮廓
%
%  Version: R2021a (你当前环境)
%  Notes:
%    - 不写 v.CurrentTime（你之前报错点）；仅“读” v.CurrentTime 来驱动竖线
%    - 仍保留你原 Force_Jamming 中段强制干扰逻辑（不改变剧本框架）
%    - 加入 Doppler 与 DopplerRate：既显示也参与性能退化（可开关）
%    - GA 使用 Global Optimization Toolbox: ga()
%    - GAN 使用 Deep Learning Toolbox: 轻量 GAN（失败自动降级为伪GAN）

clear; clc; close all;
rng(7);

%% ========================================================================
%  PART 1: 绝对可靠的过顶时间锁定 (回退到 V2 逻辑)
%  ========================================================================
fprintf('Step 1: 计算青岛站过顶时刻...\n');

t_search_start = datetime('now');
t_search_stop  = t_search_start + hours(12);
sc_search = satelliteScenario(t_search_start, t_search_stop, 60);

gs_temp  = groundStation(sc_search, 36.06, 120.38, 'Name', 'Qingdao_Search');
sat_temp = satellite(sc_search, 6371000+1200000, 0, 88, 0, 0, 0);

ac_temp   = access(sat_temp, gs_temp);
intervals = accessIntervals(ac_temp);

if isempty(intervals)
    center_time = datetime('now') + minutes(10);
    fprintf('        [警告] 自动搜索未找到，使用默认时间。\n');
else
    [~, best_idx] = max(intervals.EndTime - intervals.StartTime);
    center_time = intervals.StartTime(best_idx) + (intervals.EndTime(best_idx) - intervals.StartTime(best_idx))/2;
    fprintf('        [锁定] 最佳过顶时刻: %s\n', char(center_time));
end

sim_start   = center_time - minutes(8);
sim_stop    = center_time + minutes(8);
sample_time = 1; % seconds

%% ========================================================================
%  PART 2: 重建场景 (Scene Setup)
%  ========================================================================
fprintf('Step 2: 重建高精度场景...\n');

sc = satelliteScenario(sim_start, sim_stop, sample_time);

gs = groundStation(sc, 36.06, 120.38, 'Name', 'Qingdao_GS');

satSvc = satellite(sc, 6371000+1200000, 0, 88, 0, 0, 0,    'Name', 'Service_Sat');
satJam = satellite(sc, 6371000+1200000, 0, 88, 0, 0, -0.5, 'Name', 'Jammer_Sat');

sensSvc = conicalSensor(satSvc, 'MaxViewAngle', 45);
fovSvc  = fieldOfView(sensSvc); fovSvc.LineColor = [0 1 0];

sensJam = conicalSensor(satJam, 'MaxViewAngle', 45);
fovJam  = fieldOfView(sensJam); fovJam.LineColor = [1 0 0];

ac1 = access(satSvc, gs);  ac1.LineColor = [0 1 0];
ac2 = access(satJam, gs);  ac2.LineColor = [1 0 0];

%% ========================================================================
%  PART 3: 预计算几何 + 多普勒 (离线算好全量序列)
%  ========================================================================
fprintf('Step 3: 预计算几何与多普勒...\n');

[azS, elS, rS] = aer(gs, satSvc);
[azJ, elJ, rJ] = aer(gs, satJam);

timeVec = 0:sample_time:seconds(sim_stop - sim_start);
numSteps = min([length(elS), length(timeVec), length(elJ)]);
timeVec  = timeVec(1:numSteps);

azS = azS(1:numSteps); elS = elS(1:numSteps); rS = rS(1:numSteps);
azJ = azJ(1:numSteps); elJ = elJ(1:numSteps); rJ = rJ(1:numSteps);

% “共线”度量：方位/俯仰角差（仅用于展示/辅助对抗建模，不改变你的强制干扰窗口）
Log_OffAxisDeg = sqrt((azS-azJ).^2 + (elS-elJ).^2);

% ---- Doppler 计算（体现：fd 与 fdRate）----
% 用距离差分估计径向速度，再映射到多普勒：fd = -(v_rad/c)*fc
c  = physconst('LightSpeed');
fc = 28e9; % 与你原链路预算一致（28GHz）
rr = zeros(1,numSteps);
rr(2:end) = diff(rS) / sample_time;   % m/s
rr(1) = rr(2);
rr = movmean(rr, 7);                  % 平滑抑制 diff 抖动

Log_DopplerHz = -(rr./c) * fc;        % Hz
Log_DopplerRateHz = zeros(1,numSteps);
Log_DopplerRateHz(2:end) = diff(Log_DopplerHz)/sample_time; % Hz/s
Log_DopplerRateHz(1) = Log_DopplerRateHz(2);
Log_DopplerRateHz = movmean(Log_DopplerRateHz, 7);

%% ========================================================================
%  PART 4: GAN 生成“智能干扰轮廓” (jamAgg ∈ [0,1])
%  ========================================================================
fprintf('Step 4: 生成 GAN 智能干扰轮廓...\n');
jamAgg = makeJammerProfile_GAN_orFallback(numSteps, Log_OffAxisDeg, Log_DopplerRateHz);

%% ========================================================================
%  PART 5: GA 优化抗干扰参数（在你原状态机框架上优化“联动参数”）
%  ========================================================================
fprintf('Step 5: GA 优化抗干扰参数...\n');

% 你原功率/噪声/带宽（保持）
TxP_S = 55;  % dBm
TxP_J0 = 75; % dBm (基础值，GAN会调制)
Noise = -110;
Bandwidth = 100e6; % 100MHz

% GA 优化参数向量 x：
%   x(1) = AJ_DelaySec       抗干扰触发延迟（秒）          [0, 60]
%   x(2) = AJ_NullDepth_dB   抗干扰抑制深度（负值）        [-60, -20]
%   x(3) = DopplerGuardHzps  多普勒变化率门限（Hz/s）      [5e3, 2e5]
%   x(4) = JamBoostMax_dB    GAN干扰增强上限（dB）         [0, 15]
lb = [0,   -60, 5e3,  0];
ub = [60,  -20, 2e5, 15];

obj = @(x) gaObjective_KeepScript( ...
    x, numSteps, sample_time, elS, rS, ...
    Log_DopplerHz, Log_DopplerRateHz, jamAgg, ...
    TxP_S, TxP_J0, Noise, Bandwidth);

gaOpts = optimoptions('ga', ...
    'PopulationSize', 18, ...
    'MaxGenerations', 12, ...
    'Display', 'iter', ...
    'UseParallel', false);

try
    [xBest, ~] = ga(obj, 4, [], [], [], [], lb, ub, [], gaOpts);
catch ME
    fprintf('[警告] ga() 运行失败：%s\n', ME.message);
    fprintf('       回退到默认参数（不影响联动演示）\n');
    xBest = [30, -40, 5e4, 10];
end

AJ_DelaySec      = xBest(1);
AJ_NullDepth_dB  = xBest(2);
DopplerGuardHzps = xBest(3);
JamBoostMax_dB   = xBest(4);

fprintf('        [GA最优] AJ_Delay=%.1fs, NullDepth=%.1fdB, DopplerGuard=%.0fHz/s, JamBoostMax=%.1fdB\n', ...
    AJ_DelaySec, AJ_NullDepth_dB, DopplerGuardHzps, JamBoostMax_dB);

%% ========================================================================
%  PART 6: 执行物理层计算（最终：用 GA+GAN+Doppler 参与的版本）
%  ========================================================================
fprintf('Step 6: 执行物理层计算（GA+GAN+Doppler 参与）...\n');

[Log_SINR, Log_BER, Log_Thr, Log_Event, Log_BLER] = simulateOnce_Final( ...
    numSteps, sample_time, elS, rS, ...
    Log_DopplerHz, Log_DopplerRateHz, jamAgg, ...
    TxP_S, TxP_J0, Noise, Bandwidth, ...
    AJ_DelaySec, AJ_NullDepth_dB, DopplerGuardHzps, JamBoostMax_dB);

%% ========================================================================
%  PART 7: 生成仪表盘 + 3D 联动（保留你原结构，并增加 Doppler 显示）
%  ========================================================================
fprintf('Step 7: 生成图表 + 3D联动...\n');

t_axis = (timeVec - timeVec(1))/60;

% ---------- Dashboard (uifigure) ----------
dash = uifigure('Name','LEO EMC Dashboard (Linked to 3D Viewer) + Doppler/GA/GAN', ...
    'Color','w','Position',[50 50 1280 860]);

gl = uigridlayout(dash, [3 4]);
gl.RowHeight    = {34,'1x','1x'};
gl.ColumnWidth  = {'1x','1x','1x',420};
gl.Padding      = [10 10 10 10];
gl.RowSpacing   = 8;
gl.ColumnSpacing= 10;

titleLbl = uilabel(gl, 'Text','LEO EMC Dashboard (竖线随3D Viewer时间推进)  + Doppler / GA / GAN', ...
    'FontSize',16,'FontWeight','bold');
titleLbl.Layout.Row = 1; titleLbl.Layout.Column = [1 4];

% 三个 uiaxes（沿用你工程风格：上SINR、下BER，右边吞吐量+多普勒小窗）
axSINR = uiaxes(gl); axSINR.Layout.Row=2; axSINR.Layout.Column=[1 3];
axBER  = uiaxes(gl); axBER.Layout.Row=3; axBER.Layout.Column=[1 3];

axSINR.Toolbar.Visible = 'off'; axBER.Toolbar.Visible = 'off';
axSINR.Interactions = []; axBER.Interactions = [];

% 右侧状态区
right = uigridlayout(gl,[12 1]);
right.Layout.Row = [2 3];
right.Layout.Column = 4;
right.RowHeight = {24,24,24,24,24,24,24,24,120,120,'1x',24};
right.Padding = [0 0 0 0];

lblTime = uilabel(right,'Text','Current Time: -','FontWeight','bold');
lblSINR = uilabel(right,'Text','SINR: - dB');
lblBER  = uilabel(right,'Text','BER: -');
lblThr  = uilabel(right,'Text','Throughput: - Mbps');

lblDop  = uilabel(right,'Text','Doppler: - kHz');
lblDopR = uilabel(right,'Text','DopplerRate: - kHz/s');
lblOff  = uilabel(right,'Text','Off-axis: - deg');

lblCrit = uilabel(right,'Text','K.157 Criterion: -','FontWeight','bold');

lampLbl = uilabel(right,'Text','Compliance Lamp (BLER<=0.1):');
lamp = uilamp(right);
lamp.Color = [0 1 0];

% 吞吐量小图
axTHR = uiaxes(right);
axTHR.Toolbar.Visible = 'off';
axTHR.Interactions = [];

% 多普勒小图
axDOP = uiaxes(right);
axDOP.Toolbar.Visible = 'off';
axDOP.Interactions = [];

tbl = uitable(right, ...
    'Data', cell(0,3), ...
    'ColumnName', {'Time','Event','Detail'}, ...
    'ColumnEditable', [false false false]);
tbl.Layout.Row = 11;

lblSpeed = uilabel(right,'Text','Viewer Speed: -');
lblHint  = uilabel(right,'Text','提示：调3D Viewer播放速度，竖线会同步。', 'WordWrap','on');

% ---------- 绘图（静态曲线 + 竖线游标） ----------
% SINR（工程风格）
plot(axSINR, t_axis, Log_SINR, 'LineWidth', 2); grid(axSINR,'on');
title(axSINR,'SINR (dB)'); ylabel(axSINR,'dB');
xlim(axSINR,[0 max(t_axis)]); ylim(axSINR,[-20 50]);
yline(axSINR,0,'r--','Disconnect');

% BER
semilogy(axBER, t_axis, Log_BER, 'LineWidth', 2); grid(axBER,'on');
title(axBER,'Bit Error Rate (BER)'); ylabel(axBER,'Log Scale'); xlabel(axBER,'Time (Minutes)');
xlim(axBER,[0 max(t_axis)]); ylim(axBER,[1e-9 1]);

% THR（保持“凹陷更明显”的工程风格）
plot(axTHR, t_axis, Log_Thr, 'LineWidth', 2); grid(axTHR,'on');
title(axTHR,'Throughput (Mbps)'); ylabel(axTHR,'Mbps'); xlabel(axTHR,'Time (Minutes)');
xlim(axTHR,[0 max(t_axis)]); ylim(axTHR,[0 600]);

% Doppler 小图（体现新增）
plot(axDOP, t_axis, Log_DopplerHz/1e3, 'LineWidth', 1.8); grid(axDOP,'on');
title(axDOP,'Doppler (kHz)'); ylabel(axDOP,'kHz'); xlabel(axDOP,'Time (Minutes)');
xlim(axDOP,[0 max(t_axis)]);

% 干扰/抗干扰区块（静态标注；用 contains 防止我们后面追加后缀）
markRegionContains(axSINR, t_axis, Log_Event, "JAMMING!!!",  [-20 50], [1 0 0], 'Jamming', -10);
markRegionContains(axSINR, t_axis, Log_Event, "Protected",   [-20 50], [0 1 0], 'AJ Protected', 40);

% 三条竖线游标（联动关键）
cur1 = xline(axSINR, 0, 'k-', 'LineWidth', 1.8);
cur2 = xline(axBER,  0, 'k-', 'LineWidth', 1.8);
cur3 = xline(axTHR,  0, 'k-', 'LineWidth', 1.8);

%% ========================================================================
%  PART 8: 3D Viewer + timer 联动（完全沿用你方案：只读 v.CurrentTime）
%  ========================================================================
fprintf('启动3D视图...\n');

try
    v = satelliteScenarioViewer(sc, 'Basemap', 'none', 'PlaybackSpeedMultiplier', 20, 'Dimension', '3D');
catch
    try
        v = satelliteScenarioViewer(sc, 'PlaybackSpeedMultiplier', 20, 'Dimension', '3D');
    catch
        v = satelliteScenarioViewer(sc);
        try, v.PlaybackSpeedMultiplier = 20; catch, end
    end
end

% 播放（由 Viewer 自己推进时间）
try
    play(v);
catch
    try
        play(sc);
    catch
        try, show(sc); catch, end
    end
end

% ---------- 联动计时器：读 v.CurrentTime → 映射到 k → 更新竖线+仪表盘 ----------
app = struct();
app.sim_start = sim_start;
app.sample_time = sample_time;
app.numSteps = numSteps;
app.t_axis = t_axis;

app.Log_SINR = Log_SINR;
app.Log_BER  = Log_BER;
app.Log_Thr  = Log_Thr;
app.Log_Event= Log_Event;
app.Log_OffAxisDeg = Log_OffAxisDeg;
app.Log_DopplerHz = Log_DopplerHz;
app.Log_DopplerRateHz = Log_DopplerRateHz;
app.Log_BLER = Log_BLER;

app.dash = dash;
app.v = v;

app.lblTime = lblTime;
app.lblSINR = lblSINR;
app.lblBER  = lblBER;
app.lblThr  = lblThr;
app.lblDop  = lblDop;
app.lblDopR = lblDopR;
app.lblOff  = lblOff;
app.lblCrit = lblCrit;
app.lblSpeed= lblSpeed;

app.lamp = lamp;
app.tbl  = tbl;

app.cur1 = cur1;
app.cur2 = cur2;
app.cur3 = cur3;

% K.157 简化映射参数（保留你原仪表盘思路）
app.BlockLenBits = 1024;
app.BLER_Th = 0.1;
app.Trecover = 30;

app.inOutage = false;
app.outageStartTime = NaT;

app.lastEventTag = "";

guidata(dash, app);

tmr = timer( ...
    'ExecutionMode','fixedSpacing', ...
    'Period', 0.05, ...
    'BusyMode','drop', ...
    'TimerFcn', @(~,~)onTick(dash));

% 把 timer 存起来，避免你原 safeStopTimer(timerfindall) 误删其他 timer
app = guidata(dash);
app.tmr = tmr;
guidata(dash, app);

dash.CloseRequestFcn = @(src,evt)onClose(src,evt);
start(tmr);

%% ============================ Local Functions ============================

function onTick(dashFig)
    if ~isvalid(dashFig), return; end
    app = guidata(dashFig);

    if ~isfield(app,'v') || isempty(app.v) || ~isvalid(app.v)
        safeStopOwnTimer(dashFig);
        return;
    end

    % 只读 CurrentTime
    try
        ct = app.v.CurrentTime;
    catch
        safeStopOwnTimer(dashFig);
        return;
    end

    dt = seconds(ct - app.sim_start);
    k = round(dt / app.sample_time) + 1;
    k = max(1, min(app.numSteps, k));

    xMin = app.t_axis(k);

    % 更新竖线
    if isvalid(app.cur1), app.cur1.Value = xMin; end
    if isvalid(app.cur2), app.cur2.Value = xMin; end
    if isvalid(app.cur3), app.cur3.Value = xMin; end

    sinr = app.Log_SINR(k);
    ber  = app.Log_BER(k);
    thr  = app.Log_Thr(k);
    offd = app.Log_OffAxisDeg(k);
    dopk = app.Log_DopplerHz(k)/1e3;         % kHz
    doprk= app.Log_DopplerRateHz(k)/1e3;     % kHz/s
    tag  = app.Log_Event(k);

    % BLER（已离线算好，避免重复算）
    bler = app.Log_BLER(k);

    % K.157 简化判据
    isBad = (bler > app.BLER_Th) || contains(tag,"No Service") || (thr <= 0);

    nowTime = ct;
    if ~app.inOutage && isBad
        app.inOutage = true;
        app.outageStartTime = nowTime;
    elseif app.inOutage && ~isBad
        Tloss = seconds(nowTime - app.outageStartTime);
        if Tloss <= app.Trecover
            crit = "B (Transient Loss, Auto-Recovered)";
        else
            crit = "C (Late Recovery)";
        end
        app.inOutage = false;
        app.outageStartTime = NaT;
        app = pushEvent(app, nowTime, "Recovered", sprintf("Tloss=%.1fs, %s", Tloss, crit));
    end

    if app.inOutage
        Tloss = seconds(nowTime - app.outageStartTime);
        if Tloss <= app.Trecover
            crit = sprintf("B? (Recovering %.1fs/%.0fs)", Tloss, app.Trecover);
        else
            crit = sprintf("C (Exceeded Trecover %.1fs)", Tloss);
        end
    else
        if (bler <= app.BLER_Th) && (thr > 0)
            crit = "A (Continuous Operation)";
        else
            crit = "B? (Degraded)";
        end
    end

    % 合规灯
    if bler > app.BLER_Th
        app.lamp.Color = [1 0 0];
    else
        app.lamp.Color = [0 1 0];
    end

    % 速度
    spdTxt = "Viewer Speed: -";
    try
        if isprop(app.v,'PlaybackSpeedMultiplier')
            spdTxt = sprintf("Viewer Speed: x%.2f", app.v.PlaybackSpeedMultiplier);
        end
    catch
    end

    % UI 更新
    app.lblTime.Text = sprintf("Current Time: %s", char(ct));
    app.lblSINR.Text = sprintf("SINR: %s dB", fmtNum(sinr,2));
    app.lblBER.Text  = sprintf("BER: %s", fmtSci(ber));
    app.lblThr.Text  = sprintf("Throughput: %s Mbps", fmtNum(thr,2));
    app.lblDop.Text  = sprintf("Doppler: %s kHz", fmtNum(dopk,2));
    app.lblDopR.Text = sprintf("DopplerRate: %s kHz/s", fmtNum(doprk,2));
    app.lblOff.Text  = sprintf("Off-axis: %s deg", fmtNum(offd,2));
    app.lblCrit.Text = sprintf("K.157 Criterion: %s", crit);
    app.lblSpeed.Text= spdTxt;

    % 事件日志：变化才记
    if app.lastEventTag ~= string(tag)
        app = pushEvent(app, nowTime, "State", char(tag));
        app.lastEventTag = string(tag);
    end

    guidata(dashFig, app);

    if k >= app.numSteps
        safeStopOwnTimer(dashFig);
    end
end

function s = fmtNum(x, n)
    if isnan(x), s = "-"; return; end
    s = num2str(x, ['%.' num2str(n) 'f']);
end

function s = fmtSci(x)
    if isnan(x), s = "-"; return; end
    s = num2str(x, '%.3e');
end

function app = pushEvent(app, t, evt, detail)
    data = app.tbl.Data;
    if size(data,1) >= 200
        data = data(end-150:end,:);
    end
    app.tbl.Data = [data; {char(t), evt, detail}];
end

function safeStopOwnTimer(dashFig)
    if ~isvalid(dashFig), return; end
    app = guidata(dashFig);
    try
        if isfield(app,'tmr') && isa(app.tmr,'timer') && isvalid(app.tmr)
            stop(app.tmr);
            delete(app.tmr);
        end
    catch
    end
    try, drawnow limitrate; catch, end
end

function onClose(src, ~)
    try
        safeStopOwnTimer(src);
    catch
    end
    delete(src);
end

function markRegionContains(ax, t_axis, Log_Event, key, yLim, colorRGB, labelText, labelY)
    idx = find(contains(Log_Event, key));
    if isempty(idx), return; end
    x1 = t_axis(idx(1));
    x2 = t_axis(idx(end));
    X = [x1 x2 x2 x1];
    Y = [yLim(1) yLim(1) yLim(2) yLim(2)];
    patch(ax, X, Y, colorRGB, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    text(ax, mean([x1 x2]), labelY, labelText, 'Color', colorRGB, 'HorizontalAlignment','center');
end

%% ===================== GA Objective (keeps your script structure) =====================

function f = gaObjective_KeepScript(x, numSteps, dt, elS, rS, dopHz, dopRateHz, jamAgg, TxP_S, TxP_J0, Noise, Bandwidth)
    AJ_DelaySec      = x(1);
    AJ_NullDepth_dB  = x(2);
    DopplerGuardHzps = x(3);
    JamBoostMax_dB   = x(4);

    [~, ber, thr, evt, bler] = simulateOnce_Final( ...
        numSteps, dt, elS, rS, dopHz, dopRateHz, jamAgg, ...
        TxP_S, TxP_J0, Noise, Bandwidth, ...
        AJ_DelaySec, AJ_NullDepth_dB, DopplerGuardHzps, JamBoostMax_dB);

    % 只评价中段强制干扰窗口（保持你的 Force_Jamming 框架）
    t = (1:numSteps)/numSteps;
    idxWin = (t > 0.35) & (t < 0.65);

    meanThr  = mean(thr(idxWin));
    meanBler = mean(bler(idxWin),'omitnan');
    meanBer  = mean(ber(idxWin),'omitnan');

    % 额外惩罚：过度抑制（避免 GA 一味把 NullDepth 推到极限）
    overNull = max(0, (-AJ_NullDepth_dB - 50)) * 0.8;

    % 目标：最大化吞吐、最小化 BLER/BER
    f = -meanThr + 60*meanBler + 200*meanBer + overNull;
end

%% ===================== Final Simulation (GA+GAN+Doppler participate) =====================

function [sinrLog, berLog, thrLog, eventLog, blerLog] = simulateOnce_Final( ...
    numSteps, dt, elS, rS, dopHz, dopRateHz, jamAgg, ...
    TxP_S, TxP_J0, Noise, Bandwidth, ...
    AJ_DelaySec, AJ_NullDepth_dB, DopplerGuardHzps, JamBoostMax_dB)

    sinrLog = nan(1,numSteps);
    berLog  = nan(1,numSteps);
    thrLog  = nan(1,numSteps);
    blerLog = nan(1,numSteps);
    eventLog = strings(1,numSteps);

    BLK = 1024;

    AJ_Active = false;
    AJ_Timer  = 0;

    for k = 1:numSteps
        Has_Link = elS(k) > 5;

        % 你原“强制干扰窗口”保持不变
        pct = k/numSteps;
        Force_Jamming = (pct > 0.35) && (pct < 0.65);

        % 服务信号
        if Has_Link
            PL_S = 32.45 + 20*log10(rS(k)/1000) + 20*log10(28000);
            P_S_dBm = TxP_S - PL_S + 35;
        else
            P_S_dBm = -200;
        end

        % Doppler 压力：变化率超过门限 → 等效同步恶化（体现多普勒进入模型）
        dopplerStress = abs(dopRateHz(k)) > DopplerGuardHzps;

        if Force_Jamming && Has_Link
            PL_J = PL_S;

            % GAN 调制：更“聪明”的时候更强
            TxP_J = TxP_J0 + JamBoostMax_dB * jamAgg(k);

            % 抗干扰延迟（GA优化）
            if ~AJ_Active
                AJ_Timer = AJ_Timer + dt;
                if AJ_Timer >= AJ_DelaySec
                    AJ_Active = true;
                end
            end

            if AJ_Active
                % 抗干扰抑制深度（GA优化）
                Gain_J = AJ_NullDepth_dB;
                Event_Tag = "Protected";
            else
                Gain_J = 10 + 10*jamAgg(k);
                Event_Tag = "JAMMING!!!";
            end

            % 多普勒压力叠加（工程等效：额外退化）
            if dopplerStress
                Gain_J = Gain_J + 6;
                Event_Tag = Event_Tag + " (DopplerStress)";
            end

            P_J_dBm = TxP_J - PL_J + Gain_J;

        else
            P_J_dBm = -200;
            if ~Has_Link
                Event_Tag = "No Service";
            else
                Event_Tag = "Normal";
            end

            if pct > 0.65
                AJ_Active = false;
                AJ_Timer  = 0;
            end
        end

        % 指标计算（保持你修复后的吞吐量策略）
        if Has_Link
            p_s = 10^(P_S_dBm/10);
            p_j = 10^(P_J_dBm/10);
            p_n = 10^(Noise/10);

            sinr = 10*log10(p_s/(p_j+p_n));
            sinrLog(k) = sinr;

            lin = 10^(sinr/10);
            ber = 0.5*erfc(sqrt(lin/2));
            ber = max(min(ber,0.5),1e-9);
            berLog(k) = ber;

            bler = 1 - (1 - ber)^BLK;
            blerLog(k) = min(max(bler,0),1);

            if ber > 0.2
                thrLog(k) = 0;
            else
                eff = min(log2(1+lin), 6);
                thrLog(k) = Bandwidth * eff * 0.8 / 1e6;
            end
        else
            sinrLog(k) = nan;
            berLog(k)  = nan;
            blerLog(k) = 1;
            thrLog(k)  = 0;
        end

        eventLog(k) = Event_Tag;
    end
end

%% ===================== GAN / Fallback Profile =====================

function jamAgg = makeJammerProfile_GAN_orFallback(N, offAxisDeg, dopRateHz)
    % 输出 jamAgg ∈ [0,1]
    % 有 Deep Learning Toolbox：轻量 GAN 训练几十/几百步（很快）
    % 失败：回退伪GAN（平滑随机包络 + 融合几何/多普勒）

    haveDL = exist('dlnetwork','class') == 8;

    if ~haveDL
        jamAgg = fallbackProfile(N, offAxisDeg, dopRateHz);
        return;
    end

    try
        L = 128; % 训练/生成长度（再上采样到 N）
        x = linspace(1,N,L);
        off = interp1(1:N, offAxisDeg, x, 'linear', 'extrap');
        dr  = interp1(1:N, abs(dopRateHz), x, 'linear', 'extrap');

        % 构造“真实样本”集合（无需外部文件）
        nReal = 160;
        realData = zeros(L, nReal, 'single');
        t = linspace(0,1,L);

        offN = single(min(off / max(off+eps), 1));
        drN  = single(min(dr  / max(dr +eps), 1));

        for i = 1:nReal
            bump = exp(-0.5*((t - (0.5+0.06*randn))/ (0.12+0.03*rand)).^2);
            shape = 0.30 + 0.60*bump;
            shape = shape .* (1 + 0.8*(1-offN)) .* (1 + 0.6*drN);
            shape = shape + 0.08*randn(1,L);
            shape = movmean(shape, 7);
            shape = min(max(shape, 0), 1);
            realData(:,i) = single(shape(:));
        end

        % 轻量 GAN（z->G->xhat，D(x)->prob）
        zDim = 16;
        netG = dlnetwork(layerGraph([
            featureInputLayer(zDim,'Name','z')
            fullyConnectedLayer(128,'Name','g_fc1')
            reluLayer('Name','g_relu1')
            fullyConnectedLayer(L,'Name','g_fc2')
            sigmoidLayer('Name','g_sig')
        ]));

        netD = dlnetwork(layerGraph([
            featureInputLayer(L,'Name','x')
            fullyConnectedLayer(128,'Name','d_fc1')
            leakyReluLayer(0.2,'Name','d_lrelu1')
            fullyConnectedLayer(1,'Name','d_fc2')
            sigmoidLayer('Name','d_sig')
        ]));

        nIter = 200; batch = 24; lr = 1e-3;
        avgG=[]; avgSqG=[]; avgD=[]; avgSqD=[];

        for it = 1:nIter
            idx = randi(nReal,[1 batch]);
            xReal = dlarray(realData(:,idx),'CB');

            z = dlarray(single(randn(zDim,batch)),'CB');
            xFake = forward(netG, z);

            [gradD, lossD] = dlfeval(@dGradients, netD, xReal, xFake); %#ok<NASGU>
            [netD, avgD, avgSqD] = adamupdate(netD, gradD, avgD, avgSqD, it, lr);

            z2 = dlarray(single(randn(zDim,batch)),'CB');
            [gradG, lossG] = dlfeval(@gGradients, netG, netD, z2); %#ok<NASGU>
            [netG, avgG, avgSqG] = adamupdate(netG, gradG, avgG, avgSqG, it, lr);
        end

        % 生成一条
        z = dlarray(single(randn(zDim,1)),'CB');
        y = predict(netG, z);
        y = gather(extractdata(y));
        y = y(:)';

        % 上采样到 N
        jamAgg = interp1(linspace(1,N,L), double(y), 1:N, 'pchip', 'extrap');
        jamAgg = min(max(jamAgg,0),1);

        % 融合物理特征（让 GAN 输出更“贴近对抗”）
        midMask = ((1:N)/N > 0.35) & ((1:N)/N < 0.65);
        offW = 1 ./ (1 + (offAxisDeg/8).^2);
        drW  = min(abs(dopRateHz)/max(abs(dopRateHz)+eps),1);

        jamAgg = jamAgg .* (0.65 + 0.35*midMask) .* (0.6 + 0.7*offW) .* (0.7 + 0.5*drW);
        jamAgg = movmean(jamAgg, 11);
        jamAgg = min(max(jamAgg,0),1);

    catch
        jamAgg = fallbackProfile(N, offAxisDeg, dopRateHz);
    end
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

function jamAgg = fallbackProfile(N, offAxisDeg, dopRateHz)
    t = linspace(0,1,N);
    bump = exp(-0.5*((t-0.5)/0.12).^2);
    base = 0.25 + 0.55*bump + 0.10*randn(1,N);
    base = movmean(base, 17);

    midMask = ((1:N)/N > 0.35) & ((1:N)/N < 0.65);
    offW = 1 ./ (1 + (offAxisDeg/8).^2);
    drW  = min(abs(dopRateHz)/max(abs(dopRateHz)+eps),1);

    jamAgg = base .* (0.65 + 0.35*midMask) .* (0.6 + 0.7*offW) .* (0.7 + 0.5*drW);
    jamAgg = min(max(jamAgg,0),1);
    jamAgg = movmean(jamAgg, 11);
end
