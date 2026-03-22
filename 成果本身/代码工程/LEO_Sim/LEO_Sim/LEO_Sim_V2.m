%% Project: LEO EMC Final + 3D Viewer Linked Dashboard (R2021a)
%  Target:
%    1) 指标曲线与 3D satelliteScenarioViewer 播放“真实联动”（竖线跟随 Viewer.CurrentTime）
%    2) Viewer 播放速度变化时（PlaybackSpeedMultiplier/速度滑条），竖线推进速度自动同步
%    3) 按文档思路：事件驱动步进 + 仪表盘(uilamp/事件日志) + ITU-T K.157(A/B/C)判据映射
%
%  Version: R2021a
%  Notes:
%    - 不再写 v.CurrentTime（你之前报错点）；仅“读” v.CurrentTime 来驱动竖线
%    - 避免 Area 动态更新导致的索引越界：吞吐量改为 line
%    - Basemap 优先用 'none'（减少在线地形/底图导致的 globe/terrain 异常）；失败则回退默认

clear; clc; close all;

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
%  PART 3: 执行物理层计算 (先离线算好全量序列，联动时只取当前点)
%  ========================================================================
fprintf('Step 3: 执行物理层计算...\n');

[azS, elS, rS] = aer(gs, satSvc);
[azJ, elJ, rJ] = aer(gs, satJam); %#ok<NASGU>

timeVec = 0:sample_time:seconds(sim_stop - sim_start);
numSteps = min([length(elS), length(timeVec)]);
timeVec  = timeVec(1:numSteps);

Log_SINR  = nan(1, numSteps);
Log_BER   = nan(1, numSteps);
Log_Thr   = nan(1, numSteps);
Log_Event = strings(1, numSteps);
Log_OffAxisDeg = nan(1, numSteps); % 事件检测用（不改变原干扰剧本，只记录）

TxP_S = 55;  % dBm
TxP_J = 75;  % dBm
Noise = -110;
Bandwidth = 100e6; % 100MHz

AJ_Active = false;
AJ_Timer  = 0;

for k = 1:numSteps
    % 1) 服务链路
    if elS(k) > 5
        PL_S   = 32.45 + 20*log10(rS(k)/1000) + 20*log10(28000);
        P_S_dBm = TxP_S - PL_S + 35;
        Has_Link = true;
    else
        P_S_dBm = -200;
        Has_Link = false;
    end

    % 2) 计算“共线”度量（仅用于日志/仪表盘展示，不改变原干扰触发）
    %    这里采用简单方位/俯仰平面近似角差
    offAxis = sqrt((azS(k)-azJ(k)).^2 + (elS(k)-elJ(k)).^2);
    Log_OffAxisDeg(k) = offAxis;

    % 3) 强制干扰剧本（保持你原来的 Force_Jamming 逻辑）
    pct = k / numSteps;
    Force_Jamming = (pct > 0.35) && (pct < 0.65);

    if Force_Jamming && Has_Link
        PL_J = PL_S; % 近似干扰路损

        if AJ_Active
            Gain_J = -40;
            Event_Tag = "Protected";
        else
            Gain_J = 10;
            Event_Tag = "JAMMING!!!";

            AJ_Timer = AJ_Timer + 1;
            if AJ_Timer > 30
                AJ_Active = true;
            end
        end
        P_J_dBm = TxP_J - PL_J + Gain_J;
    else
        P_J_dBm = -200;
        Event_Tag = "Normal";
        if ~Has_Link, Event_Tag = "No Service"; end

        if pct > 0.65
            AJ_Active = false;
            AJ_Timer  = 0;
        end
    end

    % 4) 指标计算
    if Has_Link
        p_s = 10^(P_S_dBm/10);
        p_j = 10^(P_J_dBm/10);
        p_n = 10^(Noise/10);

        sinr_val = 10*log10(p_s / (p_j + p_n));
        Log_SINR(k) = sinr_val;

        lin_sinr = 10^(sinr_val/10);
        ber_val = 0.5 * erfc(sqrt(lin_sinr/2));
        Log_BER(k) = max(min(ber_val, 0.5), 1e-9);

        if ber_val > 0.2
            Log_Thr(k) = 0;
        else
            eff = log2(1 + lin_sinr);
            eff = min(eff, 6);
            Log_Thr(k) = Bandwidth * eff * 0.8 / 1e6; % Mbps
        end
    else
        Log_SINR(k) = nan;
        Log_BER(k)  = nan;
        Log_Thr(k)  = 0;
    end

    Log_Event(k) = Event_Tag;
end

%% ========================================================================
%  PART 4: 生成仪表盘 + 3D 联动（竖线跟随 Viewer.CurrentTime）
%  ========================================================================
fprintf('Step 4: 生成图表 + 3D联动...\n');

% 时间轴（分钟）
t_axis = (timeVec - timeVec(1))/60;

% ---------- Dashboard (uifigure) ----------
dash = uifigure('Name','LEO EMC Dashboard (Linked to 3D Viewer)', ...
    'Color','w','Position',[50 50 1280 820]);

gl = uigridlayout(dash, [3 4]);
gl.RowHeight    = {34,'1x','1x'};
gl.ColumnWidth  = {'1x','1x','1x',380};
gl.Padding      = [10 10 10 10];
gl.RowSpacing   = 8;
gl.ColumnSpacing= 10;

titleLbl = uilabel(gl, 'Text','LEO EMC Dashboard (竖线随3D Viewer时间推进)', ...
    'FontSize',16,'FontWeight','bold');
titleLbl.Layout.Row = 1; titleLbl.Layout.Column = [1 4];

% 三个 uiaxes
axSINR = uiaxes(gl); axSINR.Layout.Row=2; axSINR.Layout.Column=[1 3];
axBER  = uiaxes(gl); axBER.Layout.Row=3; axBER.Layout.Column=[1 3];

axSINR.Toolbar.Visible = 'off'; axBER.Toolbar.Visible = 'off';
axSINR.Interactions = []; axBER.Interactions = [];

% 右侧状态区
right = uigridlayout(gl,[10 1]);
right.Layout.Row = [2 3];
right.Layout.Column = 4;
right.RowHeight = {24,24,24,24,24,24,140,24,'1x',24};
right.Padding = [0 0 0 0];

lblTime = uilabel(right,'Text','Current Time: -','FontWeight','bold');
lblSINR = uilabel(right,'Text','SINR: - dB');
lblBER  = uilabel(right,'Text','BER: -');
lblThr  = uilabel(right,'Text','Throughput: - Mbps');
lblOff  = uilabel(right,'Text','Off-axis: - deg');

lblCrit = uilabel(right,'Text','K.157 Criterion: -','FontWeight','bold');

% 合规灯（绿色=通过，红色=不通过；阈值用 BLER>0.1）
lampLbl = uilabel(right,'Text','Compliance Lamp (BLER<=0.1):');
lamp = uilamp(right);
lamp.Color = [0 1 0];

% 事件日志表
tbl = uitable(right, ...
    'Data', cell(0,3), ...
    'ColumnName', {'Time','Event','Detail'}, ...
    'ColumnEditable', [false false false]);
tbl.Layout.Row = 7;

% 吞吐量小图（单独一个 uiaxes）
axTHR = uiaxes(right);
axTHR.Layout.Row = 9;
axTHR.Toolbar.Visible = 'off';
axTHR.Interactions = [];

% 速度显示
lblSpeed = uilabel(right,'Text','Viewer Speed: -');

% 结束提示
lblHint = uilabel(right,'Text','提示：在3D Viewer里改播放速度，竖线会自动同步。', ...
    'WordWrap','on');

% ---------- 绘图（静态曲线 + 竖线游标） ----------
% SINR
plot(axSINR, t_axis, Log_SINR, 'LineWidth', 2); grid(axSINR,'on');
title(axSINR,'SINR (dB)'); ylabel(axSINR,'dB');
xlim(axSINR,[0 max(t_axis)]); ylim(axSINR,[-20 50]);
yline(axSINR,0,'r--','Disconnect');

% BER
semilogy(axBER, t_axis, Log_BER, 'LineWidth', 2); grid(axBER,'on');
title(axBER,'Bit Error Rate (BER)'); ylabel(axBER,'Log Scale'); xlabel(axBER,'Time (Minutes)');
xlim(axBER,[0 max(t_axis)]); ylim(axBER,[1e-9 1]);

% THR
plot(axTHR, t_axis, Log_Thr, 'LineWidth', 2); grid(axTHR,'on');
title(axTHR,'Throughput (Mbps)'); ylabel(axTHR,'Mbps'); xlabel(axTHR,'Time (Minutes)');
xlim(axTHR,[0 max(t_axis)]); ylim(axTHR,[0 600]);

% 干扰/抗干扰区块（静态标注）
markRegion(axSINR, t_axis, Log_Event, "JAMMING!!!",  [-20 50], [1 0 0], 'Jamming', -10);
markRegion(axSINR, t_axis, Log_Event, "Protected",  [-20 50], [0 1 0], 'AJ Protected', 40);

% 三条竖线游标（真正联动的关键对象）
cur1 = xline(axSINR, 0, 'k-', 'LineWidth', 1.8);
cur2 = xline(axBER,  0, 'k-', 'LineWidth', 1.8);
cur3 = xline(axTHR,  0, 'k-', 'LineWidth', 1.8);

% ---------- 3D Viewer ----------
fprintf('启动3D视图...\n');

% 建议：禁用 AutoSimulate，用 Viewer 播放推进 CurrentTime
% 但“写 CurrentTime”你那边会崩，所以我们采用：让 Viewer 自己 play，
% 然后用 timer 周期性读取 v.CurrentTime 来更新竖线/仪表盘。
try
    v = satelliteScenarioViewer(sc, 'Basemap', 'none', 'PlaybackSpeedMultiplier', 20, 'Dimension', '3D');
catch
    try
        v = satelliteScenarioViewer(sc, 'PlaybackSpeedMultiplier', 20, 'Dimension', '3D');
    catch
        v = satelliteScenarioViewer(sc);
        try
            v.PlaybackSpeedMultiplier = 20;
        catch
        end
    end
end

% 播放（由 Viewer 自己推进时间）
try
    play(v);
catch
    % 某些安装下 play(v) 可能不可用，回退 play(sc)
    try
        play(sc);
    catch
        % 如果都不行，就只显示 Viewer
        try
            show(sc);
        catch
        end
    end
end

% ---------- 联动计时器（核心：读 v.CurrentTime → 映射到 k → 更新竖线） ----------
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

app.dash = dash;
app.v = v;

app.lblTime = lblTime;
app.lblSINR = lblSINR;
app.lblBER  = lblBER;
app.lblThr  = lblThr;
app.lblOff  = lblOff;
app.lblCrit = lblCrit;
app.lblSpeed= lblSpeed;

app.lamp = lamp;
app.tbl  = tbl;

app.cur1 = cur1;
app.cur2 = cur2;
app.cur3 = cur3;

% K.157 相关：简单映射参数
app.BlockLenBits = 1024;      % 用于 BER->BLER 的近似映射
app.BLER_Th = 0.1;            % 合规阈值（文档示例）
app.Trecover = 30;            % 秒：判据B恢复窗（文档给出概念 Trecover）

% 状态机：判据B/C判定用
app.inOutage = false;
app.outageStartTime = NaT;
app.outageCause = "";

% 事件去重（只在事件变化时记一次）
app.lastEventTag = "";

guidata(dash, app);

tmr = timer( ...
    'ExecutionMode','fixedSpacing', ...
    'Period', 0.05, ...             % 20 Hz 刷新
    'BusyMode','drop', ...
    'TimerFcn', @(~,~)onTick(dash));

% 关闭窗口时清理 timer
dash.CloseRequestFcn = @(src,evt)onClose(src,evt,tmr);

start(tmr);

%% ============================ Local Functions ============================

function onTick(dashFig)
    if ~isvalid(dashFig)
        return;
    end
    app = guidata(dashFig);

    % Viewer 被关掉/无效：停止更新
    if ~isfield(app,'v') || isempty(app.v) || ~isvalid(app.v)
        safeStopTimer(dashFig);
        return;
    end

    % 读取 Viewer 当前时间（只读！避免你之前遇到的 set.CurrentTime 崩溃）
    try
        ct = app.v.CurrentTime;
    catch
        safeStopTimer(dashFig);
        return;
    end

    % 映射到步序号 k（严格按 sample_time 对齐）
    dt = seconds(ct - app.sim_start);
    k = round(dt / app.sample_time) + 1;
    k = max(1, min(app.numSteps, k));

    xMin = app.t_axis(k); % minutes

    % 更新竖线
    if isvalid(app.cur1), app.cur1.Value = xMin; end
    if isvalid(app.cur2), app.cur2.Value = xMin; end
    if isvalid(app.cur3), app.cur3.Value = xMin; end

    % 取当前指标
    sinr = app.Log_SINR(k);
    ber  = app.Log_BER(k);
    thr  = app.Log_Thr(k);
    offd = app.Log_OffAxisDeg(k);
    tag  = app.Log_Event(k);

    % BER->BLER 近似（用于合规灯；文档示例是 BLER>0.1 红灯）
    if isnan(ber)
        bler = 1;
    else
        bler = 1 - (1 - min(max(ber,0),1)).^(app.BlockLenBits);
    end

    % K.157 判据映射（A/B/C 的一个“可运行工程版”简化）
    % A: 持续满足 BLER<=0.1 且吞吐量>0
    % B: 允许短时超标/掉线，但干扰移除后 Trecover 秒内恢复到 BLER<=0.1
    % C: 超过 Trecover 仍未恢复
    nowTime = ct;

    isBad = (bler > app.BLER_Th) || strcmp(tag,"No Service") || (thr <= 0);

    if ~app.inOutage && isBad
        app.inOutage = true;
        app.outageStartTime = nowTime;
        app.outageCause = string(tag);
    elseif app.inOutage && ~isBad
        % 恢复：判定 B / A
        Tloss = seconds(nowTime - app.outageStartTime);
        if Tloss <= app.Trecover
            crit = "B (Transient Loss, Auto-Recovered)";
        else
            crit = "C (Manual Intervention implied)";
        end
        app.inOutage = false;
        app.outageStartTime = NaT;
        app.outageCause = "";
        % 恢复事件写日志
        app = pushEvent(app, nowTime, "Recovered", sprintf("Tloss=%.1fs, %s", Tloss, crit));
    end

    if app.inOutage
        Tloss = seconds(nowTime - app.outageStartTime);
        if Tloss <= app.Trecover
            crit = sprintf("B? (Recovering... %.1fs/%.0fs)", Tloss, app.Trecover);
        else
            crit = sprintf("C (Exceeded Trecover: %.1fs)", Tloss);
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

    % 速度显示（如果可读）
    spdTxt = "Viewer Speed: -";
    try
        if isprop(app.v,'PlaybackSpeedMultiplier')
            spdTxt = sprintf("Viewer Speed: x%.2f", app.v.PlaybackSpeedMultiplier);
        end
    catch
    end

    % UI 文本更新
    app.lblTime.Text  = sprintf("Current Time: %s", char(ct));
    app.lblSINR.Text  = sprintf("SINR: %s dB", fmtNum(sinr, 2));
    app.lblBER.Text   = sprintf("BER: %s", fmtSci(ber));
    app.lblThr.Text   = sprintf("Throughput: %s Mbps", fmtNum(thr, 2));
    app.lblOff.Text   = sprintf("Off-axis: %s deg", fmtNum(offd, 2));
    app.lblCrit.Text  = sprintf("K.157 Criterion: %s", crit);
    app.lblSpeed.Text = spdTxt;

    % 事件日志：仅在 Event_Tag 变化时记一次（避免刷屏）
    if app.lastEventTag ~= string(tag)
        app = pushEvent(app, nowTime, "State", char(tag));
        app.lastEventTag = string(tag);
    end

    guidata(dashFig, app);

    % 播放结束：停止 timer
    if k >= app.numSteps
        safeStopTimer(dashFig);
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
    % 最多保留 200 行
    data = app.tbl.Data;
    if size(data,1) >= 200
        data = data(end-150:end, :);
    end
    newRow = {char(t), evt, detail};
    app.tbl.Data = [data; newRow];
end

function safeStopTimer(dashFig)
    % 从 CloseRequestFcn/TimerFcn 都可调用：尽量安全停止 timer
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
    % 轻触刷新一次
    if isvalid(dashFig)
        drawnow limitrate;
    end
end

function onClose(src, ~, tmr)
    % 关闭 dashboard 时，清 timer
    try
        if isa(tmr,'timer') && isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
    catch
    end
    delete(src);
end

function markRegion(ax, t_axis, Log_Event, tag, yLim, colorRGB, labelText, labelY)
    idx = find(Log_Event == tag);
    if isempty(idx), return; end
    x1 = t_axis(idx(1));
    x2 = t_axis(idx(end));
    X = [x1 x2 x2 x1];
    Y = [yLim(1) yLim(1) yLim(2) yLim(2)];
    p = patch(ax, X, Y, colorRGB, 'FaceAlpha', 0.15, 'EdgeColor', 'none'); %#ok<NASGU>
    text(ax, mean([x1 x2]), labelY, labelText, 'Color', colorRGB, 'HorizontalAlignment','center');
end
