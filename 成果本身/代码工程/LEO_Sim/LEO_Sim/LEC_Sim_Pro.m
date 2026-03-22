%% Project: LEO EMC Final Fixed (Throughput Repaired)
%  Target: 1. 恢复卫星过顶青岛的正确轨迹
%          2. 恢复干扰事件的强制触发
%          3. [修复] 吞吐量曲线为0的问题 (通过提升功率+优化公式)
%  Version: R2021a Stable
%  Author: Gemini

clear; clc; close all;

%% ========================================================================
%  PART 1: 绝对可靠的过顶时间锁定 (回退到 V2 逻辑)
%  ========================================================================
fprintf('Step 1: 计算青岛站过顶时刻...\n');

% 1. 搜索窗口
t_search_start = datetime('now');
t_search_stop  = t_search_start + hours(12);
sc_search = satelliteScenario(t_search_start, t_search_stop, 60);

% 2. 定义青岛站和服务星
gs_temp = groundStation(sc_search, 36.06, 120.38, 'Name', 'Qingdao_Search');
sat_temp = satellite(sc_search, 6371000+1200000, 0, 88, 0, 0, 0);

% 3. 计算可见性
ac_temp = access(sat_temp, gs_temp);
intervals = accessIntervals(ac_temp);

if isempty(intervals)
    % 保底：如果搜不到，使用手动设定的时间
    center_time = datetime('now') + minutes(10);
    fprintf('        [警告] 自动搜索未找到，使用默认时间。\n');
else
    % 锁定最佳过顶时刻
    [~, best_idx] = max(intervals.EndTime - intervals.StartTime);
    center_time = intervals.StartTime(best_idx) + (intervals.EndTime(best_idx) - intervals.StartTime(best_idx))/2;
    fprintf('        [锁定] 最佳过顶时刻: %s\n', char(center_time));
end

% 设定仿真窗：前后各 8 分钟，共 16 分钟
sim_start = center_time - minutes(8);
sim_stop  = center_time + minutes(8);
sample_time = 1;

%% ========================================================================
%  PART 2: 重建场景 (Scene Setup)
%  ========================================================================
fprintf('Step 2: 重建高精度场景...\n');

sc = satelliteScenario(sim_start, sim_stop, sample_time);

% 地面站
gs = groundStation(sc, 36.06, 120.38, 'Name', 'Qingdao_GS');

% 卫星 (使用搜索到的轨道，确保过顶)
satSvc = satellite(sc, 6371000+1200000, 0, 88, 0, 0, 0, 'Name', 'Service_Sat');
% 干扰星 (紧跟其后)
satJam = satellite(sc, 6371000+1200000, 0, 88, 0, 0, -0.5, 'Name', 'Jammer_Sat');

% 可视化
sensSvc = conicalSensor(satSvc, 'MaxViewAngle', 45);
fovSvc = fieldOfView(sensSvc); fovSvc.LineColor = [0 1 0];

sensJam = conicalSensor(satJam, 'MaxViewAngle', 45);
fovJam = fieldOfView(sensJam); fovJam.LineColor = [1 0 0];

ac1 = access(satSvc, gs); ac1.LineColor = [0 1 0];
ac2 = access(satJam, gs); ac2.LineColor = [1 0 0];

%% ========================================================================
%  PART 3: 强制剧本仿真 (Logic & Physics)
%  ========================================================================
fprintf('Step 3: 执行物理层计算...\n');

% AER（服务星 + 干扰星，用于联动）
[azS, elS, rS] = aer(gs, satSvc);
[azJ, elJ, rJ] = aer(gs, satJam);

timeVec = 0:sample_time:seconds(sim_stop - sim_start);
numSteps = min([length(azS), length(azJ), length(timeVec)]);
timeVec = timeVec(1:numSteps);

% 预分配
Log_SINR  = nan(1, numSteps);
Log_BER   = nan(1, numSteps);
Log_Thr   = nan(1, numSteps);
Log_Event = strings(1, numSteps);

% --- 关键修复 1：提升发射功率 (解决吞吐量为0的根本原因) ---
TxP_S = 55;  % 原为40，提升至55dBm，确保基础SNR够高
TxP_J = 75;  % 原为65，同步提升干扰功率，保持压制效果
Noise = -110;
Bandwidth = 100e6; % 100MHz

% 抗干扰状态机
AJ_Active = false;
AJ_Timer = 0;

for k = 1:numSteps
    % 1. 服务链路
    if elS(k) > 5
        PL_S = 32.45 + 20*log10(rS(k)/1000) + 20*log10(28000);
        P_S_dBm = TxP_S - PL_S + 35;
        Has_Link = true;
    else
        P_S_dBm = -200;
        Has_Link = false;
    end

    % 2. 强制干扰剧本 (保留 V2 逻辑) —— 联动修复：与干扰星可见性/距离挂钩
    pct = k / numSteps;
    Force_Window = (pct > 0.35) && (pct < 0.65);   % 中间段强制干扰窗口（保留）
    Has_Jammer   = (elJ(k) > 5);                   % 干扰星可见性门限
    Force_Jamming = Force_Window && Has_Jammer && Has_Link;

    if Force_Jamming && Has_Link
        % 干扰路损使用干扰星真实距离
        PL_J = 32.45 + 20*log10(rJ(k)/1000) + 20*log10(28000);

        if AJ_Active
            Gain_J = -40; % 抗干扰生效
            Event_Tag = "Protected";
        else
            Gain_J = 10;  % 干扰生效
            Event_Tag = "JAMMING!!!";

            % 触发抗干扰计时 (30秒后开启)
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
            AJ_Timer = 0;
        end
    end

    % 3. 指标计算
    if Has_Link
        p_s = 10^(P_S_dBm/10);
        p_j = 10^(P_J_dBm/10);
        p_n = 10^(Noise/10);

        sinr_val = 10*log10(p_s / (p_j + p_n));
        Log_SINR(k) = sinr_val;

        % BER
        lin_sinr = 10^(sinr_val/10);
        ber_val = 0.5 * erfc(sqrt(lin_sinr/2));
        Log_BER(k) = max(min(ber_val, 0.5), 1e-9);

        % --- 关键修复 2：优化吞吐量公式 ---
        if ber_val > 0.2
            Log_Thr(k) = 0;
        else
            eff = log2(1 + lin_sinr);
            eff = min(eff, 6); % 限制最高效率
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
%  PART 4: 结果展示 (Visualization) + 3D严格联动（R2021a稳定版）
%  关键：不用 play(sc)/play(v)，改为“手动步进 CurrentTime”
%  ========================================================================
fprintf('Step 4: 生成图表 + 3D严格联动...\n');

t_axis = (timeVec - timeVec(1))/60;

figure('Name', 'LEO EMC Final Fixed', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% --- SINR ---
ax1 = subplot(3,1,1);
plot(ax1, t_axis, Log_SINR, 'LineWidth', 2, 'Color', '#0072BD'); hold(ax1, 'on');
yline(ax1, 0, 'r--', 'Disconnect');
title(ax1, 'SINR (dB)'); ylabel(ax1, 'dB'); grid(ax1, 'on');
xlim(ax1, [0 max(t_axis)]); ylim(ax1, [-20 50]);

% --- BER ---
ax2 = subplot(3,1,2);
Log_BER_plot = Log_BER;
Log_BER_plot(isnan(Log_BER_plot)) = 1;
Log_BER_plot = max(Log_BER_plot, 1e-9);
semilogy(ax2, t_axis, Log_BER_plot, 'LineWidth', 2, 'Color', '#D95319'); hold(ax2, 'on');
title(ax2, 'Bit Error Rate (BER)'); ylabel(ax2, 'Log Scale');
grid(ax2, 'on'); xlim(ax2, [0 max(t_axis)]); ylim(ax2, [1e-9 1]);

% --- Throughput ---
ax3 = subplot(3,1,3);
area(ax3, t_axis, Log_Thr, 'FaceColor', '#77AC30'); hold(ax3, 'on'); % 只画一次
title(ax3, 'Throughput (Mbps)'); ylabel(ax3, 'Mbps'); xlabel(ax3, 'Time (Minutes)');
grid(ax3, 'on'); xlim(ax3, [0 max(t_axis)]); ylim(ax3, [0 600]);

% 干扰/抗干扰标记（保持你的原逻辑）
axes(ax1); %#ok<LAXES>
idx = find(Log_Event == "JAMMING!!!");
if ~isempty(idx)
    x_r = [t_axis(idx(1)) t_axis(idx(end)) t_axis(idx(end)) t_axis(idx(1))];
    patch(x_r, [-20 -20 50 50], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    text(mean(t_axis(idx)), -10, 'Jamming', 'Color','r', 'HorizontalAlignment','center');
end
idx = find(Log_Event == "Protected");
if ~isempty(idx)
    x_g = [t_axis(idx(1)) t_axis(idx(end)) t_axis(idx(end)) t_axis(idx(1))];
    patch(x_g, [-20 -20 50 50], 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    text(mean(t_axis(idx)), 40, 'AJ Protected', 'Color','g', 'HorizontalAlignment','center');
end

% 联动游标 + 当前点（只更新这些，保证不触发 area 内部bug）
hC1 = xline(ax1, t_axis(1), 'k--', 'LineWidth', 1.5);
hP1 = plot(ax1, t_axis(1), Log_SINR(1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);

hC2 = xline(ax2, t_axis(1), 'k--', 'LineWidth', 1.5);
hP2 = semilogy(ax2, t_axis(1), Log_BER_plot(1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);

hC3 = xline(ax3, t_axis(1), 'k--', 'LineWidth', 1.5);
hP3 = plot(ax3, t_axis(1), Log_Thr(1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);

drawnow;

%% ------------------ 只用3D Viewer（R2021a支持参数） ------------------
fprintf('启动3D视图...\n');
v = satelliteScenarioViewer(sc, 'Basemap', 'darkwater');  % 只用3D（默认Dimension=3D）

PlaybackSpeedMultiplier = 20;

% 关键：R2021a 里用“scenario的CurrentTime”驱动显示，最稳
canSetScenarioTime = isprop(sc, 'CurrentTime');

% 初始化到起点（可选）
if canSetScenarioTime
    try
        sc.CurrentTime = sim_start;
    catch
        canSetScenarioTime = false;
    end
end

% 手动步进：每步推进1秒（sample_time），并按倍速控制 pause
for k = 1:numSteps
    % 如果用户关闭了viewer则退出，避免“对象无效”
    if ~isvalid(v)
        warning('3D Viewer 已关闭/无效，联动停止。');
        break;
    end

    ct = sim_start + seconds((k-1) * sample_time);

    % 推进3D场景时间（驱动卫星位置变化）
    if canSetScenarioTime
        try
            sc.CurrentTime = ct;
        catch
            % 极端情况下写失败就不再尝试写（但竖线仍会动）
            canSetScenarioTime = false;
        end
    end

    % 同步更新曲线游标与当前点
    hC1.Value = t_axis(k);
    hC2.Value = t_axis(k);
    hC3.Value = t_axis(k);

    set(hP1, 'XData', t_axis(k), 'YData', Log_SINR(k));

    ber_k = Log_BER(k);
    if isnan(ber_k), ber_k = 1; end
    ber_k = max(ber_k, 1e-9);
    set(hP2, 'XData', t_axis(k), 'YData', ber_k);

    set(hP3, 'XData', t_axis(k), 'YData', Log_Thr(k));

    drawnow limitrate;
    pause(sample_time / PlaybackSpeedMultiplier);
end
