%% Project: LEO Satellite EMC & Anti-Jamming Digital Twin Simulation
%  Based on: Research Document 
%  Target Version: MATLAB R2021a or later
%  Author: Gemini (Assisting User)

clear; clc; close all;

%% ========================================================================
%  PART 1: AI-Driven Smart Interference Generation (Genetic Algorithm)
%  对应文档章节: 7.2 遗传算法（GA）搜寻“黑天鹅”场景 
%  ========================================================================
disp('正在运行AI优化模块：寻找最恶劣的干扰轨道配置...');

% 定义优化变量：[干扰星升交点赤经(RAAN), 干扰星倾角(Inc)]
% 范围：RAAN [0, 360], Inc [80, 100] (典型的极轨或近极轨)
lb = [0, 80];
ub = [360, 100];

% 遗传算法设置
options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 10, 'MaxGenerations', 5);

% 运行GA，目标函数为 minimize_sinr (即寻找最小SINR)
% 注意：为了演示速度，这里简化了适应度函数的评估时间
[best_jam_params, min_val] = ga(@evaluate_interference_scenario, 2, [], [], [], [], lb, ub, [], options);

fprintf('AI优化完成。最恶劣干扰配置 -> RAAN: %.2f deg, Inc: %.2f deg\n', best_jam_params(1), best_jam_params(2));

%% ========================================================================
%  PART 2: Visualization & Dynamic Simulation Execution
%  对应文档章节: 5. MATLAB R2021a 深度开发实施步骤 
%  ========================================================================

% --- 1. 初始化场景 (Scenario Layer) [cite: 91] ---
startTime = datetime(2026, 1, 1, 12, 0, 0);
stopTime = startTime + minutes(10); % 仿真10分钟
sampleTime = 1; % 1秒步长
sc = satelliteScenario(startTime, stopTime, sampleTime);

% --- 2. 部署蓝方资产 (Service System) [cite: 92] ---
% 使用开普勒参数模拟 OneWeb 类似轨道
% SemiMajorAxis: 7000km, Ecc: 0, Inc: 87
satBlue = satellite(sc, 7000000, 0, 87, 0, 0, 0, 'Name', 'Service_Sat (Blue)');

% 部署地面站 (Victim) [cite: 93]
% 位置设在青岛附近 (36.0N, 120.3E)，符合你的关注点
gs = groundStation(sc, 36.06, 120.38, 'Name', 'Victim_UT (Qingdao)');

% --- 3. 部署红方资产 (Jammer System) [cite: 94] ---
% 使用 AI 优化得到的参数部署干扰星
satRed = satellite(sc, 7000000, 0, best_jam_params(2), best_jam_params(1), 0, 0, 'Name', 'Jammer_Sat (Red)');

% --- 4. 具象化波束与传感器 (Visualization Layer) [cite: 97, 101] ---

% 4.1 蓝方服务波束 (绿色) [cite: 99]
sensService = conicalSensor(satBlue, 'Name', 'Service_Beam', 'MaxViewAngle', 15);
fovService = fieldOfView(sensService);
fovService.LineColor = [0 1 0]; % Green [cite: 100]

% 4.2 红方干扰波束 (红色) [cite: 101]
sensJammer = conicalSensor(satRed, 'Name', 'Jamming_Beam', 'MaxViewAngle', 20);
fovJammer = fieldOfView(sensJammer);
fovJammer.LineColor = [1 0 0]; % Red [cite: 102]

% 4.3 建立链路可视连线
% 服务链路 (动态颜色将在循环中控制)
ac_svc = access(satBlue, gs); 
ac_svc.LineColor = [0 1 0]; % 初始为绿色

% 干扰链路 (用于计算几何，不需要一直显示，但在抗干扰锁定时刻显示黄色)
ac_jam = access(satRed, gs);
ac_jam.LineColor = [1 0 0]; % 初始设为红色，平时隐藏

% --- 5. 启动可视化器 [cite: 121] ---
v = satelliteScenarioViewer(sc);
%v.ShowDetails = false;
%title(v, 'Satellite EMC & Anti-Jamming Digital Twin');

% --- 6. 物理层干扰计算引擎 (Physics Loop) [cite: 107] ---
% 预分配数据记录 (R2021a 修正版)
% 手动计算总步数：(结束时间 - 开始时间) / 步长 + 1
duration_sec = seconds(stopTime - startTime);
numSteps = floor(duration_sec / sampleTime) + 1;

SINR_History = zeros(1, numSteps);
Time_History = 0:sampleTime:(numSteps-1)*sampleTime; % 创建用于绘图的时间轴
% 仿真参数
TxPower_S = 40; % dBm
TxPower_J = 60; % dBm (强干扰)
NoiseFloor = -100; % dBm

disp('开始动态仿真循环...');

% ================= R2021a 兼容性修复开始 =================

% 1. 【新增】 在循环外一次性获取所有几何数据
% R2021a 的 aer 函数会直接返回整个时间段的数据数组
[az_S_list, el_S_list, r_S_list] = aer(gs, satBlue);
[az_J_list, el_J_list, r_J_list] = aer(gs, satRed);

disp('开始动态仿真计算 (R2021a 模式)...');

for t = 1:numSteps
    % 2. 【已删除】 advance(sc); (这是报错的根源，不要加)
    
    % 3. 【修改】 按时间步索引 t 读取数据，而不是实时计算
    az_S = az_S_list(t); el_S = el_S_list(t); r_S = r_S_list(t);
    az_J = az_J_list(t); el_J = el_J_list(t); r_J = r_J_list(t);
    
    % --- 以下物理层计算逻辑保持不变 ---
    
    % 只有当卫星可见（仰角>0）时才计算
    if el_S > 0
        % Friis 路径损耗计算
        PathLoss_S = 32.45 + 20*log10(r_S/1000) + 20*log10(28000); 
        Power_S = TxPower_S - PathLoss_S + 30; 
        
        if el_J > 0
            PathLoss_J = 32.45 + 20*log10(r_J/1000) + 20*log10(28000);
            
            % MVDR 模拟逻辑
            angle_diff = abs(az_S - az_J); 
            if angle_diff > 5 
                Jamming_Gain = -10; % 零陷
                Is_Nulling_Active = true;
            else
                Jamming_Gain = 10;  % 旁瓣
                Is_Nulling_Active = false;
            end
            Power_J = TxPower_J - PathLoss_J + Jamming_Gain;
        else
            Power_J = -200; 
            Is_Nulling_Active = false;
        end
        
        % SINR 计算
        p_s_mw = 10^(Power_S/10);
        p_j_mw = 10^(Power_J/10);
        n_mw = 10^(NoiseFloor/10);
        
        sinr_val = 10*log10(p_s_mw / (p_j_mw + n_mw));
        SINR_History(t) = sinr_val;
        
        % 更新可视化颜色 (R2021a 中只有最后时刻生效，不会报错)
        if sinr_val < 0 
            ac_svc.LineColor = [1 0 0]; 
        elseif sinr_val < 10
            ac_svc.LineColor = [1 0.6 0]; 
        else
            ac_svc.LineColor = [0 1 0]; 
        end
        
        if Is_Nulling_Active && p_j_mw > n_mw * 10
             ac_jam.LineColor = [1 1 0]; 
        else
             ac_jam.LineColor = [1 0 0]; 
        end
        
    else
        SINR_History(t) = NaN;
    end
end

% 4. 【新增】 计算完成后播放 3D 动画
disp('计算完成，正在播放 3D 场景...');
play(v);

% ================= R2021a 兼容性修复结束 =================
disp('仿真结束。生成SINR报告...');
figure;
plot(Time_History, SINR_History, 'LineWidth', 1.5);
yline(0, 'r--', '通信中断阈值 (K.157 Criteria C)');
yline(10, 'y--', '性能降级阈值 (K.157 Criteria B)');
xlabel('Time (s)'); ylabel('SINR (dB)');
title('抗干扰性能仿真结果 (SINR vs Time)');
grid on;


%% ========================================================================
%  Helper Function: Fitness Function for Genetic Algorithm
%  对应: 7.2 遗传算法（GA） 
%  ========================================================================
function fit = evaluate_interference_scenario(x)
    % x(1) = RAAN, x(2) = Inclination
    % 创建临时的极简场景进行快速评估
    
    try
        t_start = datetime(2026, 1, 1, 12, 0, 0);
        t_stop = t_start + minutes(5); % 仅评估5分钟窗口
        sc_temp = satelliteScenario(t_start, t_stop, 10); % 粗步长
        
        % 蓝方 (固定)
        satB = satellite(sc_temp, 7000000, 0, 87, 0, 0, 0);
        gs = groundStation(sc_temp, 36.06, 120.38);
        
        % 红方 (由GA控制变量 x)
        satR = satellite(sc_temp, 7000000, 0, x(2), x(1), 0, 0);
        
        % 快速计算可见性
        ac = access(satR, gs);
        [is_access, ~, ~, ~] = accessStatus(ac);
        
        % 适应度函数设计：
        % 我们希望找到让干扰时间最长、干扰距离最近的参数
        % Fit = -1 * (可见时间比例 / 平均距离)
        % 距离越小，分母越小，Fit越负（ga求最小值）
        
        if any(is_access)
            [~, ~, r_J] = aer(gs, satR);
            avg_dist = mean(r_J(is_access));
            duration = sum(is_access);
            fit = -1 * (duration * 10000 / avg_dist); 
        else
            fit = 0; % 无干扰，适应度最差
        end
        
    catch
        fit = 100; % 惩罚错误
    end
end