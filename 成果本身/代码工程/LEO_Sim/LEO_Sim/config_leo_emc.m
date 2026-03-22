%% config_leo_emc.m
%  基于MATLAB R2021a的低轨卫星EMC仿真配置脚本
%  功能：初始化参数、计算轨道几何、生成信道配置结构体

clear; clc; close all;

% =========================================================================
% 1. 全局仿真参数定义
% =========================================================================
simParams = struct();
simParams.CarrierFreq = 2e9;        % 载波频率：2 GHz (S-band)
simParams.Bandwidth   = 10e6;       % 带宽：10 MHz
simParams.SubcarrierSpacing = 15e3; % 子载波间隔：15 kHz
simParams.SampleRate  = 15.36e6;    % 采样率 (符合LTE/5G标准)
simParams.SimDuration = 10;         % 仿真时长 (秒)
simParams.SpeedOfLight = physconst('LightSpeed');

% =========================================================================
% 2. 轨道几何引擎 (基于 Satellite Communications Toolbox R2021a)
% =========================================================================
disp('正在初始化卫星场景与轨道推演...');

% 定义仿真起止时间
startTime = datetime(2021, 6, 21, 12, 0, 0);
stopTime  = startTime + seconds(simParams.SimDuration);
sampleTime = 0.1; % 几何参数更新周期 (100ms)

% 创建场景对象
sc = satelliteScenario(startTime, stopTime, sampleTime);

% 定义LEO卫星 (以Starlink为例的轨道参数)
% 半长轴 = 地球半径 + 轨道高度 (550km)
semiMajorAxis = 6378137 + 550000;
eccentricity = 0.001;   % 近圆轨道
inclination = 53;       % 倾角
raan = 0;
argPeriapsis = 0;
trueAnomaly = 0;

% 添加卫星
sat = satellite(sc, semiMajorAxis, eccentricity, inclination,...
                raan, argPeriapsis, trueAnomaly,...
                'Name', 'LEO_Sat_Serving');

% 定义地面站 (以北京为例)
lat = 39.9042;
lon = 116.4074;
gs = groundStation(sc, lat, lon, 'Name', 'Ground_Terminal');

% 执行可见性分析
ac = access(sat, gs);

% --- 计算Simulink所需的动态链路参数 ---
% 使用 aer 函数获取 方位角(Az)、仰角(El)、斜距(Range)
[az, el, r] = aer(gs, sat);

% 计算多普勒频移 (Doppler Shift)
% 公式: fd = -(v_radial / c) * fc
% 【修复：计算径向速度】
rangeRate = diff(r) ./ sampleTime; 

% 【修复：计算多普勒，注意diff后长度少1】
dopplerShift = -(rangeRate ./ simParams.SpeedOfLight) * simParams.CarrierFreq;

% 计算自由空间路损 (FSPL)
pathLoss = fspl(r, simParams.SpeedOfLight./ simParams.CarrierFreq);

% 创建 Timeseries 对象供 Simulink 读取
% 注意：需要确保时间向量与数据长度一致
timeVec = seconds(sc.StartTime - sc.StartTime : sampleTime : sc.StopTime - sc.StartTime)';

% 数据维度修正 (确保为列向量)
if isrow(dopplerShift), dopplerShift = dopplerShift'; end
if isrow(pathLoss), pathLoss = pathLoss'; end

% 【修复：数据补齐】
% diff导致长度少1，补齐最后一位以匹配时间轴
if length(dopplerShift) < length(timeVec)
    dopplerShift = [dopplerShift; dopplerShift(end)];
end
% pathLoss通常与timeVec一致，但为保险起见做个检查
if length(pathLoss) < length(timeVec)
    pathLoss = [pathLoss; pathLoss(end)];
end

% 生成 Simulink 输入变量 (这些变量必须存在于Workspace中)
ts_doppler   = timeseries(dopplerShift, timeVec);
ts_pathloss  = timeseries(pathLoss, timeVec);
ts_elevation = timeseries(el, timeVec); 

% =========================================================================
% 3. 信道模型配置
% =========================================================================
ntn_tdl_a = struct();
ntn_tdl_a.DelaySpread = 100e-9; % 100 ns
ntn_tdl_a.Taps = [0, 1.0811, 2.8416] * ntn_tdl_a.DelaySpread;
ntn_tdl_a.Powers = [0, -4.675, -6.482]; % dB
ntn_tdl_a.Fading = 'Rayleigh';

disp('配置完成！变量 ts_doppler, ts_pathloss 已加载至工作区。');
disp('现在可以运行 build_leo_emc_model 了。');