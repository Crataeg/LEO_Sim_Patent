%% run_v24_manual.m
%  功能：LEO 5G NTN EMC 仿真 - V24 (手动启动版)
%  特点：
%  1. 只负责构建模型和准备数据，绝不自动运行仿真。
%  2. 包含所有之前的修复（子系统清理、Scope保底、维度锁定）。

clear; clc; close all;
disp('======================================================');
disp('   LEO 5G NTN EMC Simulation - V24 (Manual Run)       ');
disp('======================================================');

% =========================================================================
% [Step 1] 生成数据 (Data Generation)
% =========================================================================
disp('[Step 1] 正在生成仿真数据...');
simParams = struct('CarrierFreq', 2e9, 'SampleRate', 15.36e6, 'SimDuration', 10);
simParams.SpeedOfLight = physconst('LightSpeed');

% 时间轴
sampleTime = 0.1;
t_vec = (0 : sampleTime : simParams.SimDuration)';

% 轨道模型
h = 550000;
v_sat = sqrt(3.986e14 / (6371000 + h));
dist_vec = sqrt(h^2 + (v_sat .* (t_vec - simParams.SimDuration/2)).^2);

% 物理参数
lambda = simParams.SpeedOfLight / simParams.CarrierFreq;
v_radial = [0; diff(dist_vec) ./ sampleTime];
doppler_data = -(v_radial ./ simParams.SpeedOfLight) * simParams.CarrierFreq;
pathloss_data = fspl(dist_vec, lambda);

% 写入工作区
assignin('base', 'bp_time', t_vec);
assignin('base', 'table_dop', doppler_data);
assignin('base', 'table_pl', pathloss_data);

% =========================================================================
% [Step 2] 构建模型 (Model Build)
% =========================================================================
disp('[Step 2] 正在构建 Simulink 模型...');
modelName = 'LEO_EMC_System';

if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist([modelName '.slx'], 'file'), delete([modelName '.slx']); end

new_system(modelName);
open_system(modelName);

% 2.1 全局设置
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', '1000/15.36e6'); 
set_param(modelName, 'StopTime', '10');

% -------------------------------------------------------------------------
% 2.2 基础模块
% -------------------------------------------------------------------------
add_block('simulink/Sources/Clock', [modelName '/Sim_Clock'], 'Position', [20, 250, 40, 270]);
add_block('simulink/Lookup Tables/1-D Lookup Table', [modelName '/Doppler_Table'],...
    'Position', [100, 200, 160, 230], 'BreakpointsForDimension1', 'bp_time',...
    'Table', 'table_dop', 'InterpMethod', 'Linear', 'ExtrapMethod', 'Clip');
add_block('simulink/Lookup Tables/1-D Lookup Table', [modelName '/PathLoss_Table'],...
    'Position', [100, 300, 160, 330], 'BreakpointsForDimension1', 'bp_time',...
    'Table', 'table_pl', 'InterpMethod', 'Linear', 'ExtrapMethod', 'Clip');

% 发射机
txPath = [modelName '/Tx_Gen'];
add_block('simulink/User-Defined Functions/MATLAB Function', txPath, 'Position', [50, 80, 150, 130]);
txScript = sprintf([...
    'function tx = fcn()\n persistent wave idx\n FRAME_SIZE=1000;\n',...
    'coder.varsize(''tx'', [1000 1], [0 0]);\n',...
    'if isempty(wave), rng(1); d=complex(randn(3e5,1),randn(3e5,1)); wave=d/max(abs(d)); idx=1; end\n',...
    'if idx+FRAME_SIZE-1>length(wave), idx=1; end\n',...
    'tx=complex(zeros(FRAME_SIZE,1)); tx(:)=wave(idx:idx+FRAME_SIZE-1); idx=idx+FRAME_SIZE;\n']);
configObj = sfroot; chart = configObj.find('-isa', 'Stateflow.EMChart', 'Path', txPath);
if ~isempty(chart), chart.Script = txScript; end
add_block('simulink/Signal Attributes/Signal Specification', [modelName '/Tx_Spec'],...
    'Position', [180, 95, 220, 115], 'Dimensions', '[1000 1]', 'SignalType', 'complex');

% -------------------------------------------------------------------------
% 2.3 信道子系统
% -------------------------------------------------------------------------
subSys = [modelName '/Channel_Model'];
add_block('simulink/Ports & Subsystems/Subsystem', subSys, 'Position', [350, 80, 550, 250]);

% 清理旧内容 (循环删除法，最稳健)
lines = find_system(subSys, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lines), delete_line(lines); end
blocks = find_system(subSys, 'SearchDepth', 1, 'LookUnderMasks', 'all');
blocksToDelete = setdiff(blocks, subSys);
for i = 1:length(blocksToDelete), delete_block(blocksToDelete{i}); end

add_block('simulink/Sources/In1', [subSys '/Tx'], 'Position', [20, 50, 50, 64]);
add_block('simulink/Sources/In1', [subSys '/Doppler'], 'Position', [20, 150, 50, 164]);
add_block('simulink/Sources/In1', [subSys '/PathLoss'], 'Position', [20, 250, 50, 264]);
add_block('simulink/Sinks/Out1',  [subSys '/Rx'], 'Position', [800, 50, 830, 64]);

% 多普勒计算
dopPath = [subSys '/Doppler_Math'];
add_block('simulink/User-Defined Functions/MATLAB Function', dopPath, 'Position', [150, 40, 250, 100]);
dopScript = sprintf([...
    'function y = fcn(u, f_off)\n persistent lp\n coder.varsize(''y'',[1000 1],[0 0]);\n',...
    'if isempty(lp), lp=0; end\n Ts=1/15.36e6; t=(1:1000).''*Ts;\n',...
    'inc=2*pi*f_off*t; y=u.*exp(1i*(lp+inc)); lp=mod(lp+inc(end),2*pi);\n']);
configObj = sfroot; chart = configObj.find('-isa', 'Stateflow.EMChart', 'Path', dopPath);
if ~isempty(chart), chart.Script = dopScript; end

% 信道封装
ntnPath = [subSys '/NTN_Wrapper'];
add_block('simulink/User-Defined Functions/MATLAB Function', ntnPath, 'Position', [350, 40, 450, 100]);
ntnScript = sprintf([...
    'function out = fcn(in)\n persistent h\n coder.varsize(''out'',[1000 1],[0 0]);\n',...
    'if isempty(h), h=nrTDLChannel(); h.DelayProfile=''Custom''; h.PathDelays=[0 1e-7 2e-7];\n',...
    'h.AveragePathGains=[0 -4 -6]; h.NumTransmitAntennas=1; h.NumReceiveAntennas=1; h.SampleRate=15.36e6; end\n',...
    '[out,~]=h(in);\n']);
configObj = sfroot; chart = configObj.find('-isa', 'Stateflow.EMChart', 'Path', ntnPath);
if ~isempty(chart), chart.Script = ntnScript; end

% 路损
add_block('simulink/Math Operations/Gain', [subSys '/Neg'], 'Position', [150, 250, 180, 280], 'Gain', '-0.1');
add_block('simulink/Math Operations/Math Function', [subSys '/dB2P'], 'Position', [250, 250, 280, 280], 'Operator', '10^u');
add_block('simulink/Math Operations/Product', [subSys '/Prod'], 'Position', [650, 50, 680, 80]);

add_line(subSys, 'Tx/1', 'Doppler_Math/1');
add_line(subSys, 'Doppler/1', 'Doppler_Math/2');
add_line(subSys, 'Doppler_Math/1', 'NTN_Wrapper/1');
add_line(subSys, 'PathLoss/1', 'Neg/1');
add_line(subSys, 'Neg/1', 'dB2P/1');
add_line(subSys, 'NTN_Wrapper/1', 'Prod/1');
add_line(subSys, 'dB2P/1', 'Prod/2');
add_line(subSys, 'Prod/1', 'Rx/1');

% -------------------------------------------------------------------------
% 2.4 干扰源
% -------------------------------------------------------------------------
add_block('simulink/Math Operations/Add', [modelName '/Add_Interference'], 'Position', [650, 90, 680, 120]);
jamPath = [modelName '/CW_Jammer'];
add_block('simulink/User-Defined Functions/MATLAB Function', jamPath, 'Position', [480, 140, 540, 180]);
jamScript = sprintf([...
    'function y = fcn()\n persistent p\n coder.varsize(''y'',[1000 1],[0 0]);\n',...
    'if isempty(p), p=0; end\n Fs=15.36e6; t=(0:999).''/Fs; y=complex(zeros(1000,1));\n',...
    'y(:)=0.1*exp(1i*(p+2*pi*100*t)); p=mod(p+2*pi*100*(1000/Fs),2*pi);\n']);
configObj = sfroot; chart = configObj.find('-isa', 'Stateflow.EMChart', 'Path', jamPath);
if ~isempty(chart), chart.Script = jamScript; end
add_block('simulink/Signal Attributes/Signal Specification', [modelName '/Jam_Spec'],...
    'Position', [580, 150, 620, 170], 'Dimensions', '[1000 1]', 'SignalType', 'complex');

% -------------------------------------------------------------------------
% 2.5 显示模块 (Sink) - 自动保底
% -------------------------------------------------------------------------
sinkBlockName = [modelName '/Visual_Sink']; 
sinkPos = [750, 80, 800, 130];
sinkType = '';

try
    % 1. 先加 Scope (绝对保底)
    add_block('simulink/Sinks/Scope', sinkBlockName, 'Position', sinkPos);
    sinkType = 'Scope (示波器)';
    
    % 2. 尝试替换为 Spectrum Analyzer
    try
        delete_block(sinkBlockName); % 先删掉 Scope
        add_block('dspviewers/Spectrum Analyzer', sinkBlockName, 'Position', sinkPos);
        sinkType = 'Spectrum Analyzer (DSP)';
    catch
        % 如果失败，加回 Scope
        add_block('simulink/Sinks/Scope', sinkBlockName, 'Position', sinkPos);
        sinkType = 'Scope (示波器 - 无DSP工具箱)';
    end
catch
    error('严重错误：连基础 Scope 模块都无法添加！');
end
fprintf(' -> 已添加显示模块: %s\n', sinkType);

% -------------------------------------------------------------------------
% 2.6 顶层连线
% -------------------------------------------------------------------------
add_line(modelName, 'Tx_Gen/1', 'Tx_Spec/1');
add_line(modelName, 'Tx_Spec/1', 'Channel_Model/1');
add_line(modelName, 'Sim_Clock/1', 'Doppler_Table/1');
add_line(modelName, 'Doppler_Table/1', 'Channel_Model/2');
add_line(modelName, 'Sim_Clock/1', 'PathLoss_Table/1');
add_line(modelName, 'PathLoss_Table/1', 'Channel_Model/3');
add_line(modelName, 'Channel_Model/1', 'Add_Interference/1');
add_line(modelName, 'CW_Jammer/1', 'Jam_Spec/1');
add_line(modelName, 'Jam_Spec/1', 'Add_Interference/2');
add_line(modelName, 'Add_Interference/1', 'Visual_Sink/1');

% =========================================================================
% [Step 3] 完成 - 等待用户手动运行
% =========================================================================
save_system(modelName);
disp('[Success] 模型构建完成。');
disp('--------------------------------------------------');
disp(' >> 请在 Simulink 窗口上方点击绿色的 "Run" 按钮开始仿真。');
disp('--------------------------------------------------');