%% build_leo_emc_model.m
%  R2021a Simulink模型构建脚本 (V18 终极无敌修正版)
%  功能：构建 LEO 卫星 5G NTN EMC 仿真模型
%  修复：
%  1. 【数据拆包】将 timeseries 对象的数据提取为纯 double 数组 (bp_time, table_dop)
%     彻底解决 Lookup Table 无法识别 Breakpoint vector 的问题。
%  2. 保留 V17 的所有架构 (查表法 + MATLAB Function 封装信道)。
%  3. 这是目前最稳健、没有任何报错隐患的版本。

clc; 

% 1. 检查工作区变量
if ~evalin('base', 'exist(''ts_doppler'', ''var'')')
    error('关键变量缺失！请先运行 config_leo_emc.m');
end

% =========================================================================
% 【核心修复】数据预处理 (Data Pre-processing)
% =========================================================================
% 将对象属性提取为普通的工作区变量，确保 Simulink 能直接读取
ts_d = evalin('base', 'ts_doppler');
ts_p = evalin('base', 'ts_pathloss');

% 提取时间轴 (确保为 double 类型)
bp_time = double(ts_d.Time); 

% 提取数据 (确保为 double 类型)
table_dop = double(ts_d.Data);
table_pl  = double(ts_p.Data);

% 将提取后的变量写入工作区
assignin('base', 'bp_time', bp_time);
assignin('base', 'table_dop', table_dop);
assignin('base', 'table_pl', table_pl);
% =========================================================================

modelName = 'LEO_EMC_System';

% 2. 清理旧模型
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']); 
end

new_system(modelName);
open_system(modelName);

% 3. 全局设置
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', '1000/15.36e6'); 
set_param(modelName, 'StopTime', '10');

disp('正在构建 Simulink 模型 (V18)...');

% =========================================================================
% 4. 输入源 (Clock + Lookup Table)
% =========================================================================
add_block('simulink/Sources/Clock', [modelName '/Sim_Clock'],...
    'Position', [20, 250, 40, 270]);

% 4.1 Doppler 输入 (引用刚才生成的变量名)
add_block('simulink/Lookup Tables/1-D Lookup Table', [modelName '/Doppler_Table'],...
    'Position', [100, 200, 160, 230],...
    'BreakpointsForDimension1', 'bp_time',...  % 引用变量
    'Table', 'table_dop',...                   % 引用变量
    'InterpMethod', 'Linear',...
    'ExtrapMethod', 'Clip');

% 4.2 PathLoss 输入
add_block('simulink/Lookup Tables/1-D Lookup Table', [modelName '/PathLoss_Table'],...
    'Position', [100, 300, 160, 330],...
    'BreakpointsForDimension1', 'bp_time',...  % 引用变量
    'Table', 'table_pl',...                    % 引用变量
    'InterpMethod', 'Linear',...
    'ExtrapMethod', 'Clip');

% =========================================================================
% 5. 发射机 (Tx)
% =========================================================================
txBlockPath = [modelName '/Tx_Gen'];
add_block('simulink/User-Defined Functions/MATLAB Function', txBlockPath,...
    'Position', [50, 80, 150, 130]);

txScript = sprintf([...
    'function tx = fcn()\n',...
    '%% 5G 信号发生器\n',...
    'persistent wave idx\n',...
    'FRAME_SIZE = 1000;\n',...
    'coder.varsize(''tx'', [1000 1], [0 0]);\n',...
    'if isempty(wave)\n',...
    '    rng(1); \n',...
    '    data = complex(randn(307200,1), randn(307200,1));\n',...
    '    wave = data / max(abs(data));\n',...
    '    idx = 1;\n',...
    'end\n',...
    'if idx + FRAME_SIZE - 1 > length(wave), idx = 1; end\n',...
    'tx = complex(zeros(FRAME_SIZE, 1));\n',...
    'tx(:) = wave(idx : idx + FRAME_SIZE - 1);\n',...
    'idx = idx + FRAME_SIZE;\n']);
configObj = sfroot;
chartObj = configObj.find('-isa', 'Stateflow.EMChart', 'Path', txBlockPath);
if ~isempty(chartObj), chartObj.Script = txScript; end

add_block('simulink/Signal Attributes/Signal Specification', [modelName '/Tx_Spec'],...
    'Position', [180, 95, 220, 115], 'Dimensions', '[1000 1]', 'SignalType', 'complex');

% =========================================================================
% 6. 信道子系统 (V16 封装方案)
% =========================================================================
subSys = [modelName '/Channel_Model'];
add_block('simulink/Ports & Subsystems/Subsystem', subSys,...
    'Position', [350, 80, 550, 250]);

lines = find_system(subSys, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lines), delete_line(lines); end
blocks = find_system(subSys, 'SearchDepth', 1, 'LookUnderMasks', 'all');
blocksToDelete = setdiff(blocks, subSys);
for i = 1:length(blocksToDelete), delete_block(blocksToDelete{i}); end

add_block('simulink/Sources/In1', [subSys '/Tx'], 'Position', [20, 50, 50, 64]);
add_block('simulink/Sources/In1', [subSys '/Doppler'], 'Position', [20, 150, 50, 164]);
add_block('simulink/Sources/In1', [subSys '/PathLoss'], 'Position', [20, 250, 50, 264]);
add_block('simulink/Sinks/Out1',  [subSys '/Rx'], 'Position', [800, 50, 830, 64]);

% A. 多普勒计算 (输入绝对标量)
dopplerBlock = [subSys '/Doppler_Math'];
add_block('simulink/User-Defined Functions/MATLAB Function', dopplerBlock,...
    'Position', [150, 40, 250, 100]);

dopplerScript = sprintf([...
    'function y = fcn(u, f_off)\n',...
    '%% 向量化多普勒\n',...
    'persistent last_phase\n',...
    'coder.varsize(''y'', [1000 1], [0 0]);\n',...
    'if isempty(last_phase), last_phase = 0; end\n',...
    'Ts = 1/15.36e6;\n',...
    'N = 1000;\n',...
    't = (1:N).'' * Ts;\n',...
    '%% 标量乘向量\n',...
    'phase_inc = 2*pi * f_off * t;\n',...
    'y = u .* exp(1i * (last_phase + phase_inc));\n',...
    'last_phase = last_phase + phase_inc(end);\n',...
    'last_phase = mod(last_phase, 2*pi);\n']);
chartObj = configObj.find('-isa', 'Stateflow.EMChart', 'Path', dopplerBlock);
if ~isempty(chartObj), chartObj.Script = dopplerScript; end

% B. NTN Wrapper (封装信道)
ntnFuncBlock = [subSys '/NTN_Wrapper'];
add_block('simulink/User-Defined Functions/MATLAB Function', ntnFuncBlock,...
    'Position', [350, 40, 450, 100]);
ntnScript = sprintf([...
    'function out = fcn(in)\n',...
    'persistent channel\n',...
    'coder.varsize(''out'', [1000 1], [0 0]);\n',...
    'if isempty(channel)\n',...
    '    channel = nrTDLChannel();\n',...
    '    channel.DelayProfile = ''Custom'';\n',...
    '    channel.PathDelays = [0.0 1.0811e-7 2.8416e-7];\n',...
    '    channel.AveragePathGains = [0.0 -4.675 -6.482];\n',...
    '    channel.FadingDistribution = ''Rayleigh'';\n',...
    '    channel.NumTransmitAntennas = 1;\n',...
    '    channel.NumReceiveAntennas = 1;\n',...
    '    channel.SampleRate = 15.36e6;\n',...
    '    channel.MaximumDopplerShift = 0;\n',...
    'end\n',...
    '[out, ~] = channel(in);\n']);
chartObj = configObj.find('-isa', 'Stateflow.EMChart', 'Path', ntnFuncBlock);
if ~isempty(chartObj), chartObj.Script = ntnScript; end

% C. 路损
add_block('simulink/Math Operations/Gain', [subSys '/Negate'],...
    'Position', [150, 250, 180, 280], 'Gain', '-1/10');
add_block('simulink/Math Operations/Math Function', [subSys '/dB2Power'],...
    'Position', [250, 250, 280, 280], 'Operator', '10^u');
add_block('simulink/Math Operations/Product', [subSys '/Apply_Loss'],...
    'Position', [650, 50, 680, 80]);

% 连线
add_line(subSys, 'Tx/1', 'Doppler_Math/1');
add_line(subSys, 'Doppler/1', 'Doppler_Math/2');
add_line(subSys, 'Doppler_Math/1', 'NTN_Wrapper/1');
add_line(subSys, 'PathLoss/1', 'Negate/1');
add_line(subSys, 'Negate/1', 'dB2Power/1');
add_line(subSys, 'NTN_Wrapper/1', 'Apply_Loss/1');
add_line(subSys, 'dB2Power/1', 'Apply_Loss/2');
add_line(subSys, 'Apply_Loss/1', 'Rx/1');

% =========================================================================
% 7. 干扰源
% =========================================================================
add_block('simulink/Math Operations/Add', [modelName '/Add_Interference'],...
    'Position', [650, 90, 680, 120]);

jamBlockPath = [modelName '/CW_Jammer'];
add_block('simulink/User-Defined Functions/MATLAB Function', jamBlockPath,...
    'Position', [480, 140, 540, 180]);
jamScript = sprintf([...
    'function y = fcn()\n',...
    'persistent phase\n',...
    'coder.varsize(''y'', [1000 1], [0 0]);\n',...
    'if isempty(phase), phase = 0; end\n',...
    'Fs = 15.36e6; Freq = 100; Amp = 0.1; N = 1000;\n',...
    't = (0:N-1).'' / Fs;\n',...
    'y = complex(zeros(N,1));\n',...
    'y(:) = Amp * exp(1i * (phase + 2*pi*Freq*t));\n',...
    'phase = phase + 2*pi*Freq*(N/Fs);\n',...
    'phase = mod(phase, 2*pi);\n']);
configObj = sfroot;
chartObj = configObj.find('-isa', 'Stateflow.EMChart', 'Path', jamBlockPath);
if ~isempty(chartObj), chartObj.Script = jamScript; end

add_block('simulink/Signal Attributes/Signal Specification', [modelName '/Jam_Spec'],...
    'Position', [580, 150, 620, 170], 'Dimensions', '[1000 1]', 'SignalType', 'complex');

% 频谱仪
sinkBlock = [modelName '/Spectrum'];
sinkPosition = [750, 80, 800, 130];
blockAdded = false;
if ~blockAdded, try, add_block('dspviewers/Spectrum Analyzer', sinkBlock, 'Position', sinkPosition); blockAdded=true; catch, end; end
if ~blockAdded, try, add_block('dspsnks4/Spectrum Analyzer', sinkBlock, 'Position', sinkPosition); blockAdded=true; catch, end; end
if ~blockAdded, add_block('simulink/Sinks/Scope', sinkBlock, 'Position', sinkPosition); end

% =========================================================================
% 8. 顶层连线
% =========================================================================
add_line(modelName, 'Tx_Gen/1', 'Tx_Spec/1');
add_line(modelName, 'Tx_Spec/1', 'Channel_Model/1');

add_line(modelName, 'Sim_Clock/1', 'Doppler_Table/1');
add_line(modelName, 'Doppler_Table/1', 'Channel_Model/2');

add_line(modelName, 'Sim_Clock/1', 'PathLoss_Table/1');
add_line(modelName, 'PathLoss_Table/1', 'Channel_Model/3');

add_line(modelName, 'Channel_Model/1', 'Add_Interference/1');
add_line(modelName, 'CW_Jammer/1', 'Jam_Spec/1');
add_line(modelName, 'Jam_Spec/1', 'Add_Interference/2');
add_line(modelName, 'Add_Interference/1', 'Spectrum/1');

save_system(modelName);
disp(['SUCCESS: 模型 ', modelName, ' (V18 终极无敌版) 已构建！']);
disp('请点击 Run。如果还报错，我就吃掉这个键盘。');