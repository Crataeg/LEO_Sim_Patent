function build_LEO_EMC_Sim()
% build_LEO_EMC_Sim
% 一键生成 Simulink 模型：LEO卫星通信 EMC（通信视角）仿真骨架
%
% 依赖（推荐）：
%  - Simulink
%  - Communications Toolbox（用于QPSK调制/解调、卷积码/Viterbi、误码统计等模块）
%
% 生成：
%  - LEO_EMC_Sim_Simulink.slx

model = 'LEO_EMC_Sim_Simulink';
if bdIsLoaded(model); close_system(model,0); end
if exist([model '.slx'],'file'); delete([model '.slx']); end

new_system(model);
open_system(model);

% ---------- 全局参数（写入 Base Workspace，方便外行只改这里） ----------
cfg = defaultCfg();
assignin('base','cfg',cfg);

% ---------- 画布基础设置 ----------
set_param(model,'StopTime','cfg.sim.StopTime');
set_param(model,'Solver','FixedStepDiscrete');
set_param(model,'FixedStep','cfg.sim.Ts');

% ---------- 添加子系统：TX / Channel+EMC / RX / Metrics ----------
add_block('built-in/Subsystem',[model '/TX'], 'Position',[80 80 240 240]);
add_block('built-in/Subsystem',[model '/Channel_EMC'], 'Position',[320 80 520 240]);
add_block('built-in/Subsystem',[model '/RX'], 'Position',[600 80 760 240]);
add_block('built-in/Subsystem',[model '/Metrics'], 'Position',[840 80 1020 240]);

% 顶层连接：TX -> Channel_EMC -> RX -> Metrics

% ---------- 构建 TX ----------
buildTX([model '/TX']);

% ---------- 构建 Channel + EMC ----------
buildChannelEMC([model '/Channel_EMC']);

% ---------- 构建 RX ----------
buildRX([model '/RX']);

% ---------- 构建 Metrics ----------
buildMetrics([model '/Metrics']);
add_line(model,'TX/1','Channel_EMC/1','autorouting','on');
add_line(model,'Channel_EMC/1','RX/1','autorouting','on');
add_line(model,'RX/1','Metrics/1','autorouting','on');

% ---------- 美化布局 ----------

save_system(model);
disp(['✅ 已生成模型：' model '.slx']);
disp('下一步：运行 run_LEO_EMC_Sim.m 来仿真并出图。');
end

% =================================================================
%                       默认参数（外行只改这里）
% =================================================================
function cfg = defaultCfg()
cfg.sim.Ts = 1/1e6;                 % 离散步长(秒) = 1/Rs
cfg.sim.StopTime = 0.2;           % 仿真时长（秒）

cfg.phy.Rs = 1e6;                   % 符号率(Hz)
cfg.phy.frameBits = 1000;           % 每帧信息比特数
cfg.phy.preambleHalfLen = 8;        % 重复前导半段长度 L
cfg.phy.preambleReps = 4;           % 重复次数（>=2）

cfg.chan.enableRician = true;
cfg.chan.K_dB = 6;                  % Rician K(dB)

% LEO Doppler（可在 run脚本里改成随时间变化的序列/函数）
cfg.leo.fD_Hz = 30e3;               % 多普勒频偏(Hz)（简化为常数）

% 噪声/干扰（通信视角 EMC）
cfg.noise.EbN0_dB = 8;              % 热噪声强度（粗略控制）
cfg.emc.type = 1;                   % 1=同频噪声 2=单音 3=脉冲 4=邻频等效 5=同址等效
cfg.emc.JS_dB = -5;                 % 干扰/信号 J/S(dB)
cfg.emc.toneOffset_Hz = 10e3;       % 单音干扰频偏（Hz）
cfg.emc.pulseDuty = 0.10;           % 脉冲占空比
cfg.emc.pulsePeriodSym = 200;       % 脉冲周期(符号)
cfg.emc.ACIR_dB = 30;               % 邻频等效（越大越好）
cfg.emc.cositeCoupling_dB = -60;    % 同址耦合(dB)
cfg.emc.cositeSrcP_dBW = 0;         % 同址源功率(dBW)

% CFO补偿
cfg.rx.cfoMethod = 2;               % 1=理想 2=前导估计
end

% =================================================================
%                           TX 子系统
% 输出：复基带符号流（含前导）
% =================================================================
function buildTX(path)
% 端口
add_block('built-in/Outport',[path '/Out1'],'Position',[380 120 410 140]);
cfg = evalin('base','cfg');
sigLen = cfg.phy.frameBits + cfg.phy.preambleHalfLen * cfg.phy.preambleReps;

% 1) 随机比特
add_block(['commrandsrc3/Random Integer' newline 'Generator'],[path '/RandBits'], ...
    'Position',[60 90 170 140], ...
    'SetSize','2', 'SampleTime','cfg.sim.Ts', 'SamplesPerFrame','cfg.phy.frameBits');

% 2) 卷积编码（需要 Comm Toolbox）
add_block(['commcnvcod2/Convolutional' newline 'Encoder'],[path '/ConvEnc'], ...
    'Position',[200 90 280 140], ...
    'trellis','poly2trellis(7,[171 133])');

% 3) QPSK 调制（Rectangular QAM with M=4）
add_block(['commdigbbndam3/Rectangular QAM' newline 'Modulator' newline 'Baseband'],[path '/QPSKMod'], ...
    'Position',[310 90 360 140], ...
    'M','4', 'InType','Bit', 'Enc','Gray', 'PowType','Average Power');

% 4) 前导插入（MATLAB Function Block：把重复前导 + payload 拼接）
add_block(['simulink/User-Defined' newline 'Functions/Interpreted MATLAB' newline 'Function'],[path '/PreambleInsert'], ...
    'Position',[310 160 360 220]);
set_param([path '/PreambleInsert'], 'MATLABFcn','preamble_insert', ...
    'OutputDimensions', num2str(sigLen));

% 连接：RandBits->ConvEnc->QPSKMod->PreambleInsert->Out
add_line(path,'RandBits/1','ConvEnc/1','autorouting','on');
add_line(path,'ConvEnc/1','QPSKMod/1','autorouting','on');
add_line(path,'QPSKMod/1','PreambleInsert/1','autorouting','on');
add_line(path,'PreambleInsert/1','Out1/1','autorouting','on');

% 写入 MATLAB Function 代码
end

% =================================================================
%                   Channel + EMC 子系统
% 输入：tx复基带；输出：rx复基带（含噪声+干扰+多普勒+衰落）
% =================================================================
function buildChannelEMC(path)
add_block('built-in/Inport',[path '/In1'], 'Position',[20 120 50 140]);
add_block('built-in/Outport',[path '/Out1'],'Position',[500 120 530 140]);
cfg = evalin('base','cfg');
sigLen = cfg.phy.frameBits + cfg.phy.preambleHalfLen * cfg.phy.preambleReps;

% 1) Rician 平坦衰落（Comm Toolbox）
add_block('commchan3/SISO Fading Channel',[path '/Rician'], ...
    'Position',[80 90 180 150], ...
    'FadingDistribution','Rician', ...
    'KFactor','10^(cfg.chan.K_dB/10)', ...
    'PathDelays','0', ...
    'AveragePathGains','0', ...
    'DopplerSpectrum','doppler(''Jakes'')', ...
    'MaximumDopplerShift','0'); % 我们把多普勒放到 Frequency Offset 模块

% 2) 多普勒：Frequency Offset
add_block(['commrflib2/Phase//' newline 'Frequency' newline 'Offset'],[path '/FreqOffset'], ...
    'Position',[210 90 320 150], ...
    'freqOffset','cfg.leo.fD_Hz');

% 3) 干扰源生成（MATLAB Function：根据cfg.emc.type生成干扰波形）
add_block(['simulink/User-Defined' newline 'Functions/Interpreted MATLAB' newline 'Function'],[path '/InterferenceGen'], ...
    'Position',[210 170 320 240]);
set_param([path '/InterferenceGen'], 'MATLABFcn','interf_gen', ...
    'OutputDimensions', num2str(sigLen));

% 4) 加法器：信号 + 干扰
add_block('simulink/Math Operations/Add',[path '/AddSigInt'], ...
    'Position',[350 110 380 140], ...
    'Inputs','++');

% 5) 噪声：AWGN Channel（Comm Toolbox）
add_block(['commchan3/AWGN' newline 'Channel'],[path '/AWGN'], ...
    'Position',[400 90 470 150], ...
    'EbNodB','cfg.noise.EbN0_dB', ...
    'bitsPerSym','2', ...
    'Tsym','cfg.sim.Ts');

% 连接
add_line(path,'In1/1','Rician/1','autorouting','on');
add_line(path,'Rician/1','FreqOffset/1','autorouting','on');
add_line(path,'FreqOffset/1','AddSigInt/1','autorouting','on');
add_line(path,'FreqOffset/1','InterferenceGen/1','autorouting','on');
add_line(path,'InterferenceGen/1','AddSigInt/2','autorouting','on');
add_line(path,'AddSigInt/1','AWGN/1','autorouting','on');
add_line(path,'AWGN/1','Out1/1','autorouting','on');

% 干扰生成 MATLAB Function 代码
end

% =================================================================
%                           RX 子系统
% 输入：rx复基带（含前导）；输出：译码后的信息比特
% =================================================================
function buildRX(path)
add_block('built-in/Inport',[path '/In1'], 'Position',[20 120 50 140]);
add_block('built-in/Outport',[path '/Out1'],'Position',[520 120 550 140]);
cfg = evalin('base','cfg');
sigLen = cfg.phy.frameBits + cfg.phy.preambleHalfLen * cfg.phy.preambleReps;

% 1) CFO估计与补偿（MATLAB Function）
add_block(['simulink/User-Defined' newline 'Functions/Interpreted MATLAB' newline 'Function'],[path '/CFOComp'], ...
    'Position',[90 80 220 160]);
set_param([path '/CFOComp'], 'MATLABFcn','cfo_comp', ...
    'OutputDimensions', num2str(sigLen));

% 2) QPSK 解调（Comm Toolbox）
add_block(['commdigbbndam3/Rectangular QAM' newline 'Demodulator' newline 'Baseband'],[path '/QPSKDemod'], ...
    'Position',[260 80 340 160], ...
    'M','4', 'OutType','Bit', 'DecType','Hard decision', 'Dec','Gray', 'PowType','Average Power');

% 3) Viterbi译码（Comm Toolbox）
add_block('commcnvcod2/Viterbi Decoder',[path '/Viterbi'], ...
    'Position',[380 80 480 160], ...
    'trellis','poly2trellis(7,[171 133])', ...
    'dectype','Hard decision', ...
    'tbdepth','42', ...
    'opmode','Terminated');

% 连接
add_line(path,'In1/1','CFOComp/1','autorouting','on');
add_line(path,'CFOComp/1','QPSKDemod/1','autorouting','on');
add_line(path,'QPSKDemod/1','Viterbi/1','autorouting','on');
add_line(path,'Viterbi/1','Out1/1','autorouting','on');

% CFO补偿 MATLAB Function（重复前导估计或理想补偿）
end

% =================================================================
%                       Metrics 子系统
% 输入：译码后的信息比特；输出：误码率到工作区
% =================================================================
function buildMetrics(path)
add_block('built-in/Inport',[path '/In1'], 'Position',[20 120 50 140]);

% 参考比特：为了能算BER，这里再生成同样长度的随机源（演示用）
% 更严谨做法：把TX的原始比特也输出到Metrics做对齐比较
add_block(['commrandsrc3/Random Integer' newline 'Generator'],[path '/RefBits'], ...
    'Position',[80 70 180 120], ...
    'SetSize','2','SampleTime','cfg.sim.Ts * (cfg.phy.frameBits + cfg.phy.preambleHalfLen * cfg.phy.preambleReps)','SamplesPerFrame','cfg.phy.frameBits + cfg.phy.preambleHalfLen * cfg.phy.preambleReps');

add_block('simulink/Sinks/To Workspace',[path '/RefBitsWS'], ...
    'Position',[400 60 480 100], ...
    'VariableName','refBits', 'SaveFormat','Array');

add_block('simulink/Sinks/To Workspace',[path '/RxBitsWS'], ...
    'Position',[400 130 480 170], ...
    'VariableName','rxBits', 'SaveFormat','Array');

add_line(path,'RefBits/1','RefBitsWS/1','autorouting','on');
add_line(path,'In1/1','RxBitsWS/1','autorouting','on');

% 备注：这个Metrics是“演示型”。如果你要完全严谨：
% - 在TX里把原始信息比特 u 也输出一个端口
% - 顶层连接到Metrics用于真实对比
end
