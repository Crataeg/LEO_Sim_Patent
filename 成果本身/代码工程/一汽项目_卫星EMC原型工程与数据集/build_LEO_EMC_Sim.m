function build_LEO_EMC_Sim()
% LEO EMC 通信视角 Simulink 自动建模脚本（Comm Toolbox + Stateflow 写 Script）
% R2021a兼容：MATLAB Function block 在Simulink层面可能表现为SubSystem；需通过 Stateflow.EMChart.Script 写入代码

assert(license('test','communication_toolbox')==1, "Communications Toolbox 未授权");
assert(license('test','stateflow')==1, "Stateflow 未授权（MATLAB Function block 需要）");

model = 'LEO_EMC_Sim_Simulink';
if bdIsLoaded(model); close_system(model,0); end
if exist([model '.slx'],'file'); delete([model '.slx']); end

new_system(model);
open_system(model);

cfg = defaultCfg();
assignin('base','cfg',cfg);

set_param(model,'Solver','FixedStepDiscrete');
set_param(model,'FixedStep', num2str(cfg.sim.Ts));
set_param(model,'StopTime',  cfg.sim.StopTime);

% 顶层子系统
add_block('built-in/Subsystem',[model '/TX'],          'Position',[80 80 260 260]);
add_block('built-in/Subsystem',[model '/Channel_EMC'], 'Position',[320 80 540 260]);
add_block('built-in/Subsystem',[model '/RX'],          'Position',[600 80 820 260]);
add_block('built-in/Subsystem',[model '/Metrics'],     'Position',[880 80 1120 260]);

buildTX([model '/TX']);
buildChannelEMC([model '/Channel_EMC']);
buildRX([model '/RX']);
buildMetrics([model '/Metrics']);

set_param(model,'SimulationCommand','update');

% 顶层连线
add_line(model,'TX/1','Channel_EMC/1','autorouting','on');
add_line(model,'Channel_EMC/1','RX/1','autorouting','on');
add_line(model,'RX/1','Metrics/1','autorouting','on');
add_line(model,'TX/2','Metrics/2','autorouting','on');

save_system(model);
fprintf("✅ 已生成模型：%s.slx\n", model);
fprintf("可选：运行 run_LEO_EMC_Sim 进行仿真并输出 BER\n");
end

% ======================== 参数 ========================
function cfg = defaultCfg()
cfg.sim.Ts = 1/1e6;
cfg.sim.StopTime = '0.2';

cfg.phy.Rs = 1e6;
cfg.phy.frameBits = 1000;
cfg.phy.preambleHalfLen = 8;
cfg.phy.preambleReps = 4;

cfg.chan.K_dB = 6;
cfg.leo.fD_Hz = 30e3;

cfg.noise.EbN0_dB = 8;

cfg.emc.type = 1;  % 1同频噪声 2单音 3脉冲 4邻频等效 5同址等效
cfg.emc.JS_dB = -5;
cfg.emc.toneOffset_Hz = 10e3;
cfg.emc.pulseDuty = 0.10;
cfg.emc.pulsePeriodSym = 200;
cfg.emc.ACIR_dB = 30;
cfg.emc.cositeCoupling_dB = -60;
cfg.emc.cositeSrcP_dBW = 0;

cfg.rx.cfoMethod = 2; % 1理想 2前导估计
end

% ======================== TX ========================
function buildTX(path)
Simulink.SubSystem.deleteContents(path);

add_block('built-in/Outport',[path '/OutWave'],'Position',[520 130 550 150]);
add_block('built-in/Outport',[path '/OutBits'],'Position',[520 60 550 80]);

% Rand bits（MATLAB Function block）
addMatlabFcnViaChart([path '/RandBits'], [40 90 180 150], randBitsCode());

% 卷积编码器
convBlk = resolveCommBlock({'Convolutional Encoder'});
add_block(convBlk,[path '/ConvEnc'], 'Position',[220 90 320 150]);
try set_param([path '/ConvEnc'],'TrellisStructure','poly2trellis(7,[171 133])'); catch, end

% QPSK调制（4-QAM）
modBlk = resolveCommBlock({'Rectangular QAM Modulator Baseband','QAM Modulator Baseband','QPSK Modulator Baseband'});
add_block(modBlk,[path '/QPSKMod'], 'Position',[360 90 450 150]);
try set_param([path '/QPSKMod'],'ModulationOrder','4'); catch, end
try set_param([path '/QPSKMod'],'BitInput','on'); catch, end
try set_param([path '/QPSKMod'],'NormalizationMethod','Average power'); catch, end

% 前导插入
addMatlabFcnViaChart([path '/PreambleInsert'], [360 170 450 235], preambleInsertCode());

% 连接
add_line(path,'RandBits/1','OutBits/1','autorouting','on');
add_line(path,'RandBits/1','ConvEnc/1','autorouting','on');
add_line(path,'ConvEnc/1','QPSKMod/1','autorouting','on');
add_line(path,'QPSKMod/1','PreambleInsert/1','autorouting','on');
add_line(path,'PreambleInsert/1','OutWave/1','autorouting','on');
end

% ==================== Channel + EMC ====================
function buildChannelEMC(path)
Simulink.SubSystem.deleteContents(path);
add_block('built-in/Inport',[path '/In1'], 'Position',[20 120 50 140]);
add_block('built-in/Outport',[path '/Out1'],'Position',[600 120 630 140]);

% Rician
ricBlk = resolveCommBlock({'Rician Fading Channel','Rician Channel'});
add_block(ricBlk,[path '/Rician'], 'Position',[80 90 200 150]);
try set_param([path '/Rician'],'KFactor','10^(cfg.chan.K_dB/10)'); catch, end
try set_param([path '/Rician'],'PathDelays','0'); catch, end
try set_param([path '/Rician'],'AveragePathGains','0'); catch, end
try set_param([path '/Rician'],'MaximumDopplerShift','0'); catch, end

% Frequency Offset
foBlk = resolveCommBlock({'Frequency Offset','Phase/Frequency Offset','Frequency Offset (Baseband)'});
add_block(foBlk,[path '/FreqOffset'], 'Position',[230 90 360 150]);
try set_param([path '/FreqOffset'],'FrequencyOffset','cfg.leo.fD_Hz'); catch, end
try set_param([path '/FreqOffset'],'SampleRate','cfg.phy.Rs'); catch, end

% 干扰生成（输入=信号，用于算参考功率）
addMatlabFcnViaChart([path '/InterferenceGen'], [230 175 360 235], interfGenCode());

add_block('simulink/Math Operations/Add',[path '/AddSigInt'], ...
    'Position',[390 110 420 140], 'Inputs','++');

% AWGN
awgnBlk = resolveCommBlock({'AWGN Channel','AWGN'});
add_block(awgnBlk,[path '/AWGN'], 'Position',[450 90 580 150]);
try set_param([path '/AWGN'],'EbNo','cfg.noise.EbN0_dB'); catch, end
try set_param([path '/AWGN'],'BitsPerSymbol','2'); catch, end
try set_param([path '/AWGN'],'SamplesPerSymbol','1'); catch, end
try set_param([path '/AWGN'],'SNR','cfg.noise.EbN0_dB'); catch, end

% 连接
add_line(path,'In1/1','Rician/1','autorouting','on');
add_line(path,'Rician/1','FreqOffset/1','autorouting','on');
add_line(path,'FreqOffset/1','InterferenceGen/1','autorouting','on');
add_line(path,'FreqOffset/1','AddSigInt/1','autorouting','on');
add_line(path,'InterferenceGen/1','AddSigInt/2','autorouting','on');
add_line(path,'AddSigInt/1','AWGN/1','autorouting','on');
add_line(path,'AWGN/1','Out1/1','autorouting','on');
end

% ========================= RX =========================
function buildRX(path)
Simulink.SubSystem.deleteContents(path);
add_block('built-in/Inport',[path '/In1'], 'Position',[20 120 50 140]);
add_block('built-in/Outport',[path '/Out1'],'Position',[700 120 730 140]);

% CFO 补偿
addMatlabFcnViaChart([path '/CFOComp'], [90 80 220 160], cfoCompCode());

% QPSK解调（4-QAM）
demBlk = resolveCommBlock({'Rectangular QAM Demodulator Baseband','QAM Demodulator Baseband','QPSK Demodulator Baseband'});
add_block(demBlk,[path '/QPSKDemod'], 'Position',[260 80 380 160]);
try set_param([path '/QPSKDemod'],'ModulationOrder','4'); catch, end
try set_param([path '/QPSKDemod'],'BitOutput','on'); catch, end
try set_param([path '/QPSKDemod'],'NormalizationMethod','Average power'); catch, end

% Viterbi
vitBlk = resolveCommBlock({'Viterbi Decoder'});
add_block(vitBlk,[path '/Viterbi'], 'Position',[420 80 560 160]);
try set_param([path '/Viterbi'],'TrellisStructure','poly2trellis(7,[171 133])'); catch, end
try set_param([path '/Viterbi'],'InputFormat','Hard'); catch, end
try set_param([path '/Viterbi'],'TracebackDepth','42'); catch, end
try set_param([path '/Viterbi'],'TerminationMethod','Terminated'); catch, end

add_line(path,'In1/1','CFOComp/1','autorouting','on');
add_line(path,'CFOComp/1','QPSKDemod/1','autorouting','on');
add_line(path,'QPSKDemod/1','Viterbi/1','autorouting','on');
add_line(path,'Viterbi/1','Out1/1','autorouting','on');
end

% ======================= Metrics =======================
function buildMetrics(path)
Simulink.SubSystem.deleteContents(path);
add_block('built-in/Inport',[path '/RxBits'], 'Position',[20 120 50 140]); % In1
add_block('built-in/Inport',[path '/TxBits'], 'Position',[20 60 50 80]);  % In2

errBlk = resolveCommBlock({'Error Rate Calculation','Error Rate'});
add_block(errBlk,[path '/ErrRate'], 'Position',[120 85 270 145]);

add_block('simulink/Sinks/To Workspace',[path '/ToWS'], ...
    'Position',[310 95 420 135], 'VariableName','errRate', 'SaveFormat','Array');

add_line(path,'TxBits/1','ErrRate/1','autorouting','on');
add_line(path,'RxBits/1','ErrRate/2','autorouting','on');
add_line(path,'ErrRate/1','ToWS/1','autorouting','on');
end

% ======================= 关键修复：确保命中唯一 EMChart（标量） =======================
function addMatlabFcnViaChart(dst, pos, scriptText)
add_block('simulink/User-Defined Functions/MATLAB Function', dst, 'Position', pos);
drawnow;

rt = sfroot;
hblk = get_param(dst,'Handle');

% 1) 优先按 SimulinkHandle 精确匹配（最稳）
charts = rt.find('-isa','Stateflow.EMChart','SimulinkHandle',hblk);

% 2) 兜底：按 Path 精确匹配（某些情况下 SimulinkHandle 匹配不到）
if isempty(charts)
    try
        charts = rt.find('-isa','Stateflow.EMChart','Path',dst);
    catch
        charts = [];
    end
end

% 3) 再兜底：按 Name + ParentPath 模糊缩小范围
if isempty(charts)
    blkName = get_param(dst,'Name');
    parentPath = get_param(dst,'Parent');
    c2 = rt.find('-isa','Stateflow.EMChart','Name',blkName);
    if ~isempty(c2)
        keep = false(size(c2));
        for k=1:numel(c2)
            try
                keep(k) = contains(c2(k).Path, parentPath);
            catch
                keep(k) = false;
            end
        end
        charts = c2(keep);
    end
end

if isempty(charts)
    error("已创建 MATLAB Function 块，但未找到对应 Stateflow.EMChart：%s", dst);
end

% ✅ 强制取第一个（标量），避免“参数必须为标量”
chart = charts(1);

% scriptText 必须是 char
if isstring(scriptText), scriptText = char(scriptText); end
chart.Script = scriptText;
end

% ======================= 通信块定位（按 Name 扫库） =======================
function blkPath = resolveCommBlock(nameList)
libs = {'commblk','comm','commchan','commtx','commrx','simulink'};
for n = 1:numel(nameList)
    nm = nameList{n};
    for i=1:numel(libs)
        lib = libs{i};
        try
            load_system(lib);
            hits = find_system(lib, ...
                'FollowLinks','on', ...
                'LookUnderMasks','all', ...
                'RegExp','off', ...
                'SearchDepth',Inf, ...
                'Name',nm);
            if ~isempty(hits)
                blkPath = hits{1};
                return;
            end
        catch
        end
    end
end
error("无法定位通信块（按Name）：%s", strjoin(string(nameList)," / "));
end

% ======================= MATLAB Function 脚本 =======================
function code = randBitsCode()
code = [
"function b = rand_bits()", newline, ...
"%#codegen", newline, ...
"cfg = evalin('base','cfg');", newline, ...
"persistent inited;", newline, ...
"if isempty(inited), rng(1); inited=true; end", newline, ...
"b = randi([0 1], cfg.phy.frameBits, 1);", newline, ...
"end", newline ...
];
end

function code = preambleInsertCode()
code = [
"function y = preamble_insert(payloadSym)", newline, ...
"%#codegen", newline, ...
"cfg = evalin('base','cfg');", newline, ...
"L = cfg.phy.preambleHalfLen; reps = cfg.phy.preambleReps;", newline, ...
"persistent p0;", newline, ...
"if isempty(p0)", newline, ...
"  rng(2); b=randi([0 1],2*L,1);", newline, ...
"  b0=b(1:2:end); b1=b(2:2:end);", newline, ...
"  I=1-2*b0; Q=1-2*b1;", newline, ...
"  p0=(I+1j*Q)/sqrt(2);", newline, ...
"end", newline, ...
"preamble = repmat(p0, reps, 1);", newline, ...
"y = [preamble; payloadSym(:)];", newline, ...
"end", newline ...
];
end

function code = interfGenCode()
code = [
"function i = interf_gen(x)", newline, ...
"%#codegen", newline, ...
"cfg = evalin('base','cfg');", newline, ...
"sig = x(:); N = length(sig);", newline, ...
"Ps = mean(abs(sig).^2) + 1e-12;", newline, ...
"JS = 10^(cfg.emc.JS_dB/10); Pj = Ps * JS;", newline, ...
"t = (0:N-1).';", newline, ...
"switch cfg.emc.type", newline, ...
"  case 1", newline, ...
"    i = sqrt(Pj/2) * (randn(N,1)+1j*randn(N,1));", newline, ...
"  case 2", newline, ...
"    A = sqrt(Pj); f0 = cfg.emc.toneOffset_Hz;", newline, ...
"    i = A * exp(1j*2*pi*f0*t/cfg.phy.Rs);", newline, ...
"  case 3", newline, ...
"    duty = cfg.emc.pulseDuty; per = cfg.emc.pulsePeriodSym;", newline, ...
"    onN = max(1, round(duty*per)); mask = false(N,1); pos = 1;", newline, ...
"    while pos <= N", newline, ...
"      mask(pos:min(pos+onN-1,N))=true; pos=pos+per;", newline, ...
"    end", newline, ...
"    P_on = Pj / max(duty,1e-6); i = zeros(N,1);", newline, ...
"    w = sqrt(P_on/2)*(randn(N,1)+1j*randn(N,1)); i(mask)=w(mask);", newline, ...
"  case 4", newline, ...
"    Ieq = Pj * 10^(-cfg.emc.ACIR_dB/10);", newline, ...
"    i = sqrt(Ieq/2) * (randn(N,1)+1j*randn(N,1));", newline, ...
"  case 5", newline, ...
"    Psrc = 10^(cfg.emc.cositeSrcP_dBW/10); coup = 10^(cfg.emc.cositeCoupling_dB/10);", newline, ...
"    Ieq = Psrc * coup;", newline, ...
"    i = sqrt(Ieq/2) * (randn(N,1)+1j*randn(N,1));", newline, ...
"  otherwise", newline, ...
"    i = zeros(N,1);", newline, ...
"end", newline, ...
"end", newline ...
];
end

function code = cfoCompCode()
code = [
"function y = cfo_comp(r)", newline, ...
"%#codegen", newline, ...
"cfg = evalin('base','cfg');", newline, ...
"rx = r(:); N = length(rx); Rs = cfg.phy.Rs; t = (0:N-1).';", newline, ...
"if cfg.rx.cfoMethod == 1", newline, ...
"  cfo_hat = cfg.leo.fD_Hz;", newline, ...
"else", newline, ...
"  L = cfg.phy.preambleHalfLen;", newline, ...
"  if N < 2*L", newline, ...
"    cfo_hat = 0;", newline, ...
"  else", newline, ...
"    r1 = rx(1:L); r2 = rx(L+1:2*L);", newline, ...
"    P = sum(conj(r1).*r2);", newline, ...
"    cfo_hat = angle(P) * Rs / (2*pi*L);", newline, ...
"  end", newline, ...
"end", newline, ...
"y = rx .* exp(-1j*2*pi*cfo_hat*t/Rs);", newline, ...
"end", newline ...
];
end
