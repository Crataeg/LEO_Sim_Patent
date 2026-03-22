%% LEO_EMC_Sim.m
% ==============================================================
% 低轨(LEO)卫星通信系统 EMC 仿真（通信视角、纯仿真、零基础可读）
% --------------------------------------------------------------
% 你会得到：
% 1) LEO过顶过程的距离R(t)、仰角El(t)、多普勒fD(t)
% 2) 链路预算得到 C(t)（期望信号功率）、N（噪声功率）
% 3) EMC干扰 I(t)（同频/邻频/脉冲/同址等效）
% 4) 在若干时间点做“波形级仿真”，输出 BER/BLER/吞吐
%
% 运行：
%   1) 保存为 LEO_EMC_Sim.m
%   2) MATLAB里运行：LEO_EMC_Sim
%
% 说明：
% - 本代码不依赖工具箱（纯基础MATLAB）
% - 调制：QPSK
% - 编码：卷积码(1/2, K=7, [171 133]_oct) + 硬判决Viterbi
% - 同步：重复前导的CFO估计 + 频偏补偿 + 复数增益估计
% - 信道：平坦Rician(可选) + 多普勒频偏
% - EMC：把干扰当作接收端“额外的干扰功率/波形”，从通信指标衡量
% ==============================================================

clear; clc; close all;

%% ============ 0) 配置（外行只要改这里） ======================
cfg = defaultConfig();

% 你想试哪个EMC干扰场景？把下面一行改成：
% 'none' | 'cochannel_awgn' | 'cochannel_tone' | 'pulsed_noise' | 'adjacent_acir' | 'cosite_equiv'
cfg.interf.type = 'cochannel_awgn';

% 干扰强度设置（常用：J/S dB，越大越严重）
cfg.interf.JS_dB = -5;     % -5 dB表示干扰功率比信号弱一些；0 dB表示一样强；+10 dB表示很强

% 邻频场景时用（越大越好，表示邻频泄漏越少）
cfg.interf.ACIR_dB = 30;

% 同址等效场景时用（耦合越负越好）
cfg.interf.cosite_coupling_dB = -60;   % -60 dB耦合
cfg.interf.cosite_src_P_dBW = 0;       % 干扰源功率(等效到源端) 0 dBW=1W

% 脉冲干扰参数（pulsed_noise用）
cfg.interf.pulse.duty = 0.10;          % 占空比10%
cfg.interf.pulse.period_sym = 200;     % 每200个符号来一次脉冲
cfg.interf.pulse.on_sym = round(cfg.interf.pulse.duty * cfg.interf.pulse.period_sym);

% 频偏补偿方式：
% 'ideal'   -> 假设接收机知道真实多普勒频偏并完美补偿（乐观）
% 'estimate'-> 用重复前导估计频偏（更真实、也更“LEO”）
cfg.rx.cfoMethod = 'estimate';

% 评估点数量（在可见时间内抽取多少个时间点做波形级仿真）
cfg.sim.numEvalPoints = 12;

% 每个评估点Monte Carlo多少帧（越大越准，越慢）
cfg.sim.numFramesPerPoint = 150;

%% ============ 1) 生成LEO过顶几何：R(t), El(t), fD(t) ==========
[tt, geo] = leoPassGeometry(cfg);

% 只保留可见部分（仰角>最小仰角）
vis = geo.visible;
t_vis = tt(vis);

if isempty(t_vis)
    error('该配置下卫星在给定时间窗内不可见，请增大时间窗或降低最小仰角。');
end

%% ============ 2) 链路预算：C(t), N, 以及干扰功率I(t) ==========
[C_W, N_W, I_W, SINR_dB] = computeCNI(cfg, tt, geo);

%% ============ 3) 在若干时间点评估“波形级通信性能” ===========
% 在可见时间里均匀抽取若干点
t_eval = linspace(t_vis(1), t_vis(end), cfg.sim.numEvalPoints);

results = struct();
results.t = t_eval(:);
results.fD = zeros(numel(t_eval),1);
results.SINR_dB = zeros(numel(t_eval),1);
results.BER = zeros(numel(t_eval),1);
results.BLER = zeros(numel(t_eval),1);
results.throughput_bps = zeros(numel(t_eval),1);

fprintf('开始波形级仿真（%d个评估点，每点%d帧）...\n', ...
    cfg.sim.numEvalPoints, cfg.sim.numFramesPerPoint);

for k = 1:numel(t_eval)
    idx = nearestIndex(tt, t_eval(k));

    % 取该时刻的C/N/I（功率比即可）
    Ck = C_W(idx);
    Nk = N_W;
    Ik = I_W(idx);

    ratioN = Nk / Ck;        % N/C
    ratioI = Ik / Ck;        % I/C
    fDk = geo.fD_Hz(idx);    % 多普勒(Hz)

    results.fD(k) = fDk;
    results.SINR_dB(k) = SINR_dB(idx);

    % Monte Carlo：多帧统计 BER/BLER/吞吐
    [ber, bler, thr] = runMonteCarlo(cfg, ratioN, ratioI, fDk);

    results.BER(k) = ber;
    results.BLER(k) = bler;
    results.throughput_bps(k) = thr;

    fprintf('  [%2d/%2d] t=%7.1fs  fD=%8.0fHz  SINR=%6.2fdB  BLER=%5.3f  Thr=%.1f kbps\n', ...
        k, numel(t_eval), t_eval(k), fDk, results.SINR_dB(k), bler, thr/1e3);
end

%% ============ 4) 画图：几何、多普勒、SINR、通信指标 ===========
plotAll(cfg, tt, geo, C_W, N_W, I_W, SINR_dB, results);

disp('完成。你可以去 defaultConfig() 或 cfg.* 里改参数重新跑。');

%% =================================================================
%%                         局部函数区
%% =================================================================

function cfg = defaultConfig()
    % ---------------- 基本物理常数 ----------------
    cfg.const.c = 299792458;            % 光速
    cfg.const.k = 1.380649e-23;         % 玻尔兹曼常数
    cfg.const.Re = 6371e3;              % 地球半径(近似)
    cfg.const.mu = 3.986004418e14;      % 地球引力常数

    % ---------------- LEO 场景参数 ----------------
    cfg.scenario.altitude_m = 600e3;    % 轨道高度
    cfg.scenario.tStart_s = -900;       % 模拟时间窗（相对最近点）
    cfg.scenario.tEnd_s   = +900;
    cfg.scenario.dt_s     = 1.0;        % 系统级时间步长
    cfg.scenario.minElev_deg = 5;       % 最小仰角（低于它视作不可用/不可见）

    % ---------------- 通信参数（纯仿真用）---------
    cfg.phy.fc_Hz = 2.0e9;              % 载频（影响多普勒/路径损耗）
    cfg.phy.B_Hz  = 1.0e6;              % 信号带宽（决定噪声功率）
    cfg.phy.Rs    = 1.0e6;              % 符号率（简化：Rs≈B）
    cfg.phy.M     = 4;                  % QPSK
    cfg.phy.sps   = 1;                  % 每符号采样点数（简化为1）

    % ---------------- 帧结构 -----------------------
    cfg.frame.Ninfo = 1000;             % 每帧信息比特数
    cfg.frame.preambleHalfLen = 8;      % 重复前导：半段长度L（两段相同）
    cfg.frame.preambleReps = 4;         % 前导重复次数（>=2；越大越稳健）
    % 前导总符号数 = L * preambleReps
    % CFO估计用前2段：L + L
    % 提示：重复前导的CFO估计“无模糊范围”约为 ±Rs/(2L)；
    % L越小范围越大，但估计噪声越大。

    % ---------------- 信道（平坦Rician）------------
    cfg.channel.enableRician = true;
    cfg.channel.K_dB = 6;               % Rician K因子(dB)，越大直达波越强

    % ---------------- 链路预算参数 -----------------
    cfg.link.EIRP_dBW = 20;             % 发射端EIRP（例：20 dBW=100W等效全向）
    cfg.link.Gr_dBi   = 25;             % 接收天线增益
    cfg.link.Lmisc_dB = 2;              % 杂项损耗（极化/指向/线缆等）

    % ---------------- 接收机噪声 -------------------
    cfg.rx.NF_dB = 3;                   % 噪声系数
    cfg.rx.T_K   = 290;                 % 系统温度（近似室温）

    % ---------------- CFO补偿方式 ------------------
    cfg.rx.cfoMethod = 'estimate';      % 'ideal' or 'estimate'

    % ---------------- 干扰/EMC参数 -----------------
    cfg.interf.type = 'cochannel_awgn';
    cfg.interf.JS_dB = -5;
    cfg.interf.toneOffset_Hz = 10e3;    % 单音干扰频偏（相对基带）
    cfg.interf.ACIR_dB = 30;            % 邻频等效（越大越好）
    cfg.interf.cosite_coupling_dB = -60;
    cfg.interf.cosite_src_P_dBW = 0;
    cfg.interf.pulse.duty = 0.1;
    cfg.interf.pulse.period_sym = 200;
    cfg.interf.pulse.on_sym = 20;

    % ---------------- 仿真控制 ---------------------
    cfg.sim.numEvalPoints = 12;
    cfg.sim.numFramesPerPoint = 150;
end

function [tt, geo] = leoPassGeometry(cfg)
    % 计算LEO过顶几何（简化二维圆轨道、地面站在赤道、过顶最近点t=0）
    Re = cfg.const.Re;
    mu = cfg.const.mu;
    c  = cfg.const.c;

    h  = cfg.scenario.altitude_m;
    Rs = Re + h;

    % 圆轨道角速度
    omega = sqrt(mu / Rs^3);    % rad/s

    tt = (cfg.scenario.tStart_s : cfg.scenario.dt_s : cfg.scenario.tEnd_s).';

    % 中心角 theta=omega*t（t=0时最近点/过顶）
    theta = omega * tt;

    % 二维坐标：地面点在(Re,0)，卫星在(Rs*cosθ, Rs*sinθ)
    dx = Rs*cos(theta) - Re;
    dy = Rs*sin(theta);

    R = sqrt(dx.^2 + dy.^2);                 % 斜距
    el = atan2(dx, dy) * 180/pi;             % 仰角（atan2(竖直,水平)）
    % 解释：竖直分量=dx，水平分量=dy。t=0时dy=0，仰角=90°。

    % 可见：仰角>minElev
    visible = el > cfg.scenario.minElev_deg;

    % 径向速度 v_r = dR/dt
    % R = sqrt(Re^2 + Rs^2 - 2*Re*Rs*cosθ)
    % dR/dt = (Re*Rs*omega*sinθ)/R
    v_r = (Re*Rs*omega*sin(theta)) ./ max(R, 1);   % m/s，符号表示远离/靠近

    % 多普勒频移
    fc = cfg.phy.fc_Hz;
    fD = (v_r / c) * fc;                       % Hz

    geo.R_m = R;
    geo.el_deg = el;
    geo.visible = visible;
    geo.vr_mps = v_r;
    geo.fD_Hz = fD;
end

function [C_W, N_W, I_W, SINR_dB] = computeCNI(cfg, tt, geo)
    % 链路预算得到 C(t)，噪声 N，干扰 I(t)，以及 SINR
    c = cfg.const.c;

    fc = cfg.phy.fc_Hz;
    lambda = c / fc;

    % 自由空间路径损耗 FSPL = (4πR/λ)^2
    R = geo.R_m;
    FSPL_dB = 20*log10(4*pi*R/lambda);

    % 接收功率 dBW
    Pr_dBW = cfg.link.EIRP_dBW + cfg.link.Gr_dBi - FSPL_dB - cfg.link.Lmisc_dB;

    % 信号功率（瓦特）
    C_W = 10.^(Pr_dBW/10);

    % 噪声功率 N = k*T*B*F
    kB = cfg.const.k;
    T  = cfg.rx.T_K;
    B  = cfg.phy.B_Hz;
    F  = 10^(cfg.rx.NF_dB/10);
    N_W = kB * T * B * F;

    % 干扰功率 I(t)
    I_W = zeros(size(C_W));
    for i = 1:numel(tt)
        I_W(i) = interferencePower(cfg, C_W(i));
    end

    % 不可见时刻可把C置小，或SINR置NaN，这里直接标NaN更直观
    SINR = C_W ./ (N_W + I_W);
    SINR_dB = 10*log10(SINR);
    SINR_dB(~geo.visible) = NaN;
end

function I = interferencePower(cfg, C)
    % 返回干扰功率（瓦特）
    % 统一口径：很多场景用J/S指定，即 I = C * 10^(JS/10)
    switch lower(cfg.interf.type)
        case 'none'
            I = 0;

        case {'cochannel_awgn','cochannel_tone','pulsed_noise'}
            I = C * 10^(cfg.interf.JS_dB/10);

        case 'adjacent_acir'
            % 邻频等效：假设邻频干扰在接收机附近的“邻频道功率”与信号同量级，
            % 经ACIR折算成等效落入本带的干扰
            P_adj = C; % 这里用一个简化假设：邻频道到达功率≈C（你可以改成更大/更小）
            I = P_adj * 10^(-cfg.interf.ACIR_dB/10);

        case 'cosite_equiv'
            % 同址等效：干扰源功率 * 耦合（dB）
            Psrc = 10^(cfg.interf.cosite_src_P_dBW/10);
            coup = 10^(cfg.interf.cosite_coupling_dB/10);
            I = Psrc * coup;

        otherwise
            error('未知干扰类型：%s', cfg.interf.type);
    end
end

function [ber, bler, thr_bps] = runMonteCarlo(cfg, ratioN, ratioI, fD_Hz)
    % 在某个时刻（对应某个多普勒、多种干扰强度）跑多帧统计
    numFrames = cfg.sim.numFramesPerPoint;

    bitErr = 0;
    bitTot = 0;
    blkErr = 0;

    % 有效吞吐：成功帧的信息比特 / 总时间
    % 一帧持续时间：总符号数 / Rs
    % 注意：我们用“解码正确才算吞吐”，解码错误吞吐=0
    goodBits = 0;

    for n = 1:numFrames
        [be, bt, blk_ok] = simulateOneFrame(cfg, ratioN, ratioI, fD_Hz);

        bitErr = bitErr + be;
        bitTot = bitTot + bt;
        if ~blk_ok
            blkErr = blkErr + 1;
        else
            goodBits = goodBits + cfg.frame.Ninfo;
        end
    end

    ber = bitErr / max(bitTot,1);
    bler = blkErr / numFrames;

    % 吞吐（bps）：成功信息比特 / 总仿真时长
    Nsym_frame = totalSymbolsPerFrame(cfg);
    Tframe = Nsym_frame / cfg.phy.Rs;
    thr_bps = goodBits / (numFrames * Tframe);
end

function [bitErrors, bitTotal, blockOK] = simulateOneFrame(cfg, ratioN, ratioI, fD_Hz)
    % 生成一帧 -> 加信道/多普勒/干扰/噪声 -> 同步/解调/译码 -> 统计误码
    Rs = cfg.phy.Rs;

    % ---------------- 1) 生成信息比特 ----------------
    u = randi([0 1], cfg.frame.Ninfo, 1);

    % ---------------- 2) 卷积编码(可关闭) ------------
    coded = convEncode(u);   % rate 1/2, 约输出 2*(Ninfo+6) bits

    % ---------------- 3) QPSK调制 -------------------
    payloadSym = qpskMod(coded);

    % ---------------- 4) 构造重复前导（用于频偏估计）-
    [preambleSym, preambleRef] = buildRepeatedPreamble(cfg);

    tx = [preambleSym; payloadSym];  % 一帧符号（复数）

    Nsym = numel(tx);
    n = (0:Nsym-1).';

    % ---------------- 5) 平坦Rician信道（复增益）----
    g = 1;
    if cfg.channel.enableRician
        g = ricianGain(cfg.channel.K_dB);
    end
    % txNorm功率≈1，g均方≈1

    % ---------------- 6) 多普勒：作为频偏（Hz）--------
    % 这里把多普勒当作“整帧近似常数频偏”（帧很短，合理）
    cfo = fD_Hz;

    % ---------------- 7) 接收信号：信道+频偏 ----------
    rx = (tx * g) .* exp(1j*2*pi*cfo*n/Rs);

    % ---------------- 8) 生成干扰波形（均值功率=ratioI）-
    i = genInterferenceWaveform(cfg, ratioI, Nsym);

    % ---------------- 9) 加噪声（均值功率=ratioN）-----
    w = sqrt(ratioN/2) * (randn(Nsym,1)+1j*randn(Nsym,1));

    r = rx + i + w;

    % ---------------- 10) 频偏补偿 --------------------
    switch lower(cfg.rx.cfoMethod)
        case 'ideal'
            cfo_hat = cfo;

        case 'estimate'
            % 用重复前导估计频偏（只用前2段）
            L = cfg.frame.preambleHalfLen;
            cfo_hat = estimateCFO_fromRepeated(r, L, Rs);

        otherwise
            error('未知cfoMethod：%s', cfg.rx.cfoMethod);
    end

    r_cfo = r .* exp(-1j*2*pi*cfo_hat*n/Rs);

    % ---------------- 11) 估计复增益（用已知前导）-----
    % 用全部前导做最小二乘估计：g_hat = (r*p*)/(p*p*)
    preLen = numel(preambleRef);
    ypre = r_cfo(1:preLen);
    g_hat = (ypre' * conj(preambleRef)) / (preambleRef' * conj(preambleRef));
    r_eq = r_cfo / max(g_hat, 1e-12);

    % ---------------- 12) 去除前导，解调payload -------
    y_payload = r_eq(preLen+1:end);
    hardBits = qpskDemodHard(y_payload);

    % ---------------- 13) Viterbi译码 -----------------
    u_hat = viterbiDecodeHard(hardBits);

    % u_hat长度会略大（包含尾比特），取前Ninfo
    u_hat = u_hat(1:cfg.frame.Ninfo);

    % ---------------- 14) 统计误码与块错误 ------------
    diff = (u_hat ~= u);
    bitErrors = sum(diff);
    bitTotal  = numel(u);
    blockOK   = (bitErrors == 0);
end

function Nsym = totalSymbolsPerFrame(cfg)
    % 总符号数 = 前导符号 + payload符号
    preSym = cfg.frame.preambleHalfLen * cfg.frame.preambleReps;

    % 编码后比特数 = 2*(Ninfo + 6)  (K=7 -> memory=6)
    Ncoded = 2*(cfg.frame.Ninfo + 6);

    % QPSK：2 bits/sym -> payload符号数 = Ncoded/2 = Ninfo+6
    paySym = (Ncoded/2);

    Nsym = preSym + paySym;
end

function [preambleSym, ref] = buildRepeatedPreamble(cfg)
    % 构造重复前导：生成一段随机QPSK序列p0，重复 preambleReps 次
    L = cfg.frame.preambleHalfLen;
    reps = cfg.frame.preambleReps;

    % 生成L个QPSK符号
    b = randi([0 1], 2*L, 1);
    p0 = qpskMod(b);

    preambleSym = repmat(p0, reps, 1);
    ref = preambleSym; % 已知参考
end

function cfo_hat = estimateCFO_fromRepeated(r, L, Rs)
    % 重复前导CFO估计：使用前两段相同序列
    % P = sum(conj(r1).*r2), 旋转角 = angle(P)
    % CFO估计 = angle(P) / (2*pi*L) * Rs
    if numel(r) < 2*L
        cfo_hat = 0;
        return;
    end
    r1 = r(1:L);
    r2 = r(L+1:2*L);
    P = sum(conj(r1).*r2);
    cfo_hat = angle(P) * Rs / (2*pi*L);
end

function sym = qpskMod(bits)
    % bits: 0/1 column, length even
    if mod(numel(bits),2) ~= 0
        error('QPSK调制要求比特数为偶数。');
    end
    b0 = bits(1:2:end);
    b1 = bits(2:2:end);

    I = 1 - 2*b0;  % 0->+1, 1->-1
    Q = 1 - 2*b1;

    sym = (I + 1j*Q)/sqrt(2);  % 单位平均功率
end

function bits = qpskDemodHard(sym)
    % 硬判决：实部>0判0，否则判1；虚部同理
    b0 = real(sym) < 0;
    b1 = imag(sym) < 0;

    bits = zeros(2*numel(sym),1);
    bits(1:2:end) = b0;
    bits(2:2:end) = b1;
end

function coded = convEncode(u)
    % 约束长度K=7，生成多项式[171 133]_oct（经典NASA/CCSDS风格）
    % 输出：每输入1bit输出2bit，最后加6个尾比特0终止
    g1 = [1 1 1 1 0 0 1];  % 171_oct = 1111001
    g2 = [1 0 1 1 0 1 1];  % 133_oct = 1011011
    K  = 7;
    m  = K-1;

    u_tail = [u; zeros(m,1)];
    state = zeros(1,K); % [u(k), u(k-1), ..., u(k-6)]

    coded = zeros(2*numel(u_tail),1);

    for k = 1:numel(u_tail)
        state = [u_tail(k) state(1:end-1)];
        o1 = mod(sum(state .* g1), 2);
        o2 = mod(sum(state .* g2), 2);
        coded(2*k-1) = o1;
        coded(2*k)   = o2;
    end
end

function u_hat = viterbiDecodeHard(rxBits)
    % 硬判决Viterbi译码（匹配convEncode）
    % rxBits长度应为偶数（每2bit对应一个输入bit）
    if mod(numel(rxBits),2) ~= 0
        error('Viterbi译码要求输入长度为偶数。');
    end

    g1 = [1 1 1 1 0 0 1];
    g2 = [1 0 1 1 0 1 1];
    K  = 7;
    m  = K-1;
    nStates = 2^m;

    nSteps = numel(rxBits)/2;

    % 预计算：对每个state和输入bit，得到nextState和输出(2bits)
    nextState = zeros(nStates,2);
    outBits   = zeros(nStates,2,2); % outBits(state, input+1, [o1 o2])

    for s = 0:nStates-1
        mem = de2bi(s, m, 'left-msb'); % [u(k-1)...u(k-6)]
        for in = 0:1
            shift = [in mem]; % K=7
            o1 = mod(sum(shift .* g1), 2);
            o2 = mod(sum(shift .* g2), 2);
            memNext = shift(1:end-1); % 新的m位记忆
            sNext = bi2de(memNext, 'left-msb');

            nextState(s+1,in+1) = sNext;
            outBits(s+1,in+1,1) = o1;
            outBits(s+1,in+1,2) = o2;
        end
    end

    % Viterbi DP
    INF = 1e9;
    pm = INF*ones(nStates,1);
    pm(1) = 0; % 终止码假设从全0状态开始

    prevState = zeros(nStates, nSteps);
    prevInput = zeros(nStates, nSteps);

    for t = 1:nSteps
        rpair = rxBits(2*t-1:2*t).';  % [r1 r2]
        pmNew = INF*ones(nStates,1);

        for s = 0:nStates-1
            if pm(s+1) >= INF/2
                continue;
            end
            for in = 0:1
                sn = nextState(s+1,in+1);
                ob = squeeze(outBits(s+1,in+1,:)).';  % [o1 o2]
                bm = sum(xor(ob, rpair));             % 汉明距离
                metric = pm(s+1) + bm;

                if metric < pmNew(sn+1)
                    pmNew(sn+1) = metric;
                    prevState(sn+1,t) = s;
                    prevInput(sn+1,t) = in;
                end
            end
        end

        pm = pmNew;
    end

    % 终止码：期望回到状态0（全0）
    sBest = 0;

    u_hat = zeros(nSteps,1);
    for t = nSteps:-1:1
        in = prevInput(sBest+1,t);
        sPrev = prevState(sBest+1,t);
        u_hat(t) = in;
        sBest = sPrev;
    end
end

function g = ricianGain(K_dB)
    % 生成一个平坦Rician复增益，均方约为1
    K = 10^(K_dB/10);
    % 直达分量
    phi = 2*pi*rand;
    s = sqrt(K/(K+1)) * exp(1j*phi);
    % 散射分量
    w = sqrt(1/(K+1)) * (randn + 1j*randn)/sqrt(2);
    g = s + w;
end

function i = genInterferenceWaveform(cfg, ratioI, Nsym)
    % 生成干扰波形，平均功率≈ratioI（因为信号功率归一为1）
    type = lower(cfg.interf.type);

    switch type
        case 'none'
            i = zeros(Nsym,1);

        case 'cochannel_awgn'
            i = sqrt(ratioI/2) * (randn(Nsym,1)+1j*randn(Nsym,1));

        case 'cochannel_tone'
            % 同频单音干扰：i = A*exp(j2πf n/Rs)
            Rs = cfg.phy.Rs;
            f0 = cfg.interf.toneOffset_Hz;
            n = (0:Nsym-1).';
            % 单音平均功率=|A|^2
            A = sqrt(ratioI);
            i = A * exp(1j*2*pi*f0*n/Rs);

        case 'pulsed_noise'
            % 脉冲噪声：在“开”的时候功率更高，使平均功率仍为ratioI
            duty = cfg.interf.pulse.duty;
            per  = cfg.interf.pulse.period_sym;
            onN  = cfg.interf.pulse.on_sym;

            mask = false(Nsym,1);
            pos = 1;
            while pos <= Nsym
                onEnd = min(pos+onN-1, Nsym);
                mask(pos:onEnd) = true;
                pos = pos + per;
            end

            % 为保持“平均功率=ratioI”，开的时候功率=ratioI/duty
            P_on = ratioI / max(duty,1e-6);
            noise = sqrt(P_on/2) * (randn(Nsym,1)+1j*randn(Nsym,1));
            i = zeros(Nsym,1);
            i(mask) = noise(mask);

        case 'adjacent_acir'
            % 邻频等效：用高斯噪声近似落入本带的等效干扰
            i = sqrt(ratioI/2) * (randn(Nsym,1)+1j*randn(Nsym,1));

        case 'cosite_equiv'
            % 同址等效：同样用高斯噪声近似（可进一步扩展为有色噪声）
            i = sqrt(ratioI/2) * (randn(Nsym,1)+1j*randn(Nsym,1));

        otherwise
            error('未知干扰波形类型：%s', cfg.interf.type);
    end
end

function idx = nearestIndex(t, x)
    [~, idx] = min(abs(t - x));
end

function plotAll(cfg, tt, geo, C_W, N_W, I_W, SINR_dB, results)
    vis = geo.visible;

    figure('Name','LEO几何与多普勒');
    subplot(3,1,1);
    plot(tt, geo.R_m/1e3, 'LineWidth',1.2); grid on;
    ylabel('斜距 R (km)');
    title('LEO过顶几何');

    subplot(3,1,2);
    plot(tt, geo.el_deg, 'LineWidth',1.2); grid on;
    hold on; yline(cfg.scenario.minElev_deg,'--');
    ylabel('仰角 El (deg)');

    subplot(3,1,3);
    plot(tt, geo.fD_Hz/1e3, 'LineWidth',1.2); grid on;
    ylabel('多普勒 f_D (kHz)');
    xlabel('时间 t (s)');

    figure('Name','C/N/I与SINR（系统级）');
    subplot(2,1,1);
    plot(tt(vis), 10*log10(C_W(vis)), 'LineWidth',1.2); grid on; hold on;
    yline(10*log10(N_W),'--','LineWidth',1.0);
    plot(tt(vis), 10*log10(I_W(vis)+eps), 'LineWidth',1.2);
    legend('C=信号功率(dBW)','N=噪声功率(dBW)','I=干扰功率(dBW)','Location','best');
    ylabel('功率 (dBW)');
    title('链路预算输出：C/N/I');

    subplot(2,1,2);
    plot(tt, SINR_dB, 'LineWidth',1.2); grid on;
    ylabel('SINR (dB)'); xlabel('时间 t (s)');
    title('SINR(t)=C/(N+I)');

    figure('Name','波形级评估点结果');
    subplot(3,1,1);
    plot(results.t, results.SINR_dB, 'o-','LineWidth',1.2); grid on;
    ylabel('SINR(dB)'); title('评估点SINR');

    subplot(3,1,2);
    semilogy(results.t, max(results.BER,1e-6), 'o-','LineWidth',1.2); grid on;
    ylabel('BER (log)'); title('BER（越低越好）');

    subplot(3,1,3);
    plot(results.t, results.BLER, 'o-','LineWidth',1.2); grid on;
    ylabel('BLER'); xlabel('时间 t (s)');
    title('BLER（块错误率，越低越好）');

    figure('Name','吞吐（仅成功帧计入）');
    plot(results.t, results.throughput_bps/1e3, 'o-','LineWidth',1.2); grid on;
    ylabel('吞吐 (kbps)'); xlabel('时间 t (s)');
    title('吞吐随时间变化（解码成功才算）');

    % 吞吐CDF（简单做法：对评估点吞吐排序）
    figure('Name','吞吐CDF（评估点）');
    thr = sort(results.throughput_bps);
    p = (1:numel(thr))/numel(thr);
    plot(thr/1e3, p, 'LineWidth',1.2); grid on;
    xlabel('吞吐 (kbps)'); ylabel('CDF');
    title('吞吐CDF（评估点近似）');
end
