function generate_dataset_stft_3split()
% ============================================================
% generate_dataset_stft_3split.m
% 目的：生成 STFT 时频图数据集 (train/val/test) 用于干扰分类
%
% 类别（4类）:
%   none  : 无干扰
%   tone  : 单音干扰
%   pbnj  : 部分带噪声干扰
%   mod   : 调制干扰(QPSK)  (已增强：频偏 + 突发burst)
%
% 输出目录结构：
% dataset_stft/
%   train/none, train/tone, train/pbnj, train/mod
%   val/none,   val/tone,   val/pbnj,   val/mod
%   test/none,  test/tone,  test/pbnj,  test/mod
%
% 依赖：LTE Toolbox (lteTurboEncode / lteSymbolModulate)
% ============================================================

clear; clc; close all;

%% =============== 用户可调参数 ===============
outRoot = 'dataset_stft';

% 每类样本数（建议先小后大）
numTrainPerClass = 1000;
numValPerClass   = 200;
numTestPerClass  = 300;

% 多SNR/JSR增强鲁棒性
SNRdB_list  = [-5 0 3];
JSR_dB_list = [0 3 6];

K = 6144;                 % LTE Turbo block length

% STFT 参数（固定！）
stft.win = 256;
stft.overlap = 128;
stft.nfft = 256;
stft.fs = 1;

imgSize = [128 128];      % 输出图像大小

% 复现随机性
rng(2026);

%% =============== 创建文件夹 ===============
classes = {'none','tone','pbnj','mod'};
splits  = {'train','val','test'};
makeDirs(outRoot, splits, classes);

fprintf('==== Dataset Generation (train/val/test) Started ====\n');
fprintf('Train/class=%d, Val/class=%d, Test/class=%d\n', ...
    numTrainPerClass, numValPerClass, numTestPerClass);

%% =============== 生成 train / val / test ===============
genSplit(outRoot, 'train', classes, numTrainPerClass, K, SNRdB_list, JSR_dB_list, stft, imgSize);
genSplit(outRoot, 'val',   classes, numValPerClass,   K, SNRdB_list, JSR_dB_list, stft, imgSize);
genSplit(outRoot, 'test',  classes, numTestPerClass,  K, SNRdB_list, JSR_dB_list, stft, imgSize);

fprintf('\n✅ Done! Dataset saved to: %s\n', fullfile(pwd, outRoot));
fprintf('Next: train LeNet using imageDatastore(dataset_stft/train)\n');

end

%% ============================================================
%% 生成某个 split（train/val/test）
%% ============================================================
function genSplit(outRoot, splitName, classes, numPerClass, ...
                  K, SNRdB_list, JSR_dB_list, stft, imgSize)

fprintf('\n[%s] Generating...\n', upper(splitName));

for ci = 1:numel(classes)
    className = classes{ci};
    outDir = fullfile(outRoot, splitName, className);

    fprintf('  -> %s/%s: %d samples\n', splitName, className, numPerClass);

    for n = 1:numPerClass

        % ------- 随机抽一个SNR & JSR -------
        snrDb = SNRdB_list(randi(numel(SNRdB_list)));
        jsrDb = JSR_dB_list(randi(numel(JSR_dB_list)));

        % ------- 生成“干净信号” s -------
        u = randi([0 1], K, 1);
        c = lteTurboEncode(u);
        s = lteSymbolModulate(c, 'QPSK');

        % ------- AWGN -------
        EsN0 = 10^(snrDb/10);
        noiseVar = 1/EsN0;
        n_awgn = sqrt(noiseVar/2) * (randn(size(s)) + 1j*randn(size(s)));

        % ------- 干扰 -------
        cfg = randomJammerCfg(className, jsrDb);
        j = genJammer(cfg, s, c);

        % 接收信号
        r = s + n_awgn + j;

        % ------- STFT图像 -------
        img = makeSTFTImage(r, stft, imgSize);

        % 文件名（包含参数便于追溯）
        fname = sprintf('%s_%05d_SNR%.1f_JSR%.1f.png', className, n, snrDb, jsrDb);
        imwrite(img, fullfile(outDir, fname));

        if mod(n, 200) == 0
            fprintf('     %s/%s: %d/%d\n', splitName, className, n, numPerClass);
        end
    end
end
end

%% ============================================================
%% 随机化干扰参数
%% ============================================================
function cfg = randomJammerCfg(className, jsrDb)

switch className
    case 'none'
        cfg = struct('type','none');

    case 'tone'
        % tone: 频点随机
        f0 = 0.05 + 0.40*rand;  % [0.05,0.45]
        cfg = struct('type','tone', 'JSR_dB',jsrDb, 'f0',f0);

    case 'pbnj'
        % pbnj: 频带随机
        f1 = 0.05 + 0.25*rand;
        bw = 0.05 + 0.15*rand;
        f2 = min(0.49, f1 + bw);
        cfg = struct('type','pbnj', 'JSR_dB',jsrDb, 'band',[f1 f2]);

    case 'mod'
        % ===============================
        % ✅ 改动1：调制干扰强制频偏（避免 df≈0 和 none 很像）
        % ===============================
        df = 0.03 + 0.15*rand;  % df ∈ [0.03, 0.18]
        cfg = struct('type','mod', 'JSR_dB',jsrDb, 'df',df);

    otherwise
        error('Unknown className.');
end

end

%% ============================================================
%% STFT → 灰度图
%% ============================================================
function imgOut = makeSTFTImage(r, stft, imgSize)

[S,~,~] = spectrogram(r, stft.win, stft.overlap, stft.nfft, stft.fs, 'centered');
P = abs(S).^2;
img = 10*log10(P + 1e-12);

% 归一化到[0,1]
img = img - min(img(:));
img = img ./ (max(img(:)) + 1e-12);

% resize
imgOut = imresize(img, imgSize);

end

%% ============================================================
%% 干扰生成（与你原代码一致 + mod增强 burst）
%% ============================================================
function j = genJammer(cfg, s, c)

Ns = length(s);

switch cfg.type
    case 'none'
        j = zeros(Ns,1);

    case 'tone'
        n = (0:Ns-1).';
        f0 = cfg.f0;
        j0 = exp(1j*2*pi*f0*n);
        j = scaleToJSR(j0, s, cfg.JSR_dB);

    case 'pbnj'
        u = randn(Ns,1) + 1j*randn(Ns,1);
        band = cfg.band;
        hBP = fir1(80, band);
        j0 = filter(hBP, 1, u);
        j = scaleToJSR(j0, s, cfg.JSR_dB);

    case 'mod'
        df = cfg.df;

        % 生成干扰QPSK
        bI = randi([0 1], length(c), 1);
        xI = lteSymbolModulate(bI,'QPSK');
        if length(xI) ~= Ns
            xI = xI(1:Ns);
        end
        n = (0:Ns-1).';

        % ===============================
        % ✅ 改动2：突发式 burst mask（更容易与none区分）
        % ===============================
        mask = zeros(Ns,1);
        burstLen = round(0.2 * Ns);                  % burst长度=20%帧长
        startIdx = randi([1, Ns - burstLen + 1]);    % 随机起点
        mask(startIdx:startIdx + burstLen - 1) = 1;

        % 调制干扰（频偏 + 突发）
        j0 = (xI .* exp(1j*2*pi*df*n)) .* mask;

        j = scaleToJSR(j0, s, cfg.JSR_dB);

    otherwise
        error('Unknown jammer type.');
end

end

function j = scaleToJSR(j0, s, JSR_dB)

Ps = mean(abs(s).^2);
Pj_target = Ps * 10^(JSR_dB/10);
j = j0 * sqrt(Pj_target / (mean(abs(j0).^2) + 1e-12));

end

%% ============================================================
%% 创建目录
%% ============================================================
function makeDirs(outRoot, splits, classes)

for si = 1:numel(splits)
    for ci = 1:numel(classes)
        d = fullfile(outRoot, splits{si}, classes{ci});
        if ~exist(d,'dir')
            mkdir(d);
        end
    end
end

end
