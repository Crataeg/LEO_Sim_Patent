function [trueLab, predLab, scoreMat, snapInfo] = classifyInterferenceTimeline_powerSampler( ...
    net, classes, numSteps, sample_time, sim, p_n, cfg) %#ok<INUSD>
    % 用功率级 (PS, PI, PJ, PN) 生成 IQ -> STFT -> LeNet
    %
    % 输入：
    %   sim.PS_mW / sim.PI_mW / sim.PJ_mW / sim.PJraw_mW  来自 simulateStarNet（本脚本已补齐）
    %   p_n：噪声功率(mW)
    %
    % 输出：
    %   trueLab：基于“功率谁更强”的粗略真值（用于自检/展示，不用于训练）
    %   predLab：LeNet 预测
    %   scoreMat：LeNet softmax 分数 (N×4)
    %   snapInfo：导出PPT关键帧用

    Ns = cfg.Ns;
    stft = cfg.Stft;
    imgSize = cfg.imgSize;

    trueLab = categorical(repmat("none",numSteps,1), classes);
    predLab = categorical(repmat("none",numSteps,1), classes);
    scoreMat = zeros(numSteps, numel(classes));

    % 关键帧（仍然用主仿真 Event 选时刻，只影响“导出哪些图”，不影响采样/分类）
    keyIdx = pickKeyframes(sim.Event, numSteps);

    snapInfo = struct();
    snapInfo.exportIdx = keyIdx;
    snapInfo.tIndex = keyIdx;
    snapInfo.event = strings(numel(keyIdx),1);
    snapInfo.trueClass = strings(numel(keyIdx),1);
    snapInfo.predClass = strings(numel(keyIdx),1);
    snapInfo.img = cell(numel(keyIdx),1);

    for k=1:numSteps
        % 取功率分解（mW）
        Ps = 0; Pi = 0; Pj = 0;
        if isfield(sim,'PS_mW'), Ps = sim.PS_mW(k); end
        if isfield(sim,'PI_mW'), Pi = sim.PI_mW(k); end
        if cfg.UsePostAJPower
            if isfield(sim,'PJ_mW'), Pj = sim.PJ_mW(k); end
        else
            if isfield(sim,'PJraw_mW'), Pj = sim.PJraw_mW(k);
            elseif isfield(sim,'PJ_mW'), Pj = sim.PJ_mW(k);
            end
        end
        Pn = p_n;

        % 基于功率的“粗真值”（仅用于展示/导出标题）
        clsTrue = inferTrueClassFromPowers(Pi, Pj, Pn, cfg);
        trueLab(k) = categorical(string(clsTrue), classes);

        % 生成功率对齐 IQ
        [r, ~] = sampleIQFromPowers(Ps, Pi, Pj, Pn, cfg, k); %#ok<ASGLU>

        % STFT -> 图片
        img = makeSTFTImageSimple(r, stft, imgSize);
        img1 = im2single(img);
        img1 = reshape(img1, [imgSize 1]);

        % 分类
        try
            [pl, sc] = classify(net, img1);
            pl = reordercats(pl, classes);
            predLab(k) = pl;

            sc = sc(:).';
            if numel(sc)==numel(classes)
                scoreMat(k,:) = sc;
            end
        catch
            predLab(k) = trueLab(k);
            scoreMat(k,:) = 0;
            scoreMat(k, find(strcmp(classes, char(clsTrue)),1)) = 1;
        end

        % 记录关键帧（用于导出PPT图片）
        ii = find(keyIdx==k,1);
        if ~isempty(ii)
            snapInfo.event(ii) = string(sim.Event(k));
            snapInfo.trueClass(ii) = string(clsTrue);
            snapInfo.predClass(ii) = string(predLab(k));
            snapInfo.img{ii} = img; % 128x128
        end
    end
end
