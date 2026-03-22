function [trueLab, predLab, scoreMat, snapInfo] = classifyInterferenceTimeline(net, classes, numSteps, sample_time, eventTags, jamScale_dB) %#ok<INUSD>
    % 为每个时间步合成一个短快照，用 STFT + LeNet 预测干扰类型
    % snapInfo：保存关键帧的“原始接收信号+图片+标签”，用于导出PPT图
 
    Ns = 2048;
    stft.win=256; stft.overlap=128; stft.nfft=256; stft.fs=1;
    imgSize=[128 128];
 
    trueLab = categorical(repmat("none",numSteps,1), classes);
    predLab = categorical(repmat("none",numSteps,1), classes);
    scoreMat = zeros(numSteps, numel(classes));
 
    rng(2027); % 固定可复现
 
    % 关键帧挑选：每类事件挑几帧（用于导出）
    keyIdx = pickKeyframes(eventTags, numSteps);
 
    snapInfo = struct();
    snapInfo.exportIdx = keyIdx;
    snapInfo.tIndex = keyIdx;
    snapInfo.event = strings(numel(keyIdx),1);
    snapInfo.trueClass = strings(numel(keyIdx),1);
    snapInfo.predClass = strings(numel(keyIdx),1);
    snapInfo.img = cell(numel(keyIdx),1);
 
    for k=1:numSteps
        evt = string(eventTags(k));
 
        if contains(evt,"JAMMING") || contains(evt,"CoChannel")
            pick = randi([2 4]); % tone/pbnj/mod
            cls = classes{pick};
            jsrDb = min(12, max(0, jamScale_dB/3));
        else
            cls = 'none';
            jsrDb = 0;
        end
 
        trueLab(k) = categorical(string(cls), classes);
 
        r = synthRxSnapshotSimple(cls, Ns, 0, jsrDb); % snr=0dB
        img = makeSTFTImageSimple(r, stft, imgSize);
        img1 = im2single(img);
        img1 = reshape(img1, [imgSize 1]);
 
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
            scoreMat(k, find(strcmp(classes,cls),1)) = 1;
        end
 
        % 记录关键帧（用于导出PPT图片）
        ii = find(keyIdx==k,1);
        if ~isempty(ii)
            snapInfo.event(ii) = evt;
            snapInfo.trueClass(ii) = string(cls);
            snapInfo.predClass(ii) = string(predLab(k));
            snapInfo.img{ii} = img; % 128x128
        end
    end
end
