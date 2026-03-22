function makeDirs(outRoot, splits, classes)
    for si=1:numel(splits)
        for ci=1:numel(classes)
            d = fullfile(outRoot, splits{si}, classes{ci});
            if ~exist(d,'dir'), mkdir(d); end
        end
    end
end



%% =========================
% Local Functions: Power-aligned Waveform Sampler (for STFT+LeNet "real" usage)
% 说明：
%  - 主仿真仍是“功率级/系统级”计算 SINR/BER/THR（不改主逻辑）
%  - 这里新增一个采样器：把 simulateStarNet 里算出来的 (PS, PI, PJ, PN) 变成一段 IQ
%  - 分类器看到的 STFT 快照来自“功率对齐”的 IQ（而不是从 Event 标签编剧情造波形）
% =========================
