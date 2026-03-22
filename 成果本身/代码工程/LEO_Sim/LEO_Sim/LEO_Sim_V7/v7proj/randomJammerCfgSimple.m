function cfg = randomJammerCfgSimple(className, jsrDb)
    % 贴近你PPT“随机频段/强度/类别”的简化实现
    switch className
        case 'none'
            cfg = struct('type','none');
        case 'tone'
            f0 = 0.05 + 0.40*rand;               % 随机单音频点
            cfg = struct('type','tone','JSR_dB',jsrDb,'f0',f0);
        case 'pbnj'
            f1 = 0.03 + 0.30*rand;               % 随机带通起点
            bw = 0.05 + 0.20*rand;               % 随机带宽
            f2 = min(0.49, f1 + bw);
            cfg = struct('type','pbnj','JSR_dB',jsrDb,'band',[f1 f2]);
        case 'mod'
            df = 0.03 + 0.15*rand;               % 调制搬移
            duty = 0.15 + 0.35*rand;             % 突发占空
            cfg = struct('type','mod','JSR_dB',jsrDb,'df',df,'duty',duty);
        otherwise
            cfg = struct('type','none');
    end
end
