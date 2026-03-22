function run_LEO_EMC_Sim()
% 运行生成的Simulink模型，并展示误码率输出

model = 'LEO_EMC_Sim_Simulink';
if ~bdIsLoaded(model)
    load_system(model);
end

% 你可以在这里修改 cfg（外行只改这几行就行）
cfg = evalin('base','cfg');
cfg.emc.type = 1;     % 1同频噪声 2单音 3脉冲 4邻频等效 5同址等效
cfg.emc.JS_dB = -5;
cfg.leo.fD_Hz = 30e3;
cfg.rx.cfoMethod = 2; % 1理想 2前导估计
assignin('base','cfg',cfg);

simOut = sim(model);

refBits = simOut.get('refBits');
rxBits = simOut.get('rxBits');
refBits = refBits(:);
rxBits = rxBits(:);
n = min(numel(refBits), numel(rxBits));
refBits = refBits(1:n);
rxBits = rxBits(1:n);
numErr = sum(refBits ~= rxBits);
errRate = [numErr / max(n,1), numErr, n];
assignin('base','errRate',errRate);
disp('?????????? errRate = [BER, numErr, numBits]');
fprintf("BER=%.3e, Errors=%d, Bits=%d\n", errRate(1), errRate(2), errRate(3));
end
