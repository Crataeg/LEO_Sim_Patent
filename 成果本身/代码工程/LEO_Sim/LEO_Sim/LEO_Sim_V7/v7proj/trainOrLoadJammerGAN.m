function [netG, netD, netQ] = trainOrLoadJammerGAN(seqLen, zDim, cDim, iters, modelFile, infoLambda)
    if nargin < 6 || isempty(infoLambda)
        infoLambda = 1.0;
    end

    % Try loading a compatible saved model
    if exist(modelFile,'file')==2
        try
            S = load(modelFile);
            if isfield(S,'netG') && isfield(S,'netD') && isfield(S,'netQ') && ...
               isfield(S,'seqLen') && isfield(S,'zDim') && isfield(S,'cDim') && ...
               S.seqLen==seqLen && S.zDim==zDim && S.cDim==cDim
                netG = S.netG; netD = S.netD; netQ = S.netQ;
                return;
            end
        catch
        end
    end

    fprintf('  [InfoGAN] No compatible saved model, training a lightweight InfoGAN...\n');

    % ---- Real data (unlabeled) : diverse envelope shapes ----
    nReal = 320;
    t = linspace(0,1,seqLen);
    realData = zeros(seqLen, nReal, 'single');
    for i = 1:nReal
        % bump
        c1 = 0.25 + 0.5*rand;
        w1 = 0.03 + 0.18*rand;
        bump = exp(-0.5*((t-c1)/w1).^2);

        % plateau
        a = 0.10 + 0.55*rand;
        b = min(0.98, a + (0.10 + 0.35*rand));
        plat = (t > a) & (t < b);

        % burst train
        nb = randi([1 4]);
        bursts = zeros(1,seqLen);
        for bb=1:nb
            c2 = 0.10 + 0.80*rand;
            w2 = 0.01 + 0.05*rand;
            bursts = bursts + exp(-0.5*((t-c2)/w2).^2);
        end
        bursts = bursts / (max(bursts)+1e-6);

        wB = rand; wP = rand; wR = rand;
        wsum = wB + wP + wR + 1e-6;

        shape = 0.06 + 0.70*(wB/wsum)*bump + 0.55*(wP/wsum)*plat + 0.65*(wR/wsum)*bursts;
        shape = shape + 0.05*randn(1,seqLen);
        shape = movmean(shape, 7);
        shape = min(max(shape,0),1);

        realData(:,i) = single(shape(:));
    end

    inDim = zDim + cDim;

    % ---- Generator G(z,c) -> envelope ----
    netG = dlnetwork(layerGraph([
        featureInputLayer(inDim,'Name','in')
        fullyConnectedLayer(128,'Name','g_fc1')
        reluLayer('Name','g_relu1')
        fullyConnectedLayer(seqLen,'Name','g_fc2')
        sigmoidLayer('Name','g_sig')
    ]));

    % ---- Discriminator D(x) -> real/fake ----
    netD = dlnetwork(layerGraph([
        featureInputLayer(seqLen,'Name','x')
        fullyConnectedLayer(128,'Name','d_fc1')
        leakyReluLayer(0.2,'Name','d_lrelu1')
        fullyConnectedLayer(1,'Name','d_fc2')
        sigmoidLayer('Name','d_sig')
    ]));

    % ---- Q network: predict c from xFake (InfoGAN lower bound, simplified) ----
    netQ = dlnetwork(layerGraph([
        featureInputLayer(seqLen,'Name','xq')
        fullyConnectedLayer(128,'Name','q_fc1')
        reluLayer('Name','q_relu1')
        fullyConnectedLayer(cDim,'Name','q_fc2')
        sigmoidLayer('Name','q_sig')
    ]));

    lr = 1e-3; batch = 24;
    avgG=[]; avgSqG=[]; avgD=[]; avgSqD=[]; avgQ=[]; avgSqQ=[];

    for it=1:iters
        idx = randi(nReal,[1 batch]);
        xReal = dlarray(realData(:,idx),'CB');

        % ---- Update D ----
        z  = dlarray(single(randn(zDim,batch)),'CB');
        cc = dlarray(single(rand(cDim,batch)),'CB');  % c in [0,1]
        zc = [z; cc];
        xFake = forward(netG, zc);

        [gradD, ~] = dlfeval(@dGradients, netD, xReal, xFake);
        [netD, avgD, avgSqD] = adamupdate(netD, gradD, avgD, avgSqD, it, lr);

        % ---- Update G and Q (code predictability) ----
        z2  = dlarray(single(randn(zDim,batch)),'CB');
        c2  = dlarray(single(rand(cDim,batch)),'CB');
        zc2 = [z2; c2];

        [gradG, gradQ, ~, ~] = dlfeval(@gqGradients, netG, netD, netQ, zc2, c2, infoLambda);
        [netG, avgG, avgSqG] = adamupdate(netG, gradG, avgG, avgSqG, it, lr);
        [netQ, avgQ, avgSqQ] = adamupdate(netQ, gradQ, avgQ, avgSqQ, it, lr);
    end

    try
        save(modelFile,'netG','netD','netQ','seqLen','zDim','cDim','infoLambda');
    catch
        save(modelFile,'netG','netD','netQ');
    end
    fprintf('  [InfoGAN] Training done. Saved to %s\n', modelFile);
end
