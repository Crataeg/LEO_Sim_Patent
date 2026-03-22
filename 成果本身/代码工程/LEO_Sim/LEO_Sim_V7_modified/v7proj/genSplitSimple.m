function genSplitSimple(outRoot, splitName, classes, numPerClass, Ns, SNRdB_list, JSR_dB_list, stft, imgSize)
    fprintf('  [IntfCls] Split=%s ...\n', splitName);
    for ci=1:numel(classes)
        cls = classes{ci};
        outDir = fullfile(outRoot, splitName, cls);
        for n=1:numPerClass
            snrDb = SNRdB_list(randi(numel(SNRdB_list)));
            jsrDb = JSR_dB_list(randi(numel(JSR_dB_list)));
 
            % 基带QPSK
            s = qpskRand(Ns);
 
            % AWGN
            EsN0 = 10^(snrDb/10);
            noiseVar = 1/EsN0;
            n_awgn = sqrt(noiseVar/2)*(randn(Ns,1)+1j*randn(Ns,1));
 
            % 干扰：类型 + 随机中心频率/带宽/突发位置（贴合你PPT“随机频段+强度+类别”）
            cfg = randomJammerCfgSimple(cls, jsrDb);
            j = genJammerSimple(cfg, s);
 
            r = s + n_awgn + j;
 
            img = makeSTFTImageSimple(r, stft, imgSize);
            fname = sprintf('%s_%05d_SNR%.1f_JSR%.1f.png', cls, n, snrDb, jsrDb);
            imwrite(img, fullfile(outDir, fname));
        end
    end
end
