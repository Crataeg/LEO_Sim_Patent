function generateDatasetSimpleSTFT(outRoot)
    classes = {'none','tone','pbnj','mod'};
    splits  = {'train','val','test'};
    makeDirs(outRoot, splits, classes);
 
    % 你原脚本“随机强度/频段/类型”的核心思想：这里做成轻量可控版
    numTrain=600; numVal=120; numTest=180;
    SNRdB_list=[-5 0 3];
    JSR_dB_list=[0 3 6 9 12];
    Ns = 2048;
 
    stft.win=256; stft.overlap=128; stft.nfft=256; stft.fs=1;
    imgSize=[128 128];
 
    rng(2026);
 
    genSplitSimple(outRoot,'train',classes,numTrain,Ns,SNRdB_list,JSR_dB_list,stft,imgSize);
    genSplitSimple(outRoot,'val',  classes,numVal,  Ns,SNRdB_list,JSR_dB_list,stft,imgSize);
    genSplitSimple(outRoot,'test', classes,numTest, Ns,SNRdB_list,JSR_dB_list,stft,imgSize);
end
