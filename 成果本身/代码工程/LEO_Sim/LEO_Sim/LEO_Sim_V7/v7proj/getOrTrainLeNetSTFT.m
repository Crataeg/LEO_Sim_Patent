function [net, classes] = getOrTrainLeNetSTFT(modelFile, datasetRoot, trainIfMissing)
    classes = {'none','tone','pbnj','mod'};
    if exist(modelFile,'file')
        S = load(modelFile);
        net = S.net;
        if isfield(S,'classes'), classes = S.classes; end
        return;
    end
    if ~trainIfMissing
        error('Interference classifier model not found: %s', modelFile);
    end
 
    if ~exist(datasetRoot,'dir')
        fprintf('  [IntfCls] Dataset not found, generating dataset: %s\n', datasetRoot);
        generateDatasetSimpleSTFT(datasetRoot);
    end
    fprintf('  [IntfCls] Training LeNet on STFT images...\n');
    [net, classes] = trainLeNetSTFT(datasetRoot, classes);
    save(modelFile,'net','classes');
    fprintf('  [IntfCls] Saved: %s\n', modelFile);
end
