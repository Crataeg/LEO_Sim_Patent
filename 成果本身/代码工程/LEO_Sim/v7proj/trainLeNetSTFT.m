function [net, classes] = trainLeNetSTFT(dataRoot, classes)
    trainDir = fullfile(dataRoot,'train');
    valDir   = fullfile(dataRoot,'val');
 
    inputSize=[128 128 1];
 
    % ReadFcn：你已装 Image Processing Toolbox，所以这里可直接用 im2single/im2gray
    imdsTrain = imageDatastore(trainDir,'IncludeSubfolders',true,'LabelSource','foldernames', ...
        'ReadFcn', @(x) im2single(im2gray(imread(x))));
    imdsVal   = imageDatastore(valDir,  'IncludeSubfolders',true,'LabelSource','foldernames', ...
        'ReadFcn', @(x) im2single(im2gray(imread(x))));
 
    imdsTrain.Labels = reordercats(imdsTrain.Labels, classes);
    imdsVal.Labels   = reordercats(imdsVal.Labels, classes);
 
    augTrain = augmentedImageDatastore(inputSize(1:2), imdsTrain);
    augVal   = augmentedImageDatastore(inputSize(1:2), imdsVal);
 
    layers = [
        imageInputLayer(inputSize,'Normalization','none','Name','in')
        convolution2dLayer(5,6,'Padding','same','Name','c1')
        reluLayer('Name','r1')
        averagePooling2dLayer(2,'Stride',2,'Name','p1')
        convolution2dLayer(5,16,'Name','c2')
        reluLayer('Name','r2')
        averagePooling2dLayer(2,'Stride',2,'Name','p2')
        convolution2dLayer(5,120,'Name','c3')
        reluLayer('Name','r3')
        fullyConnectedLayer(84,'Name','fc1')
        reluLayer('Name','r4')
        fullyConnectedLayer(numel(classes),'Name','fc2')
        softmaxLayer('Name','sm')
        classificationLayer('Name','out') ];
 
    opts = trainingOptions('adam', ...
        'InitialLearnRate',1e-3, ...
        'MaxEpochs',8, ...
        'MiniBatchSize',64, ...
        'Shuffle','every-epoch', ...
        'ValidationData',augVal, ...
        'ValidationFrequency',50, ...
        'Verbose',true);
 
    net = trainNetwork(augTrain, layers, opts);
end
