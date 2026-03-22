function train_lenet_stft()
% ============================================================
% train_lenet_stft.m
% 目的：使用 LeNet 训练 STFT 图像干扰分类 (4类)
% 数据目录:
%   dataset_stft/train/{none,tone,pbnj,mod}
%   dataset_stft/val/{none,tone,pbnj,mod}
%   dataset_stft/test/{none,tone,pbnj,mod}
% ============================================================

clear; clc; close all;

%% ============ 数据集路径 ============
dataRoot = 'dataset_stft';
trainDir = fullfile(dataRoot, 'train');
valDir   = fullfile(dataRoot, 'val');
testDir  = fullfile(dataRoot, 'test');

classes = {'none','tone','pbnj','mod'};

%% ============ 读取数据集 ============
imdsTrain = imageDatastore(trainDir, ...
    'IncludeSubfolders', true, ...
    'LabelSource', 'foldernames', ...
    'ReadFcn', @(x) im2single(im2gray(imread(x))) );

imdsVal = imageDatastore(valDir, ...
    'IncludeSubfolders', true, ...
    'LabelSource', 'foldernames', ...
    'ReadFcn', @(x) im2single(im2gray(imread(x))) );

imdsTest = imageDatastore(testDir, ...
    'IncludeSubfolders', true, ...
    'LabelSource', 'foldernames', ...
    'ReadFcn', @(x) im2single(im2gray(imread(x))) );

% 强制类别顺序一致（防止某个split缺类导致label顺序乱）
imdsTrain.Labels = reordercats(imdsTrain.Labels, classes);
imdsVal.Labels   = reordercats(imdsVal.Labels, classes);
imdsTest.Labels  = reordercats(imdsTest.Labels, classes);

fprintf("Train images: %d\n", numel(imdsTrain.Files));
fprintf("Val images  : %d\n", numel(imdsVal.Files));
fprintf("Test images : %d\n", numel(imdsTest.Files));

%% ============ 输入尺寸 ============
inputSize = [128 128 1];   % 你的图是灰度 128x128
numClasses = numel(classes);

%% ============ 预处理：保证尺寸一致 ============

augTrain = augmentedImageDatastore(inputSize(1:2), imdsTrain);
augVal   = augmentedImageDatastore(inputSize(1:2), imdsVal);
augTest  = augmentedImageDatastore(inputSize(1:2), imdsTest);

% 备注：如果你确定是单通道，可以直接：
% augmentedImageDatastore([128 128], imdsTrain)
% 但上面这种更稳一点（防止有的png是RGB）

%% ============ 定义 LeNet 网络结构 ============
layers = [
    imageInputLayer(inputSize, 'Name','input', 'Normalization','none')

    convolution2dLayer(5, 6, 'Padding','same', 'Name','conv1')
    reluLayer('Name','relu1')
    averagePooling2dLayer(2, 'Stride',2, 'Name','pool1')

    convolution2dLayer(5, 16, 'Name','conv2')
    reluLayer('Name','relu2')
    averagePooling2dLayer(2, 'Stride',2, 'Name','pool2')

    convolution2dLayer(5, 120, 'Name','conv3')
    reluLayer('Name','relu3')

    fullyConnectedLayer(84, 'Name','fc1')
    reluLayer('Name','relu4')

    fullyConnectedLayer(numClasses, 'Name','fc2')
    softmaxLayer('Name','softmax')
    classificationLayer('Name','output')
];

analyzeNetwork(layers);   % 可视化网络（可注释）

%% ============ 训练参数 ============
miniBatchSize = 64;
maxEpochs = 15;
initLearnRate = 1e-3;

opts = trainingOptions('adam', ...
    'InitialLearnRate', initLearnRate, ...
    'MaxEpochs', maxEpochs, ...
    'MiniBatchSize', miniBatchSize, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', augVal, ...
    'ValidationFrequency', 50, ...
    'Verbose', true, ...
    'Plots', 'training-progress');

%% ============ 开始训练 ============
fprintf("\n==== Training LeNet ====\n");
net = trainNetwork(augTrain, layers, opts);

%% ============ 验证集评估 ============
fprintf("\n==== Validation Evaluation ====\n");
YPred_val = classify(net, augVal);
YTrue_val = imdsVal.Labels;
valAcc = mean(YPred_val == YTrue_val);
fprintf("Validation Accuracy = %.2f %%\n", valAcc*100);

figure;
confusionchart(YTrue_val, YPred_val);
title(sprintf("Validation Confusion Matrix (Acc=%.2f%%)", valAcc*100));

%% ============ 测试集评估 ============
fprintf("\n==== Test Evaluation ====\n");
YPred_test = classify(net, augTest);
YTrue_test = imdsTest.Labels;
testAcc = mean(YPred_test == YTrue_test);
fprintf("Test Accuracy = %.2f %%\n", testAcc*100);

figure;
confusionchart(YTrue_test, YPred_test);
title(sprintf("Test Confusion Matrix (Acc=%.2f%%)", testAcc*100));

%% ============ 保存模型 ============
save('lenet_stft_model.mat', 'net', 'classes');
fprintf("\n✅ Model saved: lenet_stft_model.mat\n");

end
