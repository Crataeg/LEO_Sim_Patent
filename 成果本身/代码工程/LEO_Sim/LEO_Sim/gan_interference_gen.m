%% gan_interference_gen.m
%  功能：加载预训练GAN网络并生成干扰波形
%  兼容性：Deep Learning Toolbox R2021a

% 1. 定义生成器网络架构 (示例架构)
% 在实际使用中，应通过 load('my_trained_gan.mat') 加载
layers = [
    featureInputLayer(100, 'Name', 'Noise_Input', 'Normalization', 'none')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(1024, 'Name', 'fc2') % 输出层对应IQ采样点数
    regressionLayer('Name', 'out')];

lgraph = layerGraph(layers);
dlnet = dlnetwork(lgraph); % R2021a 支持 dlnetwork

% 2. 干扰生成函数 (供Simulink MATLAB Function调用)
function iq_interference = generate_gan_jamming(dlnet_obj)
    % 生成随机噪声向量 (Latent Vector)
    Z = dlarray(randn(100,1), 'CB');
    
    % 前向推理生成干扰数据
    % R2021a 中使用 predict 函数
    X_generated = predict(dlnet_obj, Z);
    
    % 转换格式
    data = double(extractdata(X_generated));
    
    % 构建复数信号
    % 假设输出的前半部分为I路，后半部分为Q路
    len = length(data)/2;
    iq_interference = complex(data(1:len), data(len+1:end));
end