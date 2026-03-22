classdef HelperLMSChannel_R21a < matlab.System
    % HelperLMSChannel_R21a: 适用于 MATLAB R2021a 的自定义 LMS 信道模型
    % 实现 ITU-R P.681 双状态马尔可夫衰落模型
    % 状态 1: Good (Rician)
    % 状态 2: Bad (Rayleigh + LogNormal Shadowing)

    properties (Nontunable)
        SampleRate = 15.36e6;
        MaxDoppler = 100; % 最大多普勒频移 (Hz)
    end

    properties (Access = private)
        RayleighFilter
        RicianFilter
        CurrentState
        ShadowingGen
    end

    methods (Access = protected)
        function setupImpl(obj)
            % 初始化内置衰落滤波器
            obj.RayleighFilter = comm.RayleighChannel(...
                'SampleRate', obj.SampleRate,...
                'MaximumDopplerShift', obj.MaxDoppler,...
                'PathGainsOutputPort', true);
            
            obj.RicianFilter = comm.RicianChannel(...
                'SampleRate', obj.SampleRate,...
                'KFactor', 10,... % Good状态下的K因子 (可设为变量)
                'MaximumDopplerShift', obj.MaxDoppler,...
                'PathGainsOutputPort', true);
            
            obj.CurrentState = 1; % 初始状态 1 (Good)
            
            % 初始化阴影生成器 (对数正态分布参数 mu, sigma)
            % 这里简化处理，实际应根据仰角动态调整
            obj.ShadowingGen = dsp.RandomSource(...
                'Distribution', 'Gaussian',...
                'Mean', -10,... % 平均衰减 -10dB
                'Variance', 4);  % 方差
        end

        function out = stepImpl(obj, in)
            % 1. 马尔可夫状态转移逻辑
            % 定义转移概率矩阵 P = [Pgg Pgb; Pbg Pbb]
            % 示例值：保持在当前状态的概率较高
            transProbs = [0.999 0.001; 0.005 0.995]; 
            
            % 生成随机数判断是否跳转
            if rand < transProbs(obj.CurrentState, 2)
                obj.CurrentState = 3 - obj.CurrentState; % 切换状态 (1->2 或 2->1)
            end

            % 2. 应用衰落
            if obj.CurrentState == 1 % Good State
                [out, ~] = obj.RicianFilter(in);
                % 虚拟消耗Rayleigh滤波器的状态以保持同步（可选）
                obj.RayleighFilter(in); 
            else % Bad State (Shadowed)
                [faded, ~] = obj.RayleighFilter(in);
                % 生成阴影因子 (dB -> Linear)
                shadowing_dB = obj.ShadowingGen();
                shadowing_lin = 10^(shadowing_dB/20);
                out = faded * shadowing_lin;
                
                % 虚拟消耗Rician
                obj.RicianFilter(in);
            end
        end

        function resetImpl(obj)
            reset(obj.RayleighFilter);
            reset(obj.RicianFilter);
            reset(obj.ShadowingGen);
            obj.CurrentState = 1;
        end
    end
end