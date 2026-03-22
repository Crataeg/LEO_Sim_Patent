function cfg = emcLaunchConfigUI(defaultCfg)
%EMCLAUNCHCONFIGUI Startup config selector.
%   Closed dialog / invalid input -> default config.

    cfg = defaultCfg;

    try
        choice = questdlg( ...
            sprintf(['V7 工程交付版启动方式：\n\n' ...
                     '1) 直接使用默认参数运行\n' ...
                     '2) 打开参数配置页面\n' ...
                     '3) 从 MAT 文件加载配置\n\n' ...
                     '若关闭对话框，将自动按默认参数运行。']), ...
            'LEO StarNet EMC V7 启动', ...
            '默认参数直接运行', '打开参数配置页', '加载MAT配置', ...
            '默认参数直接运行');
    catch
        choice = '默认参数直接运行';
    end

    if isempty(choice)
        choice = '默认参数直接运行';
    end

    switch choice
        case '打开参数配置页'
            cfgTmp = emcConfigUI(defaultCfg);
            cfg = emcMergeStruct(defaultCfg, cfgTmp);
            cfg.General.StartupMode = 'custom-ui';

        case '加载MAT配置'
            cfgLoaded = [];
            try
                [fn, fp] = uigetfile('*.mat', '选择配置MAT文件');
                if isequal(fn,0)
                    cfgLoaded = defaultCfg;
                else
                    S = load(fullfile(fp, fn));
                    if isfield(S, 'cfg') && isstruct(S.cfg)
                        cfgLoaded = S.cfg;
                    else
                        f = fieldnames(S);
                        for i = 1:numel(f)
                            if isstruct(S.(f{i}))
                                cfgLoaded = S.(f{i});
                                break;
                            end
                        end
                    end
                end
            catch
                cfgLoaded = defaultCfg;
            end
            cfg = emcMergeStruct(defaultCfg, cfgLoaded);
            cfg.General.StartupMode = 'load-mat';

        otherwise
            cfg = defaultCfg;
            cfg.General.StartupMode = 'default';
    end
end
