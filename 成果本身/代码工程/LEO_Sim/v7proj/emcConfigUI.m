function cfgOut = emcConfigUI(cfgIn)
%EMCCONFIGUI Engineering-style parameter page for V7.
%   Cancel / close -> return cfgIn.

    defCfg = emcDefaultConfig();
    cfg0 = emcMergeStruct(defCfg, cfgIn);
    cfgOut = cfg0;

    fig = uifigure('Name','LEO StarNet EMC V7 参数配置', ...
        'Position',[80 40 1320 860], 'Color','w');
    fig.CloseRequestFcn = @onCancel;

    setappdata(fig, 'cfgOut', cfg0);
    setappdata(fig, 'confirmed', false);

    gl = uigridlayout(fig, [3 1]);
    gl.RowHeight = {44, '1x', 44};
    gl.ColumnWidth = {'1x'};
    gl.Padding = [10 10 10 10];

    titleLbl = uilabel(gl, 'Text', ...
        'V7 工程交付版参数页 | 支持默认值、手工配置、加载/保存 MAT 配置', ...
        'FontSize',16, 'FontWeight','bold');
    titleLbl.Layout.Row = 1;

    tg = uitabgroup(gl);
    tg.Layout.Row = 2;

    % ===== Tab 1: scenario =====
    tab1 = uitab(tg, 'Title', '场景/星座');
    g1 = uigridlayout(tab1, [14 2]);
    g1.RowHeight = repmat({'fit'},1,14);
    g1.ColumnWidth = {260, '1x'};
    c = struct();
    c.Time_Epoch = addText(g1,1,'Epoch (UTC, yyyy-mm-dd HH:MM:SS)', char(cfg0.Time.Epoch));
    c.Time_Sample = addNumeric(g1,2,'采样时间 SampleTime (s)', cfg0.Time.SampleTime_s);
    c.Time_Duration = addNumeric(g1,3,'仿真时长 SimDuration (s, 0=一轨)', cfg0.Time.SimDuration_s);
    c.Const_Alt = addNumeric(g1,4,'轨道高度 Altitude (m)', cfg0.Constellation.Altitude_m);
    c.Const_Inc = addNumeric(g1,5,'轨道倾角 Inclination (deg)', cfg0.Constellation.Inclination_deg);
    c.Const_Planes = addNumeric(g1,6,'轨道面数 NumPlanes', cfg0.Constellation.NumPlanes);
    c.Const_SatsPerPlane = addNumeric(g1,7,'每面星数 SatsPerPlane', cfg0.Constellation.SatsPerPlane);
    c.Const_F = addNumeric(g1,8,'Walker 相位因子 F', cfg0.Constellation.FPhasing);
    c.Const_Reuse = addNumeric(g1,9,'频率复用因子 ReuseK', cfg0.Constellation.ReuseK);
    c.Const_ElMask = addNumeric(g1,10,'最小仰角 ElMask (deg)', cfg0.Constellation.ElMask_deg);
    c.Ground_UserLat = addNumeric(g1,11,'用户端纬度 UserLat', cfg0.Ground.UserLat);
    c.Ground_UserLon = addNumeric(g1,12,'用户端经度 UserLon', cfg0.Ground.UserLon);
    c.Ground_GWLat = addNumeric(g1,13,'网关纬度 GWLat', cfg0.Ground.GWLat);
    c.Ground_GWLon = addNumeric(g1,14,'网关经度 GWLon', cfg0.Ground.GWLon);

    % ===== Tab 2: downlink =====
    tab2 = uitab(tg, 'Title', '下行链路');
    g2 = uigridlayout(tab2, [8 2]);
    g2.RowHeight = repmat({'fit'},1,8);
    g2.ColumnWidth = {260, '1x'};
    c.DL_Fc = addNumeric(g2,1,'下行中心频率 Fc_DL (Hz)', cfg0.Downlink.Fc_Hz);
    c.DL_BW = addNumeric(g2,2,'下行带宽 BW_DL (Hz)', cfg0.Downlink.BW_Hz);
    c.DL_Noise = addNumeric(g2,3,'下行噪声功率 Noise_DL (dBm)', cfg0.Downlink.Noise_dBm);
    c.DL_TxS = addNumeric(g2,4,'下行有用信号 EIRP_S (dBm)', cfg0.Downlink.TxEIRP_S_dBm);
    c.DL_TxI = addNumeric(g2,5,'下行同频干扰 EIRP_I (dBm)', cfg0.Downlink.TxEIRP_I_dBm);
    c.DL_RxGain = addNumeric(g2,6,'用户端接收增益 RxGain (dB)', cfg0.Downlink.RxGain_dB);
    c.DL_IntPenalty = addNumeric(g2,7,'同频干扰惩罚 InterfPenalty (dB)', cfg0.Downlink.InterfPenalty_dB);
    addNote(g2,8,'说明：下行用于现有 V6 框架主链路，保持兼容。');

    % ===== Tab 3: uplink =====
    tab3 = uitab(tg, 'Title', '上行链路');
    g3 = uigridlayout(tab3, [11 2]);
    g3.RowHeight = repmat({'fit'},1,11);
    g3.ColumnWidth = {300, '1x'};
    c.UL_Fc = addNumeric(g3,1,'上行中心频率 Fc_UL (Hz)', cfg0.Uplink.Fc_Hz);
    c.UL_BW = addNumeric(g3,2,'上行带宽 BW_UL (Hz)', cfg0.Uplink.BW_Hz);
    c.UL_Noise = addNumeric(g3,3,'上行噪声功率 Noise_UL (dBm)', cfg0.Uplink.Noise_dBm);
    c.UL_TxS = addNumeric(g3,4,'上行有用信号 EIRP_S (dBm)', cfg0.Uplink.TxEIRP_S_dBm);
    c.UL_TxI = addNumeric(g3,5,'上行同频干扰 EIRP_I (dBm)', cfg0.Uplink.TxEIRP_I_dBm);
    c.UL_RxGain = addNumeric(g3,6,'卫星端接收增益 RxGain (dB)', cfg0.Uplink.RxGain_dB);
    c.UL_IntPenalty = addNumeric(g3,7,'上行同频干扰惩罚 InterfPenalty (dB)', cfg0.Uplink.InterfPenalty_dB);
    c.UL_CCIMode = addDrop(g3,8,'上行 CCI 模式', {'none','fixed','reuseProxy'}, cfg0.Uplink.CCI_Mode);
    c.UL_CCIFixed = addNumeric(g3,9,'上行固定 CCI (dBm)', cfg0.Uplink.CCI_Fixed_dBm);
    c.UL_JamMode = addDrop(g3,10,'上行 Jammer 代理模式', {'off','fixed','reuseDL'}, cfg0.Uplink.JamProxyMode);
    c.UL_JamFixed = addNumeric(g3,11,'上行固定 Jam 基准功率 (dBm)', cfg0.Uplink.JamProxyFixed_dBm);

    % ===== Tab 4: jammer / search =====
    tab4 = uitab(tg, 'Title', '干扰/最劣搜索');
    g4 = uigridlayout(tab4, [15 2]);
    g4.RowHeight = repmat({'fit'},1,15);
    g4.ColumnWidth = {320, '1x'};
    c.Jam_Num = addNumeric(g4,1,'Jammer 数量', cfg0.Jammer.NumJammers);
    c.Jam_TxBase = addNumeric(g4,2,'Jammer 基准 EIRP (dBm)', cfg0.Jammer.TxEIRP_Base_dBm);
    c.Jam_RxGain = addNumeric(g4,3,'Jammer 接收增益/耦合增益 (dB)', cfg0.Jammer.RxGain_dB);
    c.Jam_MainDeg = addNumeric(g4,4,'主瓣角 MainLobe (deg)', cfg0.Jammer.MainLobe_deg);
    c.Jam_SideDeg = addNumeric(g4,5,'旁瓣角 SideLobe (deg)', cfg0.Jammer.SideLobe_deg);
    c.Jam_MainGain = addNumeric(g4,6,'主瓣增益 MainGain (dB)', cfg0.Jammer.MainGain_dB);
    c.Jam_SideGain = addNumeric(g4,7,'旁瓣增益 SideGain (dB)', cfg0.Jammer.SideGain_dB);
    c.Jam_FloorGain = addNumeric(g4,8,'底噪增益 FloorGain (dB)', cfg0.Jammer.FloorGain_dB);
    c.AJ_Delay = addNumeric(g4,9,'抗干扰启用延时 Delay (s)', cfg0.AntiJam.Delay_s);
    c.AJ_Null = addNumeric(g4,10,'抗干扰零陷深度 NullDepth (dB)', cfg0.AntiJam.NullDepth_dB);
    c.WC_Enable = addCheck(g4,11,'启用最劣搜索 Worst-case Search', cfg0.WorstCase.Enable);
    c.WC_Target = addDrop(g4,12,'最劣搜索目标', {'downlink','uplink','e2e'}, cfg0.WorstCase.Target);
    c.WC_GAPop = addNumeric(g4,13,'GA PopulationSize', cfg0.WorstCase.GA_PopSize);
    c.WC_GAGen = addNumeric(g4,14,'GA MaxGenerations', cfg0.WorstCase.GA_Generations);
    c.WC_JamBias = addNumeric(g4,15,'上行复用 DL Jam 偏置 (dB)', cfg0.Uplink.JamReuseBias_dB);

    % ===== Tab 5: requirements / output =====
    tab5 = uitab(tg, 'Title', '指标/输出');
    g5 = uigridlayout(tab5, [18 2]);
    g5.RowHeight = repmat({'fit'},1,18);
    g5.ColumnWidth = {340, '1x'};
    c.Req_MinSINR = addNumeric(g5,1,'链路信噪比门限 MinSINR (dB)', cfg0.Requirements.MinSINR_dB);
    c.Req_MinThr = addNumeric(g5,2,'宽带速率门限 MinThr (Mbps)', cfg0.Requirements.MinThr_Mbps);
    c.Req_Sens = addNumeric(g5,3,'接收灵敏度门限 RxSensitivity (dBm)', cfg0.Requirements.RxSensitivity_dBm);
    c.Req_EnableStrength = addCheck(g5,4,'启用“转换后信号强度”检查', cfg0.Requirements.EnableConvertedStrengthCheck);
    c.Req_MinStrength = addNumeric(g5,5,'信号强度门限 MinSignalStrength (dBm)', cfg0.Requirements.MinSignalStrength_dBm);
    c.Req_StrengthOffset = addNumeric(g5,6,'信号强度换算偏置 Offset (dB)', cfg0.Requirements.SignalStrengthOffset_dB);
    c.Req_MaxDopRate = addNumeric(g5,7,'多普勒变化率上限 MaxDopplerRate (Hz/s)', cfg0.Requirements.MaxDopplerRate_Hzps);
    c.Req_KuEIRP_Cur = addNumeric(g5,8,'Ku EIRP 当前值 (dBw)', cfg0.Requirements.KuEIRP_Current_dBw);
    c.Req_KuEIRP_Min = addNumeric(g5,9,'Ku EIRP 门限 (dBw)', cfg0.Requirements.KuEIRP_Min_dBw);
    c.Req_KuGT_Cur = addNumeric(g5,10,'Ku G/T 当前值 (dB/K)', cfg0.Requirements.KuGT_Current_dBperK);
    c.Req_KuGT_Min = addNumeric(g5,11,'Ku G/T 门限 (dB/K)', cfg0.Requirements.KuGT_Min_dBperK);
    c.Req_JA_Target = addNumeric(g5,12,'JA 3700-MH-3-2022 目标等级', cfg0.Requirements.JA3700_TargetLevel);
    c.Req_JA_Current = addNumeric(g5,13,'JA 3700-MH-3-2022 当前等级', cfg0.Requirements.JA3700_CurrentLevel);
    c.Cls_Enable = addCheck(g5,14,'启用 STFT+LeNet 分类', cfg0.Classifier.Enable);
    c.Out_3D = addCheck(g5,15,'启用 3D Viewer 联动', cfg0.Output.Enable3DViewer);
    c.Out_Speed = addNumeric(g5,16,'3D Viewer 默认播放倍速', cfg0.Display.ViewerSpeed);
    c.Out_Folder = addText(g5,17,'输出目录 ExportFolder', cfg0.Output.ExportFolder);
    addNote(g5,18,'说明：若不提供 AF/线损/转换系数，默认关闭“转换后信号强度”合规检查。');

    % ===== bottom buttons =====
    btm = uigridlayout(gl, [1 6]);
    btm.Layout.Row = 3;
    btm.ColumnWidth = {'1x','1x','1x','1x','1x','1x'};
    btm.Padding = [0 0 0 0];

    btnDefault = uibutton(btm, 'Text', '恢复默认值', 'ButtonPushedFcn', @onDefault);
    btnLoad    = uibutton(btm, 'Text', '加载MAT配置', 'ButtonPushedFcn', @onLoad);
    btnSave    = uibutton(btm, 'Text', '保存当前配置', 'ButtonPushedFcn', @onSave);
    btnCancel  = uibutton(btm, 'Text', '取消', 'ButtonPushedFcn', @onCancel);
    btnOK      = uibutton(btm, 'Text', '确认并运行', 'ButtonPushedFcn', @onConfirm);
    btnHint    = uibutton(btm, 'Text', '说明', 'ButtonPushedFcn', @onHint);
    btnDefault.Layout.Column = 1;
    btnLoad.Layout.Column = 2;
    btnSave.Layout.Column = 3;
    btnCancel.Layout.Column = 4;
    btnOK.Layout.Column = 5;
    btnHint.Layout.Column = 6;

    uiwait(fig);

    if isvalid(fig)
        confirmed = getappdata(fig, 'confirmed');
        cfgTmp = getappdata(fig, 'cfgOut');
        if confirmed
            cfgOut = cfgTmp;
        else
            cfgOut = cfg0;
        end
        delete(fig);
    else
        cfgOut = cfg0;
    end

    % ===== nested callbacks =====
    function onDefault(~, ~)
        fillControls(defCfg);
    end

    function onLoad(~, ~)
        try
            [fn, fp] = uigetfile('*.mat', '选择配置MAT文件');
            if isequal(fn,0)
                return;
            end
            S = load(fullfile(fp, fn));
            cfgL = [];
            if isfield(S, 'cfg') && isstruct(S.cfg)
                cfgL = S.cfg;
            else
                f = fieldnames(S);
                for ii = 1:numel(f)
                    if isstruct(S.(f{ii}))
                        cfgL = S.(f{ii});
                        break;
                    end
                end
            end
            if isempty(cfgL)
                uialert(fig, 'MAT 文件中未找到 cfg 结构体。', '加载失败');
                return;
            end
            fillControls(emcMergeStruct(defCfg, cfgL));
        catch ME
            uialert(fig, ME.message, '加载失败');
        end
    end

    function onSave(~, ~)
        try
            cfgS = readControls();
            [fn, fp] = uiputfile('*.mat', '保存配置为 MAT', 'cfg_v7.mat');
            if isequal(fn,0)
                return;
            end
            cfg = cfgS; %#ok<NASGU>
            save(fullfile(fp, fn), 'cfg');
        catch ME
            uialert(fig, ME.message, '保存失败');
        end
    end

    function onConfirm(~, ~)
        try
            cfgTmp = readControls();
            setappdata(fig, 'cfgOut', cfgTmp);
            setappdata(fig, 'confirmed', true);
            uiresume(fig);
        catch ME
            uialert(fig, ME.message, '参数错误');
        end
    end

    function onCancel(~, ~)
        if isvalid(fig)
            setappdata(fig, 'cfgOut', cfg0);
            setappdata(fig, 'confirmed', false);
            uiresume(fig);
        end
    end

    function onHint(~, ~)
        msg = sprintf(['使用建议：\n\n' ...
            '1. 首次运行可直接点击“确认并运行”，保留默认值。\n' ...
            '2. 若对接方后续提供接收机端口功率、天线参数、AF、线损、转换系数，' ...
            '优先修改“指标/输出”页中的强度换算参数。\n' ...
            '3. 上行 CCI 建议先用 fixed 模式，待后期具备更完整网络负载模型后再切换为 reuseProxy。\n' ...
            '4. 最劣搜索目标建议默认选择 e2e，以端到端吞吐最小值作为设计牵引。']);
        uialert(fig, msg, '说明');
    end

    % ===== nested helpers =====
    function fillControls(cfgX)
        c.Time_Epoch.Value = char(cfgX.Time.Epoch);
        c.Time_Sample.Value = cfgX.Time.SampleTime_s;
        c.Time_Duration.Value = cfgX.Time.SimDuration_s;
        c.Const_Alt.Value = cfgX.Constellation.Altitude_m;
        c.Const_Inc.Value = cfgX.Constellation.Inclination_deg;
        c.Const_Planes.Value = cfgX.Constellation.NumPlanes;
        c.Const_SatsPerPlane.Value = cfgX.Constellation.SatsPerPlane;
        c.Const_F.Value = cfgX.Constellation.FPhasing;
        c.Const_Reuse.Value = cfgX.Constellation.ReuseK;
        c.Const_ElMask.Value = cfgX.Constellation.ElMask_deg;
        c.Ground_UserLat.Value = cfgX.Ground.UserLat;
        c.Ground_UserLon.Value = cfgX.Ground.UserLon;
        c.Ground_GWLat.Value = cfgX.Ground.GWLat;
        c.Ground_GWLon.Value = cfgX.Ground.GWLon;

        c.DL_Fc.Value = cfgX.Downlink.Fc_Hz;
        c.DL_BW.Value = cfgX.Downlink.BW_Hz;
        c.DL_Noise.Value = cfgX.Downlink.Noise_dBm;
        c.DL_TxS.Value = cfgX.Downlink.TxEIRP_S_dBm;
        c.DL_TxI.Value = cfgX.Downlink.TxEIRP_I_dBm;
        c.DL_RxGain.Value = cfgX.Downlink.RxGain_dB;
        c.DL_IntPenalty.Value = cfgX.Downlink.InterfPenalty_dB;

        c.UL_Fc.Value = cfgX.Uplink.Fc_Hz;
        c.UL_BW.Value = cfgX.Uplink.BW_Hz;
        c.UL_Noise.Value = cfgX.Uplink.Noise_dBm;
        c.UL_TxS.Value = cfgX.Uplink.TxEIRP_S_dBm;
        c.UL_TxI.Value = cfgX.Uplink.TxEIRP_I_dBm;
        c.UL_RxGain.Value = cfgX.Uplink.RxGain_dB;
        c.UL_IntPenalty.Value = cfgX.Uplink.InterfPenalty_dB;
        c.UL_CCIMode.Value = cfgX.Uplink.CCI_Mode;
        c.UL_CCIFixed.Value = cfgX.Uplink.CCI_Fixed_dBm;
        c.UL_JamMode.Value = cfgX.Uplink.JamProxyMode;
        c.UL_JamFixed.Value = cfgX.Uplink.JamProxyFixed_dBm;

        c.Jam_Num.Value = cfgX.Jammer.NumJammers;
        c.Jam_TxBase.Value = cfgX.Jammer.TxEIRP_Base_dBm;
        c.Jam_RxGain.Value = cfgX.Jammer.RxGain_dB;
        c.Jam_MainDeg.Value = cfgX.Jammer.MainLobe_deg;
        c.Jam_SideDeg.Value = cfgX.Jammer.SideLobe_deg;
        c.Jam_MainGain.Value = cfgX.Jammer.MainGain_dB;
        c.Jam_SideGain.Value = cfgX.Jammer.SideGain_dB;
        c.Jam_FloorGain.Value = cfgX.Jammer.FloorGain_dB;
        c.AJ_Delay.Value = cfgX.AntiJam.Delay_s;
        c.AJ_Null.Value = cfgX.AntiJam.NullDepth_dB;
        c.WC_Enable.Value = logical(cfgX.WorstCase.Enable);
        c.WC_Target.Value = cfgX.WorstCase.Target;
        c.WC_GAPop.Value = cfgX.WorstCase.GA_PopSize;
        c.WC_GAGen.Value = cfgX.WorstCase.GA_Generations;
        c.WC_JamBias.Value = cfgX.Uplink.JamReuseBias_dB;

        c.Req_MinSINR.Value = cfgX.Requirements.MinSINR_dB;
        c.Req_MinThr.Value = cfgX.Requirements.MinThr_Mbps;
        c.Req_Sens.Value = cfgX.Requirements.RxSensitivity_dBm;
        c.Req_EnableStrength.Value = logical(cfgX.Requirements.EnableConvertedStrengthCheck);
        c.Req_MinStrength.Value = cfgX.Requirements.MinSignalStrength_dBm;
        c.Req_StrengthOffset.Value = cfgX.Requirements.SignalStrengthOffset_dB;
        c.Req_MaxDopRate.Value = cfgX.Requirements.MaxDopplerRate_Hzps;
        c.Req_KuEIRP_Cur.Value = cfgX.Requirements.KuEIRP_Current_dBw;
        c.Req_KuEIRP_Min.Value = cfgX.Requirements.KuEIRP_Min_dBw;
        c.Req_KuGT_Cur.Value = cfgX.Requirements.KuGT_Current_dBperK;
        c.Req_KuGT_Min.Value = cfgX.Requirements.KuGT_Min_dBperK;
        c.Req_JA_Target.Value = cfgX.Requirements.JA3700_TargetLevel;
        c.Req_JA_Current.Value = cfgX.Requirements.JA3700_CurrentLevel;
        c.Cls_Enable.Value = logical(cfgX.Classifier.Enable);
        c.Out_3D.Value = logical(cfgX.Output.Enable3DViewer);
        c.Out_Speed.Value = cfgX.Display.ViewerSpeed;
        c.Out_Folder.Value = cfgX.Output.ExportFolder;
    end

    function cfgX = readControls()
        cfgX = cfg0;

        cfgX.Time.Epoch = safeDatetime(c.Time_Epoch.Value, cfg0.Time.Epoch);
        cfgX.Time.SampleTime_s = safeNum(c.Time_Sample.Value, cfg0.Time.SampleTime_s);
        cfgX.Time.SimDuration_s = safeNum(c.Time_Duration.Value, cfg0.Time.SimDuration_s);

        cfgX.Constellation.Altitude_m = safeNum(c.Const_Alt.Value, cfg0.Constellation.Altitude_m);
        cfgX.Constellation.Inclination_deg = safeNum(c.Const_Inc.Value, cfg0.Constellation.Inclination_deg);
        cfgX.Constellation.NumPlanes = round(safeNum(c.Const_Planes.Value, cfg0.Constellation.NumPlanes));
        cfgX.Constellation.SatsPerPlane = round(safeNum(c.Const_SatsPerPlane.Value, cfg0.Constellation.SatsPerPlane));
        cfgX.Constellation.FPhasing = round(safeNum(c.Const_F.Value, cfg0.Constellation.FPhasing));
        cfgX.Constellation.ReuseK = round(safeNum(c.Const_Reuse.Value, cfg0.Constellation.ReuseK));
        cfgX.Constellation.ElMask_deg = safeNum(c.Const_ElMask.Value, cfg0.Constellation.ElMask_deg);

        cfgX.Ground.UserLat = safeNum(c.Ground_UserLat.Value, cfg0.Ground.UserLat);
        cfgX.Ground.UserLon = safeNum(c.Ground_UserLon.Value, cfg0.Ground.UserLon);
        cfgX.Ground.GWLat = safeNum(c.Ground_GWLat.Value, cfg0.Ground.GWLat);
        cfgX.Ground.GWLon = safeNum(c.Ground_GWLon.Value, cfg0.Ground.GWLon);

        cfgX.Downlink.Fc_Hz = safeNum(c.DL_Fc.Value, cfg0.Downlink.Fc_Hz);
        cfgX.Downlink.BW_Hz = safeNum(c.DL_BW.Value, cfg0.Downlink.BW_Hz);
        cfgX.Downlink.Noise_dBm = safeNum(c.DL_Noise.Value, cfg0.Downlink.Noise_dBm);
        cfgX.Downlink.TxEIRP_S_dBm = safeNum(c.DL_TxS.Value, cfg0.Downlink.TxEIRP_S_dBm);
        cfgX.Downlink.TxEIRP_I_dBm = safeNum(c.DL_TxI.Value, cfg0.Downlink.TxEIRP_I_dBm);
        cfgX.Downlink.RxGain_dB = safeNum(c.DL_RxGain.Value, cfg0.Downlink.RxGain_dB);
        cfgX.Downlink.InterfPenalty_dB = safeNum(c.DL_IntPenalty.Value, cfg0.Downlink.InterfPenalty_dB);

        cfgX.Uplink.Fc_Hz = safeNum(c.UL_Fc.Value, cfg0.Uplink.Fc_Hz);
        cfgX.Uplink.BW_Hz = safeNum(c.UL_BW.Value, cfg0.Uplink.BW_Hz);
        cfgX.Uplink.Noise_dBm = safeNum(c.UL_Noise.Value, cfg0.Uplink.Noise_dBm);
        cfgX.Uplink.TxEIRP_S_dBm = safeNum(c.UL_TxS.Value, cfg0.Uplink.TxEIRP_S_dBm);
        cfgX.Uplink.TxEIRP_I_dBm = safeNum(c.UL_TxI.Value, cfg0.Uplink.TxEIRP_I_dBm);
        cfgX.Uplink.RxGain_dB = safeNum(c.UL_RxGain.Value, cfg0.Uplink.RxGain_dB);
        cfgX.Uplink.InterfPenalty_dB = safeNum(c.UL_IntPenalty.Value, cfg0.Uplink.InterfPenalty_dB);
        cfgX.Uplink.CCI_Mode = char(c.UL_CCIMode.Value);
        cfgX.Uplink.CCI_Fixed_dBm = safeNum(c.UL_CCIFixed.Value, cfg0.Uplink.CCI_Fixed_dBm);
        cfgX.Uplink.JamProxyMode = char(c.UL_JamMode.Value);
        cfgX.Uplink.JamProxyFixed_dBm = safeNum(c.UL_JamFixed.Value, cfg0.Uplink.JamProxyFixed_dBm);

        cfgX.Jammer.NumJammers = round(safeNum(c.Jam_Num.Value, cfg0.Jammer.NumJammers));
        cfgX.Jammer.TxEIRP_Base_dBm = safeNum(c.Jam_TxBase.Value, cfg0.Jammer.TxEIRP_Base_dBm);
        cfgX.Jammer.RxGain_dB = safeNum(c.Jam_RxGain.Value, cfg0.Jammer.RxGain_dB);
        cfgX.Jammer.MainLobe_deg = safeNum(c.Jam_MainDeg.Value, cfg0.Jammer.MainLobe_deg);
        cfgX.Jammer.SideLobe_deg = safeNum(c.Jam_SideDeg.Value, cfg0.Jammer.SideLobe_deg);
        cfgX.Jammer.MainGain_dB = safeNum(c.Jam_MainGain.Value, cfg0.Jammer.MainGain_dB);
        cfgX.Jammer.SideGain_dB = safeNum(c.Jam_SideGain.Value, cfg0.Jammer.SideGain_dB);
        cfgX.Jammer.FloorGain_dB = safeNum(c.Jam_FloorGain.Value, cfg0.Jammer.FloorGain_dB);

        cfgX.AntiJam.Delay_s = safeNum(c.AJ_Delay.Value, cfg0.AntiJam.Delay_s);
        cfgX.AntiJam.NullDepth_dB = safeNum(c.AJ_Null.Value, cfg0.AntiJam.NullDepth_dB);

        cfgX.WorstCase.Enable = logical(c.WC_Enable.Value);
        cfgX.WorstCase.Target = char(c.WC_Target.Value);
        cfgX.WorstCase.GA_PopSize = round(safeNum(c.WC_GAPop.Value, cfg0.WorstCase.GA_PopSize));
        cfgX.WorstCase.GA_Generations = round(safeNum(c.WC_GAGen.Value, cfg0.WorstCase.GA_Generations));
        cfgX.Uplink.JamReuseBias_dB = safeNum(c.WC_JamBias.Value, cfg0.Uplink.JamReuseBias_dB);

        cfgX.Requirements.MinSINR_dB = safeNum(c.Req_MinSINR.Value, cfg0.Requirements.MinSINR_dB);
        cfgX.Requirements.MinThr_Mbps = safeNum(c.Req_MinThr.Value, cfg0.Requirements.MinThr_Mbps);
        cfgX.Requirements.RxSensitivity_dBm = safeNum(c.Req_Sens.Value, cfg0.Requirements.RxSensitivity_dBm);
        cfgX.Requirements.EnableConvertedStrengthCheck = logical(c.Req_EnableStrength.Value);
        cfgX.Requirements.MinSignalStrength_dBm = safeNum(c.Req_MinStrength.Value, cfg0.Requirements.MinSignalStrength_dBm);
        cfgX.Requirements.SignalStrengthOffset_dB = safeNum(c.Req_StrengthOffset.Value, cfg0.Requirements.SignalStrengthOffset_dB);
        cfgX.Requirements.MaxDopplerRate_Hzps = safeNum(c.Req_MaxDopRate.Value, cfg0.Requirements.MaxDopplerRate_Hzps);
        cfgX.Requirements.KuEIRP_Current_dBw = safeNum(c.Req_KuEIRP_Cur.Value, cfg0.Requirements.KuEIRP_Current_dBw);
        cfgX.Requirements.KuEIRP_Min_dBw = safeNum(c.Req_KuEIRP_Min.Value, cfg0.Requirements.KuEIRP_Min_dBw);
        cfgX.Requirements.KuGT_Current_dBperK = safeNum(c.Req_KuGT_Cur.Value, cfg0.Requirements.KuGT_Current_dBperK);
        cfgX.Requirements.KuGT_Min_dBperK = safeNum(c.Req_KuGT_Min.Value, cfg0.Requirements.KuGT_Min_dBperK);
        cfgX.Requirements.JA3700_TargetLevel = round(safeNum(c.Req_JA_Target.Value, cfg0.Requirements.JA3700_TargetLevel));
        cfgX.Requirements.JA3700_CurrentLevel = round(safeNum(c.Req_JA_Current.Value, cfg0.Requirements.JA3700_CurrentLevel));

        cfgX.Classifier.Enable = logical(c.Cls_Enable.Value);
        cfgX.Output.Enable3DViewer = logical(c.Out_3D.Value);
        cfgX.Display.ViewerSpeed = safeNum(c.Out_Speed.Value, cfg0.Display.ViewerSpeed);
        cfgX.Output.ExportFolder = char(c.Out_Folder.Value);
        cfgX.WorstCase.OutageThr_Mbps = cfgX.Requirements.MinThr_Mbps;
    end

    function dt = safeDatetime(v, fallback)
        try
            dt = datetime(v, 'TimeZone', 'UTC');
            if isempty(dt) || isnat(dt)
                dt = fallback;
            end
        catch
            dt = fallback;
        end
    end

    function x = safeNum(v, fallback)
        x = fallback;
        try
            if isnumeric(v) && isfinite(v)
                x = double(v);
            elseif ischar(v) || isstring(v)
                y = str2double(v);
                if isfinite(y)
                    x = y;
                end
            end
        catch
            x = fallback;
        end
    end
end

function h = addNumeric(parent, row, labelText, value)
    lbl = uilabel(parent, 'Text', labelText, 'HorizontalAlignment', 'right');
    lbl.Layout.Row = row; lbl.Layout.Column = 1;
    h = uieditfield(parent, 'numeric', 'Value', double(value));
    h.Layout.Row = row; h.Layout.Column = 2;
end

function h = addText(parent, row, labelText, value)
    lbl = uilabel(parent, 'Text', labelText, 'HorizontalAlignment', 'right');
    lbl.Layout.Row = row; lbl.Layout.Column = 1;
    h = uieditfield(parent, 'text', 'Value', char(value));
    h.Layout.Row = row; h.Layout.Column = 2;
end

function h = addDrop(parent, row, labelText, items, value)
    lbl = uilabel(parent, 'Text', labelText, 'HorizontalAlignment', 'right');
    lbl.Layout.Row = row; lbl.Layout.Column = 1;
    h = uidropdown(parent, 'Items', items);
    h.Value = char(value);
    h.Layout.Row = row; h.Layout.Column = 2;
end

function h = addCheck(parent, row, labelText, value)
    lbl = uilabel(parent, 'Text', labelText, 'HorizontalAlignment', 'right');
    lbl.Layout.Row = row; lbl.Layout.Column = 1;
    h = uicheckbox(parent, 'Value', logical(value), 'Text', '');
    h.Layout.Row = row; h.Layout.Column = 2;
end

function addNote(parent, row, txt)
    lbl = uilabel(parent, 'Text', txt, 'WordWrap', 'on', 'FontAngle', 'italic');
    lbl.Layout.Row = row; lbl.Layout.Column = [1 2];
end
