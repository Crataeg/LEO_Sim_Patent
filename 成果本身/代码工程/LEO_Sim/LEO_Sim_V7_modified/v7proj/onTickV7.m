function onTickV7(dashFig)
%ONTICKV7
% 修正版：
% 1) 不动态更新 satelliteScenarioViewer 链路
% 2) 星座图保持二维 Plane-Slot 正常显示
% 3) Sky View 名称每 tick 重建，避免重影与错位
% 4) 对不在当前天空中的链路做地平线裁剪显示
% 5) 底部两行文字栏显示当前路由
% 6) 清除所有带 Tag 的旧标签，杜绝背景残留名称

    if ~isvalid(dashFig)
        return;
    end

    app = guidata(dashFig);
    if isempty(app)
        return;
    end

    ct = app.sim_start;
    if isfield(app, 'v') && ~isempty(app.v)
        try
            if isvalid(app.v)
                ct = app.v.CurrentTime;
            end
        catch
            ct = app.sim_start;
        end
    end

    dtsec = seconds(ct - app.sim_start);
    k = floor(dtsec / app.sample_time) + 1;
    k = max(1, min(app.numSteps, k));
    x = app.t_axis_min(k);

    lineSetX(app.curDLSINR, x);
    lineSetX(app.curDLBER,  x);
    lineSetX(app.curDLTHR,  x);
    lineSetX(app.curDLDOP,  x);

    lineSetX(app.curULSINR, x);
    lineSetX(app.curULBER,  x);
    lineSetX(app.curULTHR,  x);
    lineSetX(app.curULDOP,  x);

    lineSetX(app.curE2E,   x);
    lineSetX(app.curOvThr, x);
    lineSetX(app.curOvJam, x);
    lineSetX(app.curOvDop, x);

    dl  = app.worstDL;
    ul  = app.worstUL;
    e2e = app.e2eWorst;

    sinrDL = safeIndex(dl.SINR, k, NaN);
    berDL  = safeIndex(dl.BER,  k, NaN);
    thrDL  = safeIndex(dl.THR,  k, NaN);
    prxDL  = pickFieldValue(dl, 'Prx_dBm', k, NaN);

    sinrUL = safeIndex(ul.SINR, k, NaN);
    berUL  = safeIndex(ul.BER,  k, NaN);
    thrUL  = safeIndex(ul.THR,  k, NaN);
    prxUL  = pickFieldValue(ul, 'Prx_dBm', k, NaN);

    dotSet(app.dotDLSINR, x, pickY(sinrDL, app.cfg.Display.SINR_YLIM(1)));
    dotSet(app.dotDLBER,  x, max(pickY(berDL, 1), 1e-9));
    dotSet(app.dotDLTHR,  x, pickY(thrDL, 0));

    dotSet(app.dotULSINR, x, pickY(sinrUL, app.cfg.Display.SINR_YLIM_UL(1)));
    dotSet(app.dotULBER,  x, max(pickY(berUL, 1), 1e-9));
    dotSet(app.dotULTHR,  x, pickY(thrUL, 0));

    serv = safeIndex(dl.Serving, k, 0);
    gw   = safeIndex(dl.Gateway, k, 0);

    evtDL = string(safeStringIndex(dl.Event, k, "NA"));
    evtUL = string(safeStringIndex(ul.Event, k, "NA"));

    routeNow = [];
    if serv > 0 && gw > 0
        try
            routeNow = shortestpath(app.Gisl, serv, gw);
        catch
            routeNow = [];
        end
    end

    spdTxt = 'Viewer Speed: -';
    if isfield(app, 'v') && ~isempty(app.v)
        try
            spdTxt = sprintf('Viewer Speed: x%.2f', app.v.PlaybackSpeedMultiplier);
        catch
        end
    end

    visU = 0; visG = 0;
    try, visU = sum(app.elU(k,:) > app.cfg.Constellation.ElMask_deg); catch, end
    try, visG = sum(app.elG(k,:) > app.cfg.Constellation.ElMask_deg); catch, end

    if isempty(routeNow)
        hopsTxt = '-';
    else
        hopsTxt = sprintf('%d', max(0, numel(routeNow)-1));
    end

    app.lblTime.Text  = sprintf('Current Time: %s', char(ct));
    app.lblSpeed.Text = spdTxt;
    app.lblServ.Text  = sprintf('Serving Sat: #%d', serv);
    app.lblGW.Text    = sprintf('Gateway Sat: #%d', gw);
    app.lblHops.Text  = sprintf('ISL Hops: %s', hopsTxt);
    app.lblVis.Text   = sprintf('Visible(User/GW): %d / %d', visU, visG);
    app.lblDL.Text    = sprintf('DL | SINR=%s dB | Thr=%s Mbps | Prx=%s dBm', ...
        fmtNum(sinrDL,2), fmtNum(thrDL,1), fmtNum(prxDL,1));
    app.lblUL.Text    = sprintf('UL | SINR=%s dB | Thr=%s Mbps | Prx=%s dBm', ...
        fmtNum(sinrUL,2), fmtNum(thrUL,1), fmtNum(prxUL,1));

    delay_ms = pickFieldValue(e2e, 'Delay_ms', k, NaN);
    app.lblE2E.Text = sprintf('E2E | Thr(min)=%s Mbps | Delay=%s ms', ...
        fmtNum(safeIndex(e2e.THR, k, NaN),1), fmtNum(delay_ms,2));

    try
        [rows, overallPass] = emcComputeComplianceRowsV7(app.cfg, dl, ul, e2e, k);
        app.tblComp.Data = rows;
        if overallPass
            app.lamp.Color = [0 1 0];
        else
            app.lamp.Color = [1 0 0];
        end
    catch
    end

    if isfield(app, 'intfPred') && ~isempty(app.intfPred) && numel(app.intfPred) >= k
        pred = app.intfPred(k);
        app.lblIntf.Text = sprintf('Interference Class (DL STFT+LeNet): %s', char(pred));
        if isfield(app, 'intfScore') && ~isempty(app.intfScore) && size(app.intfScore,1) >= k
            try
                app.barIntf.YData = app.intfScore(k,:);
            catch
            end
        end
    end

    % 2D Constellation Grid
    if serv > 0
        app.gridServ.XData = app.satPlane(serv);
        app.gridServ.YData = app.satSlot(serv);
    else
        app.gridServ.XData = nan;
        app.gridServ.YData = nan;
    end

    if gw > 0 && gw ~= serv
        app.gridGW.XData = app.satPlane(gw);
        app.gridGW.YData = app.satSlot(gw);
    else
        app.gridGW.XData = nan;
        app.gridGW.YData = nan;
    end

    if ~isempty(routeNow) && numel(routeNow) >= 2
        app.pathLine.XData = app.satPlane(routeNow);
        app.pathLine.YData = app.satSlot(routeNow);
    else
        app.pathLine.XData = nan;
        app.pathLine.YData = nan;
    end

    % Sky View visible satellites
    visIdx = [];
    try
        visIdx = find(app.elU(k,:) > app.cfg.Constellation.ElMask_deg);
    catch
    end

    if ~isempty(visIdx)
        azVis = app.azU(k, visIdx);
        elVis = app.elU(k, visIdx);
        [xv,yv,zv] = skyXYZVisible(azVis, elVis);
        app.sky3All.XData = xv;
        app.sky3All.YData = yv;
        app.sky3All.ZData = zv;
    else
        app.sky3All.XData = nan;
        app.sky3All.YData = nan;
        app.sky3All.ZData = nan;
    end

    % Service star
    if serv > 0
        [xs,ys,zs] = skyXYZClipped(app.azU(k, serv), app.elU(k, serv));
        app.sky3Serv.XData = xs;
        app.sky3Serv.YData = ys;
        app.sky3Serv.ZData = zs;

        app.sky3UserLink.XData = [0 xs];
        app.sky3UserLink.YData = [0 ys];
        app.sky3UserLink.ZData = [0 zs];
    else
        app.sky3Serv.XData = nan;
        app.sky3Serv.YData = nan;
        app.sky3Serv.ZData = nan;
        app.sky3UserLink.XData = nan;
        app.sky3UserLink.YData = nan;
        app.sky3UserLink.ZData = nan;
    end

    % Gateway star
    if gw > 0 && gw ~= serv
        [xg,yg,zg] = skyXYZClipped(app.azU(k, gw), app.elU(k, gw));
        app.sky3GW.XData = xg;
        app.sky3GW.YData = yg;
        app.sky3GW.ZData = zg;
    else
        app.sky3GW.XData = nan;
        app.sky3GW.YData = nan;
        app.sky3GW.ZData = nan;
    end

    % ISL route clipped
    if ~isempty(routeNow) && numel(routeNow) >= 2
        azPath = app.azU(k, routeNow);
        elPath = app.elU(k, routeNow);
        [xp,yp,zp] = skyXYZClipped(azPath, elPath);
        app.sky3ISLPath.XData = xp;
        app.sky3ISLPath.YData = yp;
        app.sky3ISLPath.ZData = zp;
    else
        app.sky3ISLPath.XData = nan;
        app.sky3ISLPath.YData = nan;
        app.sky3ISLPath.ZData = nan;
    end

    % ---- 强力清除旧标签 ----
    try
        if isfield(app, 'sky3Labels') && ~isempty(app.sky3Labels)
            for ii = 1:numel(app.sky3Labels)
                try
                    if isvalid(app.sky3Labels(ii))
                        delete(app.sky3Labels(ii));
                    end
                catch
                end
            end
        end
    catch
    end

    try
        staleTxt = findall(app.axSky3D, 'Type', 'text', 'Tag', 'SkyDynLabel');
        if ~isempty(staleTxt)
            delete(staleTxt);
        end
    catch
    end

    app.sky3Labels = gobjects(0,1);

    % 每颗星只保留一个标签
    labelMap = containers.Map('KeyType','double','ValueType','any');

    % Visible blue satellites
    if ~isempty(visIdx)
        for ii = 1:numel(visIdx)
            sid = visIdx(ii);
            [xt,yt,zt] = skyXYZVisible(app.azU(k, sid), app.elU(k, sid));
            labelMap(sid) = struct('x',xt,'y',yt,'z',zt,'fs',7,'fw','normal','col',[0.20 0.20 0.35]);
        end
    end

    % Service/gateway/route overwrite style
    emphIdx = [];
    if serv > 0, emphIdx(end+1) = serv; end %#ok<AGROW>
    if gw > 0 && gw ~= serv, emphIdx(end+1) = gw; end %#ok<AGROW>
    if ~isempty(routeNow), emphIdx = unique([emphIdx, routeNow]); end

    for ii = 1:numel(emphIdx)
        sid = emphIdx(ii);
        [xt,yt,zt] = skyXYZClipped(app.azU(k, sid), app.elU(k, sid));
        labelMap(sid) = struct('x',xt,'y',yt,'z',zt,'fs',8,'fw','bold','col',[0.05 0.05 0.05]);
    end

    keysList = cell2mat(keys(labelMap));
    for ii = 1:numel(keysList)
        sid = keysList(ii);
        info = labelMap(sid);
        try
            app.sky3Labels(end+1) = text(app.axSky3D, info.x, info.y, info.z + 0.03, ...
                char(app.satName(sid)), ...
                'FontSize', info.fs, ...
                'FontWeight', info.fw, ...
                'Color', info.col, ...
                'HorizontalAlignment', 'center', ...
                'Tag', 'SkyDynLabel'); %#ok<AGROW>
        catch
        end
    end

    % Bottom route text
    routeLines = buildRouteLines(routeNow, app.satName);
    if isfield(app, 'txtSkyRoute') && ~isempty(app.txtSkyRoute) && isvalid(app.txtSkyRoute)
        app.txtSkyRoute.Value = routeLines;
    end

    % event log
    evtPair = "DL:" + evtDL + " | UL:" + evtUL;
    if ~isfield(app, 'lastEvent') || app.lastEvent ~= evtPair
        try
            data = app.tblEvt.Data;
            if size(data,1) >= 150
                data = data(end-120:end,:);
            end
            app.tblEvt.Data = [data; {char(ct), char(evtDL), char(evtUL), char(evtPair)}];
        catch
        end
        app.lastEvent = evtPair;
    end

    guidata(dashFig, app);
    drawnow limitrate;
end

function lines = buildRouteLines(routeNow, satName)
    if isempty(routeNow)
        lines = {'Current Route: -',' '};
        return;
    end

    names = cell(1, numel(routeNow));
    for i = 1:numel(routeNow)
        names{i} = char(satName(routeNow(i)));
    end

    fullStr = strjoin(names, ' -> ');
    if numel(fullStr) <= 72
        lines = {['Current Route: ' fullStr], ' '};
        return;
    end

    mid = ceil(numel(names)/2);
    line1 = strjoin(names(1:mid), ' -> ');
    line2 = strjoin(names(mid+1:end), ' -> ');
    lines = {['Current Route: ' line1], line2};
end

function [x,y,z] = skyXYZVisible(azDeg, elDeg)
    rxy = cosd(elDeg);
    x = rxy .* sind(azDeg);
    y = rxy .* cosd(azDeg);
    z = sind(elDeg);
end

function [x,y,z] = skyXYZClipped(azDeg, elDeg)
    elClip = elDeg;
    elClip(elClip < 2) = 2;
    elClip(elClip > 89) = 89;

    rxy = cosd(elClip);
    x = rxy .* sind(azDeg);
    y = rxy .* cosd(azDeg);
    z = sind(elClip);
end

function lineSetX(h, x)
    try, h.XData = [x x]; catch, end
end

function dotSet(h, x, y)
    try
        h.XData = x;
        h.YData = y;
    catch
    end
end

function y = pickY(v, fallback)
    if isempty(v) || isnan(v)
        y = fallback;
    else
        y = v;
    end
end

function s = fmtNum(v, n)
    if nargin < 2, n = 2; end
    if isempty(v) || isnan(v)
        s = '-';
    else
        s = num2str(v, ['%0.' num2str(n) 'f']);
    end
end

function v = safeIndex(arr, k, fallback)
    try
        v = arr(k);
        if isempty(v) || isnan(v), v = fallback; end
    catch
        v = fallback;
    end
end

function s = safeStringIndex(arr, k, fallback)
    try
        s = arr(k);
        if isempty(s), s = fallback; end
    catch
        s = fallback;
    end
end

function v = pickFieldValue(S, fieldName, k, fallback)
    try
        if isfield(S, fieldName)
            arr = S.(fieldName);
            v = arr(k);
            if isempty(v) || isnan(v), v = fallback; end
        else
            v = fallback;
        end
    catch
        v = fallback;
    end
end