function onTickV7(dashFig)
%ONTICKV7 Timer callback for V7 dashboard + 3D link highlight.

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
    k = floor(dtsec/app.sample_time) + 1;
    k = max(1, min(app.numSteps, k));
    x = app.t_axis_min(k);

    % ---- move cursors ----
    lineSetX(app.curDLSINR, x); lineSetX(app.curDLBER, x); lineSetX(app.curDLTHR, x); lineSetX(app.curDLDOP, x);
    lineSetX(app.curULSINR, x); lineSetX(app.curULBER, x); lineSetX(app.curULTHR, x); lineSetX(app.curULDOP, x);
    lineSetX(app.curE2E, x); lineSetX(app.curOvThr, x); lineSetX(app.curOvJam, x); lineSetX(app.curOvDop, x);

    % ---- current values ----
    dl = app.worstDL;
    ul = app.worstUL;
    e2e = app.e2eWorst;

    sinrDL = dl.SINR(k); berDL = dl.BER(k); thrDL = dl.THR(k); prxDL = dl.Prx_dBm(k);
    sinrUL = ul.SINR(k); berUL = ul.BER(k); thrUL = ul.THR(k); prxUL = ul.Prx_dBm(k);
    thrE2E = e2e.THR(k);
    evtDL = string(dl.Event(k)); evtUL = string(ul.Event(k));

    dotSet(app.dotDLSINR, x, pickY(sinrDL, app.cfg.Display.SINR_YLIM(1)));
    dotSet(app.dotDLBER,  x, max(pickY(berDL, 1), 1e-9));
    dotSet(app.dotDLTHR,  x, pickY(thrDL, 0));
    dotSet(app.dotULSINR, x, pickY(sinrUL, app.cfg.Display.SINR_YLIM_UL(1)));
    dotSet(app.dotULBER,  x, max(pickY(berUL, 1), 1e-9));
    dotSet(app.dotULTHR,  x, pickY(thrUL, 0));

    % ---- labels ----
    spdTxt = 'Viewer Speed: -';
    if isfield(app, 'v') && ~isempty(app.v)
        try
            spdTxt = sprintf('Viewer Speed: x%.2f', app.v.PlaybackSpeedMultiplier);
        catch
        end
    end

    serv = dl.Serving(k);
    gw = dl.Gateway(k);
    visU = dl.VisUser(k);
    visG = dl.VisGW(k);

    hopsTxt = '-';
    if serv > 0 && gw > 0
        try
            pth = shortestpath(app.Gisl, serv, gw);
            hopsTxt = sprintf('%d', max(0, numel(pth)-1));
            app.pathLine.XData = app.satPlane(pth);
            app.pathLine.YData = app.satSlot(pth);
            app.gridServ.XData = app.satPlane(serv); app.gridServ.YData = app.satSlot(serv);
            app.gridGW.XData = app.satPlane(gw); app.gridGW.YData = app.satSlot(gw);
        catch
            app.pathLine.XData = nan; app.pathLine.YData = nan;
        end
    else
        app.pathLine.XData = nan; app.pathLine.YData = nan;
        app.gridServ.XData = nan; app.gridServ.YData = nan;
        app.gridGW.XData = nan; app.gridGW.YData = nan;
    end

    app.lblTime.Text  = sprintf('Current Time: %s', char(ct));
    app.lblSpeed.Text = spdTxt;
    app.lblServ.Text  = sprintf('Serving Sat: #%d', serv);
    app.lblGW.Text    = sprintf('Gateway Sat: #%d', gw);
    app.lblHops.Text  = sprintf('ISL Hops: %s', hopsTxt);
    app.lblVis.Text   = sprintf('Visible(User/GW): %d / %d', visU, visG);
    app.lblDL.Text    = sprintf('DL | SINR=%s dB | Thr=%s Mbps | Prx=%s dBm', fmtNum(sinrDL,2), fmtNum(thrDL,1), fmtNum(prxDL,1));
    app.lblUL.Text    = sprintf('UL | SINR=%s dB | Thr=%s Mbps | Prx=%s dBm', fmtNum(sinrUL,2), fmtNum(thrUL,1), fmtNum(prxUL,1));
    app.lblE2E.Text   = sprintf('E2E | Thr(min)=%s Mbps | Delay=%s ms', fmtNum(thrE2E,1), fmtNum(e2e.Delay_ms(k),2));

    % ---- compliance ----
    [rows, overallPass] = emcComputeComplianceRowsV7(app.cfg, dl, ul, e2e, k);
    app.tblComp.Data = rows;
    if overallPass
        app.lamp.Color = [0 1 0];
    else
        app.lamp.Color = [1 0 0];
    end

    % ---- classifier ----
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

    % ---- sky view ----
    visIdx = find(app.elU(k,:) > app.cfg.Constellation.ElMask_deg);
    if ~isempty(visIdx)
        app.skyAll.XData = wrap180_vec(app.azU(k, visIdx));
        app.skyAll.YData = app.elU(k, visIdx);
        if serv > 0
            app.skyServ.XData = wrap180(app.azU(k, serv));
            app.skyServ.YData = app.elU(k, serv);
        else
            app.skyServ.XData = nan; app.skyServ.YData = nan;
        end
        if gw > 0
            app.skyGW.XData = wrap180(app.azU(k, gw));
            app.skyGW.YData = app.elU(k, gw);
        else
            app.skyGW.XData = nan; app.skyGW.YData = nan;
        end
    else
        app.skyAll.XData = nan; app.skyAll.YData = nan;
        app.skyServ.XData = nan; app.skyServ.YData = nan;
        app.skyGW.XData = nan; app.skyGW.YData = nan;
    end

    % ---- 3D access highlight ----
    if isfield(app, 'v') && ~isempty(app.v)
        try
            if app.lastServ ~= serv
                if app.lastServ > 0 && app.lastServ <= numel(app.acUser) && isvalid(app.acUser(app.lastServ))
                    app.acUser(app.lastServ).LineColor = [0.75 0.75 0.75];
                    app.acUser(app.lastServ).LineWidth = 0.6;
                end
                if serv > 0 && serv <= numel(app.acUser) && isvalid(app.acUser(serv))
                    app.acUser(serv).LineColor = [0 1 0];
                    app.acUser(serv).LineWidth = 2.2;
                end
                app.lastServ = serv;
            end

            if app.lastGW ~= gw
                if app.lastGW > 0 && app.lastGW <= numel(app.acGW) && isvalid(app.acGW(app.lastGW))
                    app.acGW(app.lastGW).LineColor = [0.65 0.75 1.00];
                    app.acGW(app.lastGW).LineWidth = 0.6;
                end
                if gw > 0 && gw <= numel(app.acGW) && isvalid(app.acGW(gw))
                    app.acGW(gw).LineColor = [0 0.45 1];
                    app.acGW(gw).LineWidth = 2.2;
                end
                app.lastGW = gw;
            end
        catch
        end

        try
            if isfield(app, 'lastISLEdges') && ~isempty(app.lastISLEdges)
                for ee = app.lastISLEdges(:).'
                    if ee >= 1 && ee <= numel(app.acISL) && isvalid(app.acISL(ee))
                        app.acISL(ee).LineColor = [0.80 0.80 0.80];
                        app.acISL(ee).LineWidth = 0.4;
                    end
                end
            end
            edgesNow = [];
            if serv > 0 && gw > 0
                pth2 = shortestpath(app.Gisl, serv, gw);
                for ii = 1:(numel(pth2)-1)
                    ee = app.islMap(pth2(ii), pth2(ii+1));
                    if ee > 0
                        edgesNow(end+1) = ee; %#ok<AGROW>
                    end
                end
                for ee = edgesNow
                    if ee >= 1 && ee <= numel(app.acISL) && isvalid(app.acISL(ee))
                        app.acISL(ee).LineColor = [1 0.6 0];
                        app.acISL(ee).LineWidth = 2.0;
                    end
                end
            end
            app.lastISLEdges = edgesNow;
        catch
        end
    end

    % ---- event log ----
    evtPair = "DL:" + evtDL + " | UL:" + evtUL;
    if app.lastEvent ~= evtPair
        data = app.tblEvt.Data;
        if size(data,1) >= 150
            data = data(end-120:end,:);
        end
        app.tblEvt.Data = [data; {char(ct), char(evtDL), char(evtUL), char(evtPair)}];
        app.lastEvent = evtPair;
    end

    guidata(dashFig, app);
    drawnow limitrate;
end

function lineSetX(h, x)
    try
        h.XData = [x x];
    catch
    end
end

function dotSet(h, x, y)
    try
        h.XData = x; h.YData = y;
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
