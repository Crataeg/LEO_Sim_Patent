function app = updateViewerHighlightOnChange(app, serv, gw, routeNow)
%UPDATEVIEWERHIGHLIGHTONCHANGE
% viewer 中只保留当前通信链路：
%   1) 用户 -> 服务星
%   2) 服务星 -> ... -> 网关星（ISL）
%   3) 网关星 -> 地面网关
%
% 不再保留默认星地/星间链路，不再做持续颜色替换。

    if ~isfield(app, 'lastServ'),  app.lastServ = 0; end
    if ~isfield(app, 'lastGW'),    app.lastGW = 0; end
    if ~isfield(app, 'lastRoute'), app.lastRoute = []; end

    servChanged  = (serv ~= app.lastServ);
    gwChanged    = (gw ~= app.lastGW);
    routeChanged = ~isequal(routeNow, app.lastRoute);

    if ~(servChanged || gwChanged || routeChanged)
        return;
    end

    % -------------------------------------------------
    % 1) 删除旧的当前链路对象
    % -------------------------------------------------
    try
        if ~isempty(app.curUserLink) && isvalid(app.curUserLink)
            delete(app.curUserLink);
        end
    catch
    end
    app.curUserLink = [];

    try
        if ~isempty(app.curGWLink) && isvalid(app.curGWLink)
            delete(app.curGWLink);
        end
    catch
    end
    app.curGWLink = [];

    try
        if ~isempty(app.curISLLinks)
            for i = 1:numel(app.curISLLinks)
                try
                    if isvalid(app.curISLLinks(i))
                        delete(app.curISLLinks(i));
                    end
                catch
                end
            end
        end
    catch
    end
    app.curISLLinks = gobjects(0,1);

    % -------------------------------------------------
    % 2) 创建新的当前链路对象
    % -------------------------------------------------
    hlColor = [1.00 0.45 0.05];
    if isfield(app, 'activeLinkColor') && ~isempty(app.activeLinkColor)
        hlColor = app.activeLinkColor;
    end

    % 用户 -> 服务星
    if serv > 0
        try
            app.curUserLink = access(app.satConst{serv}, app.gsUser);
            app.curUserLink.LineColor = hlColor;
            app.curUserLink.LineWidth = 2.8;
        catch
            app.curUserLink = [];
        end
    end

    % 服务星 -> ... -> 网关星（ISL）
    if ~isempty(routeNow) && numel(routeNow) >= 2
        app.curISLLinks = gobjects(numel(routeNow)-1,1);
        for ii = 1:(numel(routeNow)-1)
            u = routeNow(ii);
            v = routeNow(ii+1);
            try
                app.curISLLinks(ii) = access(app.satConst{u}, app.satConst{v});
                app.curISLLinks(ii).LineColor = hlColor;
                app.curISLLinks(ii).LineWidth = 2.2;
            catch
            end
        end
    end

    % 网关星 -> 地面网关
    if gw > 0
        try
            app.curGWLink = access(app.satConst{gw}, app.gsGW);
            app.curGWLink.LineColor = hlColor;
            app.curGWLink.LineWidth = 2.8;
        catch
            app.curGWLink = [];
        end
    end

    % -------------------------------------------------
    % 3) 更新缓存
    % -------------------------------------------------
    app.lastServ  = serv;
    app.lastGW    = gw;
    app.lastRoute = routeNow;
end