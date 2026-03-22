function onCloseV7(src, ~, tmr)
%ONCLOSEV7 Stop timer and close detached windows safely.

    app = [];
    try
        app = guidata(src);
    catch
    end

    try
        if isa(tmr, 'timer') && isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
    catch
    end

    try
        if ~isempty(app) && isfield(app,'skyFig') && ~isempty(app.skyFig) && isvalid(app.skyFig)
            delete(app.skyFig);
        end
    catch
    end

    try
        if ~isempty(app) && isfield(app,'evtFig') && ~isempty(app.evtFig) && isvalid(app.evtFig)
            delete(app.evtFig);
        end
    catch
    end

    try
        delete(src);
    catch
    end
end