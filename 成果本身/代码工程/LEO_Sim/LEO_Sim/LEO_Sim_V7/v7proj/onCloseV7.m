function onCloseV7(src, ~, tmr)
%ONCLOSEV7 Stop timer and close dashboard.

    try
        if isa(tmr, 'timer') && isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
    catch
    end

    try
        delete(src);
    catch
    end
end
