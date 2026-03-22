function idx = pickKeyframes(eventTags, numSteps)
    % 从事件序列中挑选关键帧：干扰开始/中段/结束 + Protected/CoChannel
    tags = string(eventTags);
    idx = [];
    % JAMMING
    j = find(contains(tags,"JAMMING"));
    if ~isempty(j)
        idx = [idx; j(1); j(round(end/2)); j(end)];
    end
    % Protected
    p = find(contains(tags,"Protected"));
    if ~isempty(p)
        idx = [idx; p(1); p(round(end/2)); p(end)];
    end
    % CoChannel
    c = find(contains(tags,"CoChannel"));
    if ~isempty(c)
        idx = [idx; c(1); c(round(end/2)); c(end)];
    end
    % Normal（挑均匀3点）
    n = find(contains(tags,"Normal"));
    if ~isempty(n)
        idx = [idx; n(1); n(round(end/2)); n(end)];
    end
    idx = unique(max(1, min(numSteps, idx(:))));
    if numel(idx) > 12
        idx = idx(round(linspace(1,numel(idx),12)));
    end
end
