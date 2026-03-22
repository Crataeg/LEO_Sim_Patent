function shadeTagRegions(ax, t_axis, tags, key, yLim, colorRGB, labelText, labelY)
    idx = find(contains(tags, key));
    if isempty(idx), return; end
    idx = idx(:);
    breaks = [1; find(diff(idx)>1)+1; numel(idx)+1];
    for b=1:(numel(breaks)-1)
        seg = idx(breaks(b):breaks(b+1)-1);
        x1=t_axis(seg(1)); x2=t_axis(seg(end));
        X=[x1 x2 x2 x1]; Y=[yLim(1) yLim(1) yLim(2) yLim(2)];
        patch(ax,X,Y,colorRGB,'FaceAlpha',0.12,'EdgeColor','none');
        text(ax,mean([x1 x2]),labelY,labelText,'Color',colorRGB,'HorizontalAlignment','center');
    end
end
 
%% =========================
% Local Functions: STFT+LeNet Interference Classifier (R2021a)  + ?“图片导出完全体”
% =========================
