function exportDatasetMontage(datasetRoot, exportDir)
    % 导出每类若干张样例拼图（用于PPT）
    try
        classes = {'none','tone','pbnj','mod'};
        split = 'train';
        outPng = fullfile(exportDir, sprintf('montage_%s.png', split));
        files = {};
        for i=1:numel(classes)
            d = fullfile(datasetRoot, split, classes{i});
            L = dir(fullfile(d,'*.png'));
            take = min(8, numel(L));
            for k=1:take
                files{end+1} = fullfile(L(k).folder, L(k).name); %#ok<AGROW>
            end
        end
        if isempty(files), return; end
        f = figure('Visible','off','Color','w','Position',[100 100 1200 650]);
        montage(files,'Size',[4 8]); title(sprintf('Dataset Samples (split=%s)', split));
        exportgraphics(f, outPng, 'Resolution', 180);
        close(f);
        fprintf('  [IntfCls][Export] Saved montage: %s\n', outPng);
    catch ME
        fprintf('  [IntfCls][Export] Montage failed: %s\n', ME.message);
    end
end
