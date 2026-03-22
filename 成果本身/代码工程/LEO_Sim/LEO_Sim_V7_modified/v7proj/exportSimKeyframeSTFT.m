function exportSimKeyframeSTFT(snapInfo, exportDir)
    % 导出仿真关键帧STFT图：每帧单独一张 + 总拼图
    try
        if ~exist(exportDir,'dir'), mkdir(exportDir); end
 
        n = numel(snapInfo.tIndex);
        if n==0, return; end
 
        files = cell(n,1);
        for i=1:n
            img = snapInfo.img{i};
            if isempty(img), continue; end
            evt = snapInfo.event(i);
            tc  = snapInfo.trueClass(i);
            pc  = snapInfo.predClass(i);
            k   = snapInfo.tIndex(i);
 
            f = figure('Visible','off','Color','w','Position',[100 100 520 480]);
            imshow(img,[]); colormap gray;
            title(sprintf('k=%d | evt=%s | true=%s | pred=%s', k, evt, tc, pc), 'Interpreter','none');
            fn = fullfile(exportDir, sprintf('sim_keyframe_%02d_k%04d_%s_%s.png', i, k, tc, pc));
            exportgraphics(f, fn, 'Resolution', 200);
            close(f);
            files{i} = fn;
        end
 
        files = files(~cellfun(@isempty,files));
        if isempty(files), return; end
 
        f2 = figure('Visible','off','Color','w','Position',[100 100 1200 650]);
        montage(files); title('Simulation Keyframes STFT (PPT-ready)');
        fn2 = fullfile(exportDir, 'sim_keyframes_montage.png');
        exportgraphics(f2, fn2, 'Resolution', 180);
        close(f2);
 
        fprintf('  [IntfCls][Export] Saved sim keyframes to: %s\n', exportDir);
    catch ME
        fprintf('  [IntfCls][Export] Sim keyframe export failed: %s\n', ME.message);
    end
end
