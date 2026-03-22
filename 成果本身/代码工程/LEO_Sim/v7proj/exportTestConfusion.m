function exportTestConfusion(net, datasetRoot, exportDir, classes)
    % 导出 test 集混淆矩阵（用于PPT）
    try
        testDir = fullfile(datasetRoot,'test');
        imdsTest = imageDatastore(testDir,'IncludeSubfolders',true,'LabelSource','foldernames', ...
            'ReadFcn', @(x) im2single(im2gray(imread(x))));
        imdsTest.Labels = reordercats(imdsTest.Labels, classes);
        augTest = augmentedImageDatastore([128 128], imdsTest);
 
        pred = classify(net, augTest);
        pred = reordercats(pred, classes);
 
        cm = confusionmat(imdsTest.Labels, pred, 'Order', categorical(classes,classes));
        cmN = cm ./ max(1,sum(cm,2));
 
        f = figure('Visible','off','Color','w','Position',[100 100 900 700]);
        imagesc(cmN); axis image; colorbar;
        xticks(1:numel(classes)); yticks(1:numel(classes));
        xticklabels(classes); yticklabels(classes);
        title('Confusion Matrix (Normalized) on Test Set');
        xlabel('Predicted'); ylabel('True');
        for i=1:numel(classes)
            for j=1:numel(classes)
                text(j,i,sprintf('%.2f',cmN(i,j)),'HorizontalAlignment','center','Color','w','FontWeight','bold');
            end
        end
        outPng = fullfile(exportDir, 'confusion_test.png');
        exportgraphics(f, outPng, 'Resolution', 180);
        close(f);
        fprintf('  [IntfCls][Export] Saved confusion matrix: %s\n', outPng);
    catch ME
        fprintf('  [IntfCls][Export] Confusion matrix export failed: %s\n', ME.message);
    end
end
