
% Load settings file

clear mex
close all

eval(sprintf('settings_%s;',getPCName()));

% Build directories

visible_logFolder = '/home/will/Data/Thermalvis/visible-sequences/results_2012_06_25_19_54_54';
thermal_logFolder = '/home/will/Data/Thermalvis/thermal-sequences/results_2012_06_25_13_59_18';

rootDir = sprintf('/home/%s/Data/Thermalvis/combined-sequences',getPCName());

c = clock;
c(6) = floor(c(6));
logFolder = sprintf('%s/results_%04d_%02d_%02d_%02d_%02d_%02d',...
    rootDir,c(1),c(2),c(3),c(4),c(5),c(6));
system(sprintf('mkdir %s',logFolder));
system(sprintf('cp settings_%s.m %s/',getPCName(),logFolder));
system(sprintf('mkdir %s/descriptors',logFolder));
for subDir = unique([trainingTimes testingTimes])
    system(sprintf('mkdir %s/descriptors/%s',logFolder,subDir{:}));
end
descriptorSub = '%04d.yml';
numDatasets = size(testingTimes,2);

clTreeFile = sprintf('%s/clTree.yml',logFolder);

% Load and combine descriptors

for subDir = unique([trainingTimes testingTimes])
    
    visible_imgDescriptorFile = [visible_logFolder '/descriptors/' subDir{:} '/imgDescriptors.yml'];
    thermal_imgDescriptorFile = [thermal_logFolder '/descriptors/' subDir{:} '/imgDescriptors.yml'];
    combined_imgDescriptorFile = [logFolder '/descriptors/' subDir{:} '/imgDescriptors.yml'];
    
    combinedImgDescriptors = [m_load_imgDescriptors(visible_imgDescriptorFile) m_load_imgDescriptors(thermal_imgDescriptorFile)];
    
    m_save_imgDescriptors(combinedImgDescriptors, combined_imgDescriptorFile);
end

% Build combined Chow-Liu tree

m_generate_cltree('initialise',' ');

for subDir = trainingTimes
    
    imgDescriptorFile = [logFolder '/descriptors/' subDir{:} '/imgDescriptors.yml'];
    m_generate_cltree('add',imgDescriptorFile);

end

m_generate_cltree('make',clTreeFile);

clTreeData = m_load_clTree(clTreeFile);

if (debugMode > 0)
    F = figure;
    hist(clTreeData(2,:),50)
    treeHistFilename = sprintf('%s/combined_tree_hist.pdf',logFolder);
    save2pdf(treeHistFilename, F, 600);
end


% Generate FAB-MAP confusion matrices

allConfusionMats = zeros(2*numFrames,2*numFrames,size(testingTimes,2),size(testingTimes,2));
allPrecisionRecall = zeros(2,2*numFrames,size(testingTimes,2),size(testingTimes,2));
allRecall = zeros(size(testingTimes,2),size(testingTimes,2));

min_dist_rec = [ones(1,numFrames) zeros(1,numFrames)];
blankFrames = 10;

tp_dist = 3;
fn_dist = 1;

index1 = 1;
index2 = 1;
for subDir1 = testingTimes
    for subDir2 = testingTimes
        
        %clear mex
        
        imgDescriptorFile1 = [logFolder '/descriptors/' subDir1{:} '/imgDescriptors.yml'];
        imgDescriptorFile2 = [logFolder '/descriptors/' subDir2{:} '/imgDescriptors.yml'];
        
        confusionMat = m_process_fabmap(imgDescriptorFile1,imgDescriptorFile2,...
            clTreeFile,fabmapVersion,fabmapNewPlace,confusionMatMode);
        
        h_rec = zeros(1,2*numFrames);
        h_index_rec = zeros(1,2*numFrames);
        h_dist_rec = ones(1,2*numFrames)*2*numFrames;
        
        for i=blankFrames:size(confusionMat,1)
            [h_rec(1,i) h_index_rec(1,i)] = max(confusionMat(i,1:i-blankFrames+1));
            h_dist_rec(1,i) = abs(h_index_rec(1,i) - (i - numFrames));
        end
        
        sorted_results = [h_rec; h_index_rec; h_dist_rec; min_dist_rec; 1:2*numFrames];
        sorted_results = fliplr(sortrows(sorted_results',1)');
        
        prec_recall = zeros(2,2*numFrames);

        for i=1:size(prec_recall,2)
            fp = sum(sorted_results(3,1:i)>tp_dist);
            tp = i-fp;
            fn = sum(sorted_results(4,i+1:end)<fn_dist);
            prec_recall(1,i) = tp/(tp+fn);
            prec_recall(2,i) = tp/(tp+fp);
        end
        
%         plot(prec_recall(1,:),prec_recall(2,:))
%         imagesc(confusionMat(300:600,1:300))
%         colorbar
        
        allConfusionMats(:,:,index1,index2) = confusionMat;
        allPrecisionRecall(:,:,index1,index2) = prec_recall;
        
        index2 = index2 + 1;
    end
    
    index1 = index1 + 1;
    index2 = 1;
   
end

% Generate PDF plots of precision-recall curves

allConfusionMatsMod = zeros(size(allConfusionMats));

framewiseMats = zeros(size(testingTimes,2), size(testingTimes,2), numFrames);
framewiseMatsMod = zeros(size(testingTimes,2), size(testingTimes,2), numFrames);


for iii = 1:size(testingTimes,2)
    for jjj = 1:size(testingTimes,2)
        
        divMatrix = repmat(max([sum(allImgDescriptors(:,:,iii)>0,2); sum(allImgDescriptors(:,:,jjj)>0,2)],1),1,2*numFrames);
        allConfusionMatsMod(:,:,iii,jjj) = allConfusionMats(:,:,iii,jjj)./divMatrix;
        %allConfusionMatsMod(:,:,iii,jjj) = allConfusionMatsMod(:,:,iii,jjj)./repmat(diag(allConfusionMatsMod(:,:,iii,jjj)),1,2*numFrames);
        
        if (debugMode > 0)
            F = figure;
            plot(allPrecisionRecall(1,:,iii,jjj), allPrecisionRecall(2,:,iii,jjj));

            prFileName = sprintf('%s/pr-%s-%s.pdf',logFolder, testingTimes{iii}, testingTimes{jjj});
            save2pdf(prFileName, F, 600);

            F = figure;
            imagesc(allConfusionMats(:,:,iii,jjj));

            confusionFileName = sprintf('%s/confusion-%s-%s.pdf',logFolder, testingTimes{iii}, testingTimes{jjj});
            save2pdf(confusionFileName, F, 600);
            
        end

        for aaa = 1:numFrames
            
            x = numFrames + aaa;
            y = aaa;
                        
            framewiseMats(iii,jjj,aaa) = allConfusionMats(x, y, iii, jjj);
            framewiseMatsMod(iii,jjj,aaa) = allConfusionMatsMod(x, y, iii, jjj);

        end
        
        
    end
end

framewiseMean = mean(framewiseMats, 3);
framewiseMeanMod = mean(framewiseMatsMod, 3);

if (debugMode > 0)
    F = figure;
    imagesc(framewiseMean); colorbar;
    framewiseFileName = sprintf('%s/framewise.pdf',logFolder);
    save2pdf(framewiseFileName, F, 600);
    
end

% Save FAB-MAP results data

matFileName = sprintf('%s/results.mat',logFolder);
save(matFileName,'allConfusionMats','allPrecisionRecall','clTreeData', 'framewiseMean');
