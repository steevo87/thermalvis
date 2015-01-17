function [S B] = descriptors_profiling_summary(modality)

    eval('plot_settings');
    
    res = 10 * (1 / breadth);

    xValues = linspace(0, 1.00, res+1);
    
    desiredCount = 300;
    maxAllowableDiff = 10;
    
    
    %descriptorPrefixes = {'SURF'; 'SIFT'; 'SpTb64'; 'BRIEF'; 'SpTn64'};
    
    %descriptorPrefixes = {'Sc8Tn64'; 'Sc8Tb208'; 'BRIEF'; 'SURF'};
    %descriptorPrefixes = {'Sc8Vn64'; 'Sc8Vb208'; 'BRIEF'; 'SURF'};
    
    %descriptorPrefixes = {'Sc8Tb64'; 'Sc8Tb208'; 'HAAR-b'; 'DCT-b'; 'RAD-b'};
    %descriptorPrefixes = {'Sc8Vn16'; 'Sc8Vn32'; 'Sc8Vn48'; 'Sc8Vn64'; 'Sc8Vn80'};
    %descriptorPrefixes = {'Sc8Vb16'; 'Sc8Vb32'; 'Sc8Vb48'; 'Sc8Vb64';
    %'Sc8Vb80'};
    
    %descriptorPrefixes = {'Sc8Vn64'; 'Sc8Vb64'; 'SURF'; 'RAD-n'; 'BRIEF'};
    
    %descriptorPrefixes = {'Sc8Vn48-r8ch3'; 'Sc8Vn72-r8ch3'; 'Sc8Tn48'; 'SURF'; 'BRIEF'}; % Sc8Tn48
    
    %descriptorPrefixes = {'Sc8Tn40-r8'; 'Sc8Tn48'; 'Sc8Tn64-r12'; 'SURF'; 'BRIEF'};
    
    % Best THERMAL-SPARSE CODEBOOKS
    %descriptorPrefixes = {'Sc8Tn40-r8'; 'Sc8Tn48-r16'; 'Sc8Tn64-r16'; 'SURF'; 'BRIEF'};
    %descriptorPrefixes = {'Sc8Tn64-r16'; 'Sc8Tn32-r8'; 'RAD-48-r16'; 'DCT'; 'HAAR'};
    
    %descriptorPrefixes = {'Sc8Vn64-r16'; 'Sc8Vn32-r8'; 'SURF'; 'BRIEF'};
    
    %descriptorPrefixes = {'Sc8Tn64-r16'; 'Sc8Tn32-r8'; 'SURF'; 'BRIEF'};
    
    % 'MM-JNT-ALL'; 'MM-SEP-THM'; 'MM-SEP-F-0.5'
    %descriptorPrefixes = {'SURF'; 'BRIEF'; 'MM-JNT-ALL'; 'MM-SEP-VIS'; 'MM-SEP-ALL'; 'MM-SEP-F-0.8'};
    
    % SURF comparison
    %descriptorPrefixes = {'SURF-TWARP-6.660000'; 'SURF-TWARP-0.000000'; 'SURF-TWARP-5.000000'};
    
    % SPARSE comparison
    %descriptorPrefixes = {'MM-JNT-ALL'; 'MM-SEP-ALL'; 'MM-SEP-VIS'; 'MM-SEP-THM'};
    
    % FUSION comparison
    %descriptorPrefixes = {'SURF-TWARP-6.660000'; 'MM-SEP-ALL'; 'MM-JNT-ALL'};
    
    % 'MM-JNT-ALL'; 
    % Sparse coding paper
    %descriptorPrefixes = {'SURF'; 'BRIEF'; 'MM-SEP-VIS'};
    %descriptorPrefixes = {'SURF'; 'BRIEF'; 'MM-SEP-ALL'; 'MM-JNT-ALL'};
    
    %descriptorPrefixes = {'SURF'; 'SURF-TWARP--1.000000'; 'SURF-TWARP--2.000000'; 'SURF-TWARP-1.000000';  'SURF-TWARP-0.000000'; 'SURF-TWARP-3.000000'};
    %descriptorPrefixes = {'SURF'; 'SURF-TWARP-1.000000'; 'SURF-TWARP-9.990000'};
    
    % 1.04 is 0.001 & 0.02
    % 1.05 is 0.001 & 0.10 % seems to be best so far, maybe try slightly
    % higher/lower B or slightly higher k...?
    % 1.06 is 0.0001 & 0.00
    % 1.07 is 0.0001 & 0.05
    % 1.08 is 0.001 & 0.05
    % 1.09 is 0.001 & 0.08
    % 1.10 is 0.003 & 0.08
    %descriptorPrefixes = {'SURF'; 'SURF-TWARP-1.090000'; 'SURF-TWARP-1.100000'};
    %descriptorPrefixes = {'SURF'; 'SURF-TWARP-1.040000'; 'SURF-TWARP-6.660000'};
    
    %descriptorPrefixes = {'SURF-TWARP-0.990000'; 'SURF'; 'SURF-TWARP-0.020000'; 'SURF-TWARP-0.100000'; 'SURF-TWARP-1.000000'; 'SURF-TWARP-3.000000'};
    % 'SURF-TWARP-1.000000'; 'SURF-TWARP-9.990000'; 
    % 'SURF-TWARP-1.060000'; 'SURF-TWARP-1.070000'
    %descriptorPrefixes = {'SURF-NND'; 'SURF-NNDR'; 'SURF-SVM'};
    
    %descriptorPrefixes = {'Sc8Vn32-r8'};
    %descriptorPrefixes = {'Sc8Vn32-r8'; 'Sc8Vn32-r8ch3'; 'Sc8Mn32-r8ch2'; 'Sc8Mn32-r8ch4'}; % pixelCount = size(F_all, 1) / visChannels;
    
    %descriptorPrefixes = {'Sc8Vn32-r8'; 'SURF'; 'BRIEF'};
    
    % BEST THERMAL-RANDOM CODEBOOKS
    %descriptorPrefixes = {'Sc8Tn64-r16'; 'Sc8Vn64-r16'; 'RAD-48-r16'; 'DCT'; 'HAAR'};
    
    %descriptorPrefixes = {'Sc8Vb64-pos'; 'Sc8Tb64-pos'; 'Sc8Vb128-pos'; 'Sc8Tb128-pos'; 'BRIEF'};
    
    %descriptorPrefixes = {'Sc8Vn64'; 'Sc8Tn64'; 'Sc8Tn64-8x8'; 'SURF';
    %'BRIEF'};
    
    %descriptorPrefixes = {'Sc8Vn64'; 'Sc8Tn64-id'; 'SURF'; 'USURF'; 'RSURF'; 'BRIEF'};
    
    %descriptorPrefixes = {'Sc8Tn64'; 'SURF'; 'BRIEF'};
    
    %descriptorPrefixes = {'Sc8Tn64'; 'HAAR-n'; 'DCT-n'; 'RAD-n'; 'Sc8Tn8'};
    
    %descriptorPrefixes = {'Sc8Tb64'; 'HAAR-b'; 'DCT-b'; 'RAD-b'};
    
    %descriptorPrefixes = { 'Sc8Vn64-0.000100' ; 'Sc8Vn64-0.000080' ; 'Sc8Vn64-0.000060' ; 'Sc8Vn64-0.000040' ; 'Sc8Vn64-0.000020' };
    %descriptorPrefixes = { 'Sc8Vb64-0.000100' ; 'Sc8Vb64-0.000080' ; 'Sc8Vb64-0.000060' ; 'Sc8Vb64-0.000040' ; 'Sc8Vb64-0.000020' };
    %descriptorPrefixes = {'Sc8Tn64'; 'NGC'; 'HAAR-n'; 'DCT-n'; 'RAD-n'}; %'NGC'; 'Sc8Tb208'; 
    %descriptorPrefixes = {'Sc8Tb64'; 'NGB'; 'HAAR-b'; 'DCT-b'; 'RAD-b'}; %'NGC'; 'Sc8Tb208'; 
    
    blankMark = {'-k '; '-.b '; '--r '; '-m '; '-.c '; '--g '};
    mark = {' ko'; ' b^'; ' rv'; ' ms'; ' cd'; ' gx'};
    fullMark = {'-ko'; '-.b^'; '--rv'; '-ms'; '-.cd'; '--gx'};
    % 'oven'; 
    
    
    % 7,3 and 7,5 causing problems...
    
    % nitrogen
    % library
    
    % Get list of environments...
    
    resultsPath = sprintf('%s/%s', dataPath, 'descresults');
    d = dir(resultsPath);
    folderCount = size(d, 1);
    
    actualFolders = {};
    
    realFolderCount = 0;
    
    for level = 1:folderCount
       if ((strcmp(d(level).name, '.') == 0) && (strcmp(d(level).name, '..') == 0))
           realFolderCount = realFolderCount + 1;
           actualFolders{realFolderCount} = d(level).name;
       end
    end
    
    realFolderCount = size(wantedEnvironments, 1);
    actualFolders = wantedEnvironments;
    
    % actualFolders
    
    %S = zeros(realFolderCount, size(descriptorPrefixes,1));
    S = zeros(res+1, size(descriptorPrefixes,1));
    
    counts = zeros(1, size(descriptorPrefixes,1));
    
    B = zeros(1, size(S,2));
    
    for env = 1:realFolderCount
        
        profilePath = sprintf('%s/%s/%s/%s', resultsPath, actualFolders{env}, modality, 'profile');
        
        for descriptor = 1:size(descriptorPrefixes,1)
            
            
            resultsName = sprintf('%s/%s.csv', profilePath, descriptorPrefixes{descriptor});
            
            currScore = 0.00;
            
            if (exist(resultsName) ~= 0)

                
                %env 
                %descriptor
                
                M = csvread(resultsName);
                
                M_smooth = smooth_pr_pts(M);
                
                %M_smooth
                
                %size(M)
                %size(M_smooth)
                %env
                %descriptor
                %plot(M_smooth);
                %R = input('continue?')
                
                Mss = subsample_descriptor_curve(M_smooth, (1.0 / res));
                
                %M_smooth(1:2,1:2)
                %Mss(1:2,1:2)
                
                [X P A E] = pr_score(M);
                
                qq = A;
                
                if (qq > currScore)
                    currScore = qq;
                end
                
                B(1,descriptor) = B(1,descriptor) + currScore;
                
                S(:,descriptor) = S(:,descriptor) + Mss(:,2);
                counts(1, descriptor) = counts(1, descriptor) + 1;
                
            end

            %S(env, descriptor) = currScore;
            
            %R = input('continue?')
            
        end
    end
    
    counts = counts
    
    H = figure; %(1);
    
    %grid on;
    ylabel('Recall');
    xlabel('1 - Precision');
    hold on;
    
    axis([0 breadth 0.0 1.0]);
      
    x_vals = linspace(0, breadth, 11);
    y_vals = linspace(0, 1.0, 11);
    
    draw_gray_grid(x_vals, y_vals);
    
    xlhand = get(gca,'xlabel');
    set(xlhand,'fontsize',16);
    ylhand = get(gca,'ylabel');
    set(ylhand,'fontsize',16);
    
    G = gca;
    set(G,'gridlinestyle','-');
    
    pause(0.1);
    
    legendShown = 0;
    
    interpFactor = 1;

    
    for descriptor = 1:size(descriptorPrefixes,1)
        
        if (counts(1, descriptor) > 0)
            S(:,descriptor) = S(:,descriptor) ./ counts(1, descriptor);
            B(1,descriptor) = B(1,descriptor) ./ counts(1, descriptor);
            
            P = plot(xValues, S(:,descriptor), blankMark{descriptor}, 'LineWidth', line_width, 'MarkerSize', marker_size);
            
            for iii = 1:size(xValues,2)-interpFactor
                if ((interpFactor == 1) || (mod(iii,interpFactor) == 1))
                    P = plot([xValues(iii) xValues(iii+interpFactor)], [S(iii,descriptor) S(iii+interpFactor,descriptor)], mark{descriptor}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                end
            end
            
            P = plot([xValues(1) xValues(1)], [S(1,descriptor) S(1,descriptor)], fullMark{descriptor}, 'LineWidth', line_width, 'MarkerSize', marker_size);

            desc_name = descriptorPrefixes{descriptor};
            
            if (strcmp(desc_name, 'MM-SEP-VIS') > 0)
               desc_name = 'VISIBLE';
            elseif  (strcmp(desc_name, 'SURF-TWARP--1.000000') > 0)
               desc_name = 'FUSED-MIN';  
            elseif  (strcmp(desc_name, 'SURF-TWARP--2.000000') > 0)
                desc_name = 'FUSED-MAX';  
            elseif  (strcmp(desc_name, 'SURF-TWARP-1.000000') > 0)
                desc_name = 'FUSED-0.50';  
            elseif  (strcmp(desc_name, 'SURF-TWARP-0.000000') > 0)
                desc_name = 'SURF-VISIBLE';  
            elseif  (strcmp(desc_name, 'SURF-TWARP-3.000000') > 0)
                desc_name = 'THERMAL-0.75';  
            elseif (strcmp(desc_name, 'SURF-TWARP-5.000000') > 0)
                desc_name = 'SURF-THERMAL';
            elseif (strcmp(desc_name, 'SURF-TWARP-6.660000') > 0)
                desc_name = 'SURF-LF';
            elseif (strcmp(desc_name, 'SURF-TWARP-1.070000') > 0)
                desc_name = 'SURF-FUSION';
            elseif (strcmp(desc_name, 'MM-JNT-ALL') > 0)
                desc_name = 'EARLY-FUSION';
            elseif (strcmp(desc_name, 'MM-SEP-THM') > 0)
                desc_name = 'THERMAL';
            elseif (strcmp(desc_name, 'SIFTX') > 0)
                desc_name = 'SIFT';
            elseif (strcmp(desc_name, 'GLOHX') > 0)
                desc_name = 'GLOH';
            elseif (strcmp(desc_name, 'PCAX') > 0)
                desc_name = 'PCA-SIFT';
            elseif (strcmp(desc_name, 'SCX') > 0)
                desc_name = 'Shape Context';
            elseif (strcmp(desc_name, 'BRIEF') > 0)
                desc_name = 'Brief';
            elseif (strcmp(desc_name, 'MM-SEP-VIS-TWARP-1.070000') > 0)
                desc_name = 'LATE-FUSION'
            end
            
            if (legendShown == 0)
                L = legend(P, desc_name, 'Location', 'Best', 'Orientation', 'vertical');
                legendShown = 1;
            else
                hold on
                legend('show');
                [LEGH,OBJH,OUTH,OUTM] = legend;
                L = legend([OUTH;P],OUTM{:}, desc_name, 'Location', 'Best', 'Orientation', 'vertical');
            end
            

            set(L,'FontSize',10);
            
        end
        
        
        
    end
    
    filename = sprintf('%s/%s-descriptor-summary.pdf', output_dir, modality);
    
    
    save_to_file_if_possible(H, filename);
    
    
end