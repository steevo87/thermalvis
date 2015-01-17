function pr_plot_transformation(location, modality, transformation, mode)
% Generates transformation plots

    dataPath = get_path_to_data();
    
    eval('plot_settings');
    
    plotVar = 4; % (1 : Score); (2 : Product); (3 : Area); (4 : EER)

% mode: 0 = same-level - plotting score
% mode: 1 = base-ref - plotting score
% mode: 2 = maybe doing full pr plots between two levels or within a single
% level?

    % customizable settings
    % =====================
    % Patt Rec:
    %descriptorPrefixes = {'SURF'; 'SIFT'; 'SmTn64'; 'BRIEF'; 'SpTn64'}; %; 'NGC'};
    
    %descriptorPrefixes = {'Sc8Tn64'; 'Sc8Tb64'; 'SURF'; 'RAD-n'; 'BRIEF'};
    
    %descriptorPrefixes = {'SURF'; 'SIFT'; 'ORB'; 'BRIEF'};
    %descriptorPrefixes = {'Sc8Tn64-r16'; 'Sc8Tn32-r8'; 'SURF'; 'BRIEF'};
    %descriptorPrefixes = {'Sc8Tn64-sup8'; 'Sc8Tn64-sup24'; 'Sc8Tn64-sup40'; 'Sc8Tn64-sup64'};
    
    % All major floating point dictionaries
    %descriptorPrefixes = {'SURF'; 'SpVn64'; 'SpTn64'; 'SpVn208'; 'SpTn208'};
    
    % Sparse coding:
    %descriptorPrefixes = {'SURF'; 'ORB'; 'SPN'; 'SPB'; 'NGC'; 'NGB'};
       
    format shortG

    close all;
    
    disp('Initialization...');
    
    set(0,'defaultaxesfontsize',12);
    
    %mark = {'-bo'; ':r^'; '-.gv'; '--cs'; '-md'; ':yx'};
    
    %mark = {'-bo'; ':rx'; '-.g+'; '--cs'; '-md'; ':kv'};
    
    %mark=['xb ';'xr ';'xg ';'xm ';'xc ';'xy '; 'xk ']; % also m
    %mark=['-b ';'-r ';'-g ';'-m ';'-c ';'-y '; '-k ']; % also m
    %mark = mark(1:size(descriptorPrefixes, 1), :);
    
    d_mark = {'-k '; '-.b '; '--r '; '-m '; '-.c '; '--g '};
    %mark = {' ko'; ' b^'; ' rv'; ' ms'; ' cd'; ' gx'};
    mark = {'-ko'; '-.b^'; '--rv'; '-ms'; '-.cd'; '--gx'};
    
    %d_mark = ['--b ';'--r ';'--g ';'--c ';'--m ';'--k '];
    d_mark = d_mark(1:size(descriptorPrefixes, 1), :);
    
       
    folderName = sprintf('%s/analysis/desc_profiling', dataPath);
    mkdir(folderName);
    
    folderPath = sprintf('%s/descresults/%s/%s/%s/*', dataPath, location, modality, transformation);
    d = dir(folderPath);
    folderCount = size(d, 1);
    
    actualFolders = {};
    
    realFolderCount = 0;
    
    for level = 1:folderCount
       if (strcmp(d(level).name, '.') == 0)
           if (strcmp(d(level).name, '..') == 0)
               
               
               if (mode == 0)
                   % Ignores 0000 folder which is only for base-ref
                   if (strcmp(d(level).name, '0000') == 0)
                       % Ignores BR ones (obviously just for base-ref)
                       if (strcmp(d(level).name(end-1:end), 'BR') == 0)
                           realFolderCount = realFolderCount + 1;
                           actualFolders{realFolderCount} = d(level).name;
                       end
                   end
               else
                   if (strcmp(d(level).name(end-1:end), 'BR') ~= 0)
                    realFolderCount = realFolderCount + 1;
                    actualFolders{realFolderCount} = d(level).name;
                   end
               end
               
               
           end
       end
    end
    
    %actualFolders
    
    % R = input('continue?')
    
    disp('Commencing algorithm.');
    
    H = figure(1);
    
    %grid on;
    ylabel('Precision-Recall Score');
    xlabel('Transformation Level');
    hold on;
    
    x_vals = linspace(1, realFolderCount, realFolderCount);
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
    
    axis_ = linspace(1, realFolderCount, realFolderCount);
    G = zeros(size(descriptorPrefixes,1), realFolderCount);
    
    size(axis_)
    size(G)
    
    %plot(axis_, G);
    
    realFolderCount = realFolderCount;
    
    axis([1 realFolderCount 0 1.0]);
    
    
    
    %R = input('continue?')
          
    % ===== FOR EACH FEATURE DETECTOR =====
    for folder = 1:realFolderCount
        
        disp(sprintf('Folder: %s', actualFolders{folder}));
        
        for descriptor = 1:size(descriptorPrefixes,1)
        
            %disp(sprintf('Descriptor: %s', descriptorPrefixes{descriptor}));

            resultsName = sprintf('%s/descresults/%s/%s/%s/%s/%s.csv', dataPath, location, modality, transformation, actualFolders{folder}, descriptorPrefixes{descriptor});

            %disp(resultsName)

            if (exist(resultsName) == 0)
                %folder
                disp('result doesnt exist...');
                %G
            else

                M = 0;

                M = csvread(resultsName);

                [S P A E] = pr_score(M);
            
                Q = [S P A E];
                
                

                G(descriptor, folder) = Q(plotVar);

                
                

            end
            %R = input('continue?')
        end
    end

    
    for descriptor = 1:size(descriptorPrefixes,1)
        %P = plot(axis_, G(descriptor,:), mark(descriptor,:), 'LineWidth', 2, 'MarkerSize', 2);
        
        anyMarked = 0;
        
        % Only want to plot nonzero values (i.e. when the detector had
        % enough points)
        
        for ggg = 1:size(G, 2)-1
                
            %if ((G(descriptor,ggg) > 0) && (G(descriptor,ggg+1) > 0))
                P = plot(axis_(ggg:ggg+1), G(descriptor,ggg:ggg+1), mark{descriptor}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                anyMarked = 1;
            %end

            

        end
        
        desc_name = descriptorPrefixes{descriptor};
        
        if (strcmp(desc_name, 'SIFTX') > 0)
            desc_name = 'SIFT';
        elseif (strcmp(desc_name, 'GLOHX') > 0)
            desc_name = 'GLOH';
        elseif (strcmp(desc_name, 'PCAX') > 0)
            desc_name = 'PCA-SIFT';
        elseif (strcmp(desc_name, 'SCX') > 0)
            desc_name = 'Shape Context';
        elseif (strcmp(desc_name, 'BRIEF') > 0)
            desc_name = 'Brief';
        end
        
        if (anyMarked > 0)
            if (legendShown == 0)
                L = legend(P, desc_name, 'Location', 'SouthEast', 'Orientation', 'vertical');
                legendShown = 1;
            else
                hold on
                legend('show')
                [LEGH,OBJH,OUTH,OUTM] = legend;
                L = legend([OUTH;P],OUTM{:}, desc_name, 'Location', 'SouthEast', 'Orientation', 'vertical');
            end
            
            set(L,'FontSize',10);
        end
        
    end
    
    axis([1 realFolderCount 0.0 1.0])
    %axis([1 realFolderCount 0.0 0.1*ceil(10*max(G(:)))])
    set(gca,'XTick',[1:1:realFolderCount])
    
    
    
    if (strcmp(transformation,'tod') > 0)
        set(gca,'XTicklabel',{'0900', '', '1100', '', '1300', '', '1500', '', '1700', '', '1900', '', '2100'})
        xlabel('24-hour timestamp');
    elseif (strcmp(transformation,'ofb') > 0)
        set(gca,'XTicklabel',{'-4', '-3', '-2', '-1', '0', '1', '2'})
        xlabel('Focus level');
    elseif (strcmp(transformation,'nuc') > 0)
        set(gca,'XTicklabel',{'0', '30', '60', '90', '120', '150', '180', '210', '240', '270', '300'})
        xlabel('Time since NUC (s)');
    end
    
    xlhand = get(gca,'xlabel');
    set(xlhand,'fontsize',16);
    
    folder = sprintf('%s/analysis/descriptor_transformations', dataPath);
    mkdir(folder);
    folder = sprintf('%s/analysis/descriptor_transformations/%s', dataPath, transformation);
    mkdir(folder);
    
    if (mode == 0)
        filename = sprintf('%s/analysis/descriptor_transformations/%s/%s-%s-%s-SL.pdf', dataPath, transformation, location, modality, transformation);
    else
        filename = sprintf('%s/analysis/descriptor_transformations/%s/%s-%s-%s-BR.pdf', dataPath, transformation, location, modality, transformation);
    end
       
    save_to_file_if_possible(H, filename);
        
    % Find summary statistics / characteristic sensitivty/count [actually
    % can do this during the main loop, probably...]

end