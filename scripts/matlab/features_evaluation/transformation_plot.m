function transformation_plot(location, modality, transformation, mode, measure)
    %
    % Generates transformation plots and other results

    % measure = 0 is repeatability, measure = 1 is matchability
    if (nargin < 5)
        measure = 0;
    end

    if (measure == 0)
        resultsType = 'results';
    else
        resultsType = 'match_results';
    end

    % mode = 0 is same-level, mode = 1 is base-ref

    dataPath = get_path_to_data();
    
    eval('plot_settings');


    % customizable settings
    % =====================
    %detectorPrefixes = {'SURF'; 'STAR'; 'FAST'; 'ORB'; 'MSR'; 'HAF'; 'RAND'}; %; 'rns']; 'hes' 'rnd' 'har'? 'orb'; 
    
    length = 20;
    % =====================
    
    format shortG

    close all;
    
    disp('Initialization...');
    
    set(0,'defaultaxesfontsize',12);
    
    
    d_mark = {'-k '; '-.b '; '--r '; '-m '; '-.c '; '--g '};
    %mark = {' ko'; ' b^'; ' rv'; ' ms'; ' cd'; ' gx'};
    mark = {'-ko'; '-.b^'; '--rv'; '-ms'; '-.cd'; '--gx'};
    
    

    %mark = {'-bo'; ':rx'; '-.g+'; '--cs'; '-yd'; ':kv'};
    
    %mark=['-b ';'-r ';'-g ';'-m ';'-c ';'-y '; '-k ']; % also m
    %d_mark = ['--b ';'--r ';'--g ';'--m ';'--c ';'--y '; '--k '];
    
    folderPath = sprintf('%s/%s/%s/%s/%s/*', dataPath, resultsType, location, modality, transformation);
    d = dir(folderPath);
    folderCount = size(d, 1);
    
    actualFolders = {};
    
    realFolderCount = 0;
    
    for level = 1:folderCount
        d(level).name
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
               
               %realFolderCount = realFolderCount + 1;
               %actualFolders{realFolderCount} = d(level).name;
           end
       end
    end
    
    actualFolders
    
    folderName = sprintf('%s/analysis/%s', dataPath, transformation);
    mkdir(folderName);
              
    disp('Commencing algorithm.');
    
    H = figure(1)
    
    %grid on;
    if (measure == 0)
        ylabel('Repeatability');
    else
        ylabel('Matchability');
    end
    xlabel('Transformation Level');
    hold on;
    
    axis_ = linspace(1, realFolderCount, realFolderCount);
    G = zeros(size(detectorPrefixes,1), realFolderCount);
    
    realFolderCount
    x_vals = linspace(1, realFolderCount, realFolderCount);
    y_vals = linspace(0, 1.0, 11);
    
    draw_gray_grid(x_vals, y_vals);
      
    xlhand = get(gca,'xlabel');
    set(xlhand,'fontsize',16);
    ylhand = get(gca,'ylabel');
    set(ylhand,'fontsize',16);
    
    %G = gca;
    %set(G,'gridlinestyle','-');
    
    pause(0.1)
          
    folderCount = 20; % really want to actually count folders
    
    
    %axis([1 realFolderCount 0 1.0]);
    
    %actualFolders;
    
    legendShown = 0;
    
    minLevel = 99;
    maxLevel = 0;
    
    for folder = 1:realFolderCount
        
        %disp(sprintf('Folder: %s', actualFolders{folder}));
        
        for detector = 1:size(detectorPrefixes,1)
        
            %disp(sprintf('Detector: %s', detectorPrefixes{detector}));

            resultsName = sprintf('%s/%s/%s/%s/%s/%s/%s.csv', dataPath, resultsType, location, modality, transformation, actualFolders{folder}, detectorPrefixes{detector});

            %disp(resultsName)

            if (exist(resultsName) == 0)
                %disp('result doesnt exist...');
                G(detector, folder) = 0.00;
            else

                M = 0;

                M = csvread(resultsName);

                N(folder,1) = folder;
                N(folder,2:4) = M(1,2:4);
                
                G(detector, folder) = min(M(1, 2), 0.995);
                
                %R = input('continue?')

            end
            
            %G(detector, folder)
            %R = input('continue?')
            
        end
    end
    
    %G

    for detector = 1:size(detectorPrefixes,1)
        
        anyMarked = 0;
        
        % Only want to plot nonzero values (i.e. when the detector had
        % enough points)
        
        for ggg = 1:size(G, 2)-1
                
            if ((G(detector,ggg) > 0.00) || (G(detector,ggg+1) > 0.00))
                P = plot(axis_(ggg:ggg+1), G(detector,ggg:ggg+1), mark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                anyMarked = 1;
            end

            

        end
        
        
        if (anyMarked > 0)
            if (legendShown == 0)
                L = legend(P, detectorPrefixes{detector},  'Location', 'Best', 'Orientation', 'vertical');
                legendShown = 1;

            else
                hold on
                legend('show')
                [LEGH,OBJH,OUTH,OUTM] = legend;
                L = legend([OUTH;P],OUTM{:}, detectorPrefixes{detector}, 'Location', 'Best', 'Orientation', 'vertical');

            end
        end
        
        set(L,'FontSize',10);
        
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
    
    %legend_best_fit(P);
    
    %axis([minLevel maxLevel 0 1]);
    
    if (measure == 0)
        if (mode == 0)
            filename = sprintf('%s/analysis/%s/%s-%s-%s-SL-rpt.pdf', dataPath, transformation, location, modality, transformation);
        else
            filename = sprintf('%s/analysis/%s/%s-%s-%s-BR-rpt.pdf', dataPath, transformation, location, modality, transformation);
        end
    else
        if (mode == 0)
            filename = sprintf('%s/analysis/%s/%s-%s-%s-SL-mth.pdf', dataPath, transformation, location, modality, transformation);
        else
            filename = sprintf('%s/analysis/%s/%s-%s-%s-BR-mth.pdf', dataPath, transformation, location, modality, transformation);
        end
    end
    
    save_to_file_if_possible(H, filename);

    % Find summary statistics / characteristic sensitivty/count [actually
    % can do this during the main loop, probably...]

end

