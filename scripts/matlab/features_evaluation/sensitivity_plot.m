function sensitivity_plot(location, modality, mode)
% Generates sensitivity plots and other results

    if (nargin < 3)
        mode = 0;
    end

    % mode == 0 : repeatability
    % mode == 1 : matchability
    
    if (mode == 0)
        resultsType = 'results';
    else
        resultsType = 'match_results';
    end

    subaddr = '';

    if (strcmp(modality, 'x-modality'))
        subaddr = 'visible-thermal';
    else
        subaddr = 'profile' 
    end
    
    dataPath = get_path_to_data();

    % customizable settings
    % =====================
    
    if (strcmp(modality, 'x-modality'))
        detectorPrefixes = {'SURF'; 'STAR'; 'FAST'; 'ORB'; 'MSER'}; %; 'RAND'}
    else
        detectorPrefixes = {'SURF'; 'STAR'; 'FAST'; 'ORB'; 'MSER'}; %; 'RAND'}; %; 'rns']; 'HAF' 'hes' 'rnd' 'har'? 'orb'; 
    %detectorPrefixes = {'SURF'; 'GFTT'; 'SIFT'; 'MSER'; 'HARRIS'; 'RAND'}
    end
    
    
    length = 40;
    max_count_for_plot = 250;
    
    sensitivityCounts = zeros(size(detectorPrefixes,1), length, 2);
    
    %format shortG

    %close all;
    
    H = figure;
    
    disp('Initialization...');
    
    set(0,'defaultaxesfontsize',12);
    
    blankMark = {'-b '; ':r '; '-.g '; '--c '; '-m '; ':y '};
    mark = {' bo'; ' rx'; ' g+'; ' cs'; ' md'; ' yv'};
    fullMark = {'-bo'; ':rx'; '-.g+'; '--cs'; '-md'; ':yv'};
    
    %mark=['-b ';'-r ';'-g ';'-m ';'-c ';'-y '; '-k ']; % also m
    %mark = mark(1:size(detectorPrefixes, 1), :);
    
    d_mark = ['--b ';'--r ';'--g ';'--m ';'--c ';'--y '; '--k '];
    d_mark = d_mark(1:size(detectorPrefixes, 1), :);
    
       
    folderName = sprintf('%s/analysis/profiling', dataPath);
    mkdir(folderName);
    
    disp('Commencing algorithm.');
    
    
    
    grid on;
    
    if (mode == 0)
        ylabel('Repeatability');
    else
        ylabel('Matchability');
    end
    xlabel('Feature Count');
    hold on;
    
    axisArray = [0 max_count_for_plot 0 1];
    
    axis(axisArray);
      
    xlhand = get(gca,'xlabel');
    set(xlhand,'fontsize',16);
    ylhand = get(gca,'ylabel');
    set(ylhand,'fontsize',16);
    
    G = gca;
    set(G,'gridlinestyle','-');
    
    pause(0.1);
    
    legendShown = 0;
    
    maxCount = 0;
    
    S = [];
    
    minForInterp = 10;
          
    % ===== FOR EACH FEATURE DETECTOR =====
    for detector = 1:size(detectorPrefixes,1)
        
        disp(sprintf('Detector: %s', detectorPrefixes{detector}));
        
        resultsName = sprintf('%s/%s/%s/%s/%s/%s.csv', dataPath, resultsType, location, modality, subaddr, detectorPrefixes{detector});
        
        disp(resultsName)
        
        if (exist(resultsName) == 0)
            disp('result doesnt exist...');
        else
            
            M = csvread(resultsName);
            
            if (max(M(:,1)) > maxCount)
               maxCount = max(M(:,1)); 
            end
            
            axisArray = [0 min(max_count_for_plot, maxCount) 0 1];
    
            axis(axisArray);
            
            %P = plot(M(:,1), M(:,3), d_mark(detector,:), 'LineWidth', 1, 'MarkerSize', 1);
            %P = plot(M(:,1), M(:,4), d_mark(detector,:), 'LineWidth', 1, 'MarkerSize', 1);
            
            if (size(M, 1) > 1)
                
                prevVal = M(1);
                
                for iii = 1:size(M,1)-1
                    P = plot(M(iii:iii+1,1), M(iii:iii+1,2), blankMark{detector}, 'LineWidth', 2, 'MarkerSize', 6);
                    
                    
                    if (abs(prevVal - M(iii+1, 1)) > minForInterp)
                        prevVal = M(iii+1, 1);
                        plot([M(1,1) M(iii+1, 1)], [M(1,2) M(iii+1,2)], mark{detector}, 'LineWidth', 2, 'MarkerSize', 6);
                    end
                    
                end
                
                
                %P = semilogx(M(:,1), M(:,2), mark{detector}, 'LineWidth', 2, 'MarkerSize', 6);
                
                P = plot(M(1:1,1), M(1:1,2), fullMark{detector}, 'LineWidth', 2, 'MarkerSize', 6);
                
                if (legendShown == 0)
                    legend(P, detectorPrefixes{detector}, 'Location', 'NorthOutside', 'Orientation', 'horizontal');
                    legendShown = 1;
                else
                    hold on
                    legend('show')
                    [LEGH,OBJH,OUTH,OUTM] = legend;
                    legend([OUTH;P],OUTM{:}, detectorPrefixes{detector}, 'Location', 'NorthOutside', 'Orientation', 'horizontal');
                end
                
                

                
            end

            


        end
    end
    
    if (mode == 0)
        filename = sprintf('%s/analysis/profiling/%s-%s-%s-rpt.pdf', dataPath, location, modality, subaddr);
    else
        filename = sprintf('%s/analysis/profiling/%s-%s-%s-mth.pdf', dataPath, location, modality, subaddr);
    end
       
    save_to_file_if_possible(H, filename);
        
    % Find summary statistics / characteristic sensitivty/count [actually
    % can do this during the main loop, probably...]

end