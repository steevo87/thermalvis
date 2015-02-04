function [S B] = detectors_profiling_summary(modality, mode)
    dataPath = get_path_to_data();
    
    line_width = 2;
    marker_size = 8;
    
    if (nargin < 2)
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
    
    detectorPrefixes = {'SURF'; 'ORB'; 'STAR'; 'FAST'; 'MSER'; 'GFTT'}; %  'STAR'; 'FAST'; 'ORB'; 
    blankMark = {'-k '; '-.b '; '--r '; '-m '; '-.c '; '--g '};
    mark = {' ko'; ' b^'; ' rv'; ' ms'; ' cd'; ' gx'};
    fullMark = {'-ko'; '-.b^'; '--rv'; '-ms'; '-.cd'; '--gx'};
    
    wantedEnvironments = {'aquavist'; 'balcony'; 'pipes'; 'nitrogen'; 'driveway'; 'pavement'; 'office'; 'library'; 'desk'};
    
    % Get list of environments...
    
    resultsPath = sprintf('%s/%s', dataPath, resultsType);
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
    
    countsVec = [20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100 110 120 130 140 150 160 170 180 190 200 250 300];
    S = zeros(size(countsVec,2), size(detectorPrefixes,1));
    Sc = zeros(size(S));
    
    for env = 1:realFolderCount
        
        profilePath = sprintf('%s/%s/%s/%s', resultsPath, actualFolders{env}, modality, subaddr);
        
        for detector = 1:size(detectorPrefixes,1)
            
            
            resultsName = sprintf('%s/%s.csv', profilePath, detectorPrefixes{detector});
            

            if (exist(resultsName) ~= 0)
                
                M = csvread(resultsName);
                
                %env
                %detectorPrefixes{detector}
                
                %M
                
                %R = input('continue?')
                
                for sss = 1:size(countsVec, 2)
                    
                    closestCount = 0;
                    currScore = -1;
                    
                    %maxAllowableDiff = min(max(2.5, 0.1*countsVec(sss)), 10);
                    
                    if (sss == 1)
                        maxAllowableDiff = 5;
                    elseif (sss == size(countsVec,2))
                        maxAllowableDiff = 50;
                    else
                        maxAllowableDiff = min(abs(countsVec(sss-1) - countsVec(sss)), abs(countsVec(sss+1) - countsVec(sss)));
                    end
                    
                    maxAllowableDiff = 100;
                    
                    for iii = 1:size(M, 1)
                        if (abs(M(iii,1) - countsVec(sss)) < maxAllowableDiff)
                            if (abs(M(iii,1) - countsVec(sss)) < abs(closestCount - countsVec(sss)))
                                closestCount = M(iii,1);
                                currScore = M(iii,2);
                            end
                        end
                    end
                    
                    if (currScore > -1)
                        S(sss, detector) = S(sss, detector) + currScore;
                        Sc(sss, detector) = Sc(sss, detector) + 1;
                    end
                    
                    
                end
                
                
            else
                %disp(strcat(resultsName, ' doesnt exist...'));
            end

            
            
            %R = input('continue?')
            
        end
    end
    
Sc

    B = zeros(1, size(S,2));
    
    for detector = 1:size(detectorPrefixes,1)
        
        count = 0;
        
        for sss = 1:size(countsVec, 2)
        
            if (Sc(sss, detector) > 0)
                S(sss, detector) = S(sss, detector) ./ Sc(sss, detector);
                count = count + 1;
                B(detector) = B(detector) + S(sss, detector);
            end
        end
        
        B(detector) = B(detector) / count;
        
    end
    
    size(S)
    
    %set(0,'defaultaxesfontsize',12);
    
    %mark = {'-bo'; ':rx'; '-.g+'; '--cs'; '-md'; ':yv'};
    
    H = figure(1);
    
    %grid on;
    if (mode == 0)
        ylabel('Repeatability');
    else
        ylabel('Matchability');
    end
    xlabel('Feature Count');
    hold on;
    
    
    maxFeatures = 250;
    axisArray = [0 250 0 1];
    
    % Draw manual grid-lines
    y_vals = linspace(0, 1, 11);
        
    x_hor = [0 maxFeatures];
    y_hor = [1 1];
    plot(x_hor, y_hor, 'k');
    
    grey = [0.7,0.7,0.7];
    
    for iii = 2:size(y_vals,2)-1
        y_hor = [y_vals(iii) y_vals(iii)];
        plot(x_hor, y_hor, 'Color', grey, 'LineWidth', 0.2, 'LineStyle', '--');
    end
    
    x_vals = linspace(0, maxFeatures, 6);
    
    x_ver = [maxFeatures maxFeatures];
    y_ver = [0 1];
    plot(x_ver, y_ver, 'k');
    
    grey = [0.7,0.7,0.7];
    
    for iii = 2:size(x_vals,2)-1
        x_ver = [x_vals(iii) x_vals(iii)];
        plot(x_ver, y_ver, 'Color', grey, 'LineWidth', 0.2, 'LineStyle', '--');
    end
    
    axis(axisArray);
      
     xlhand = get(gca,'xlabel');
    set(xlhand,'fontsize',16);
    ylhand = get(gca,'ylabel');
    set(ylhand,'fontsize',16);
    
    G = gca;
    set(G,'gridlinestyle','-');
    
    pause(0.1);
    
    legendShown = 0;
    
    minForInterp = 10;
    
    for detector = 1:size(detectorPrefixes,1)
        
        prevVal = countsVec(1);
        
        maxIndex = 0;
        
        
        
        for iii = 2:size(S,1)
            if (Sc(iii, detector) == max(max(Sc(:, detector))))
                
                maxIndex = iii;
                %P = plot(countsVec(iii:iii+1), (S(iii:iii+1,detector))', fullMark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                
                range = [];
                
                if (iii > 1)
                   range = [range S(iii-1,detector)]; 
                end
                
                range = [range S(iii,detector)];
                
                if (iii < size(S,1))
                    range = [range S(iii+1,detector)];
                end
                
                
                
                if ((S(iii,detector) == min(range)) || (S(iii,detector) == max(range)) || (iii == size(S,1)))
                    P = plot([countsVec(1) countsVec(iii)], [S(1,detector) S(iii,detector)], mark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                    P = plot([countsVec(1) countsVec(1)], [S(1,detector) S(1,detector)], fullMark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                end
                
                if (abs(prevVal - countsVec(iii)) > minForInterp)
                    %prevVal = countsVec(iii+1);
                    %plot([countsVec(1) countsVec(iii+1)], [S(1,detector) S(iii,detector)], mark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
                end
            end

            
            
               
            %R = input('continue?')
        end
        
        if (maxIndex > 0)
            %P = plot(countsVec(1:maxIndex), (S(1:maxIndex, detector))', fullMark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
            plot(countsVec(1:maxIndex), (S(1:maxIndex, detector))', blankMark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
            
        end
        
        %if (detector == 7)
        %    P = semilogx(countsVec(1:8), (S(1:8,detector))', mark{detector}, 'LineWidth', 2, 'MarkerSize', 6);
        %else
        %    P = semilogx(countsVec, (S(:,detector))', mark{detector}, 'LineWidth', 2, 'MarkerSize', 6);
        %end
        
        
        %P = plot(countsVec(1:1), (S(1:1,detector))', fullMark{detector}, 'LineWidth', line_width, 'MarkerSize', marker_size);
        
        if (legendShown == 0)
            L = legend(P, detectorPrefixes{detector}, 'Location', 'Best', 'Orientation', 'vertical');
            legendShown = 1;
        else
            hold on
            legend('show')
            [LEGH,OBJH,OUTH,OUTM] = legend;
            L = legend([OUTH;P],OUTM{:}, detectorPrefixes{detector}, 'Location', 'Best', 'Orientation', 'vertical');
        end
        
        set(L,'FontSize',10);
        
    end
    
    %countsVec
    
    %set(gca,'GridLineStyle','--');
    
    %set(gca,'XColor',[0.8,0.8,0.8]);
    
    if (mode == 0)
        filename = sprintf('%s/analysis/%s-detector-summary-rpt.pdf', dataPath, modality);
    else
        filename = sprintf('%s/analysis/%s-detector-summary-mth.pdf', dataPath, modality);
    end
    
    save_to_file_if_possible(H, filename);
    
end