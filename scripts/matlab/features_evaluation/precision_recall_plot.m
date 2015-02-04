function precision_recall_plot(location, modality, transformation, level)
% Generates pr-plots and other results

    close all;
    
    line_width = 0.5;
    marker_size = 4;
    
    breadth = 0.25;

    dataPath = get_path();
    
    res = 10 * (1 / breadth);

    xValues = linspace(0, 1.00, res+1);
    
    desiredCount = 300;
    maxAllowableDiff = 10;

    profileMode = 0;

    if (nargin < 3)
        profileMode = 1;
    end
    
    % customizable settings
    % =====================
    descriptorPrefixes = {'SURF'; 'SIFT'; 'ORB'; 'BRIEF'; 'MSER'};
    
    length = res;
    
    profileResults = zeros(size(descriptorPrefixes,1), length, 2);
    
    format shortG

    
    
    disp('Initialization...');
    
    set(0,'defaultaxesfontsize',12);
    
    blankMark = {'-b '; ':r '; '-.m '; '--c '; '-g '; ':k '};
    mark = {' bo'; ' r^'; ' mv'; ' cs'; ' gd'; ' kx'};
    fullMark = {'-bo'; ':r^'; '-.mv'; '--cs'; '-gd'; ':kx'};
    
    %mark=['xb ';'xr ';'xg ';'xm ';'xc ';'xy '; 'xk '];
    %mark=['-b ';'-r ';'-g ';'-m ';'-c ';'-y '; '-k ';];
    %mark = mark(1:size(descriptorPrefixes, 1), :);
    
    %d_mark = ['--b ';'--r ';'--g ';'--m ';'--c ';'--y '; '--k '];
    %d_mark = d_mark(1:size(descriptorPrefixes, 1), :);
    
       
    folderName = sprintf('%s/analysis/desc_profiling', dataPath);
    mkdir(folderName);
    %folderName = sprintf('../analysis/%s', location);
    %mkdir(folderName);
    %folderName = sprintf('../analysis/%s/%s', location, modality);
    %mkdir(folderName);
    
    disp('Commencing algorithm.');
    
    H = figure; %(1);
    
    grid on;
    ylabel('Recall');
    xlabel('1 - Precision');
    hold on;
    
    axis([0 breadth 0.0 1.0]);
      
    xlhand = get(gca,'xlabel');
    set(xlhand,'fontsize',16);
    ylhand = get(gca,'ylabel');
    set(ylhand,'fontsize',16);
    
    G = gca;
    set(G,'gridlinestyle','-');
    
    pause(0.1);
    
    legendShown = 0;
    
    interpFactor = 1;
          
    % ===== FOR EACH FEATURE DETECTOR =====
    for descriptor = 1:size(descriptorPrefixes,1)
        
        disp(sprintf('Descriptor: %s', descriptorPrefixes{descriptor}));
        
        if (profileMode == 1)
            resultsName = sprintf('%s/descresults/%s/%s/profile/%s.csv', dataPath, location, modality, descriptorPrefixes{descriptor});
        else
            resultsName = sprintf('%s/descresults/%s/%s/%s/%s/%s.csv', dataPath, location, modality, transformation, level, descriptorPrefixes{descriptor});
        end
        
        disp(resultsName)
        
        if (exist(resultsName) == 0)
            disp('result doesnt exist...');
        else
            
            M = 0;
            
            s = dir(resultsName);
            
            if (s.bytes == 0)
                break;
            end
            
            M = csvread(resultsName);
            
            %Z = smooth(M(:,1), M(:,2), 20);
            
            %M(:,2) = Z;
            
            M_smooth = smooth_pr_pts(M);
            
            Mss = subsample_descriptor_curve(M_smooth, (1.0 / res));
            M = Mss;
            %M = subsample_descriptor_curve(M, 0.02);
            
            %M;
            
            % M(:, 1) = 1 - M(:, 1);
            
            % R = input('continue?')
            
            %P = plot(M(:,1), M(:,3), d_mark(detector,:), 'LineWidth', 1, 'MarkerSize', 1);
            %P = plot(M(:,1), M(:,4), d_mark(detector,:), 'LineWidth', 1, 'MarkerSize', 1);

            %P = plot(M(:,1), M(:,2), mark(detector,:), 'LineWidth', 2, 'MarkerSize', 5);
            
            P = plot(M(:,1), M(:,2), blankMark{descriptor}, 'LineWidth', line_width, 'MarkerSize', 6);
            
            for iii = 1:size(M,1)-interpFactor
                if ((interpFactor == 1) || (mod(iii,interpFactor) == 1))
                    P = plot([M(iii, 1) M(iii+interpFactor, 1)], [M(iii, 2) M(iii+interpFactor,2)], mark{descriptor}, 'LineWidth', line_width, 'MarkerSize', 6);
                end
            end
            
            P = plot([M(1,1) M(1,1)], [M(1,2) M(1,2)], fullMark{descriptor}, 'LineWidth', line_width, 'MarkerSize', marker_size);

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
            
            if (legendShown == 0)
                L = legend(P, desc_name, 'Location', 'NorthOutside', 'SouthEast', 'vertical');
                legendShown = 1;
            else
                hold on
                legend('show')
                [LEGH,OBJH,OUTH,OUTM] = legend;
                L = legend([OUTH;P],OUTM{:}, desc_name, 'Location', 'SouthEast', 'Orientation', 'vertical');
            end

            set(L,'FontSize',12);

        end
    end
    
    if (profileMode == 1)
        filename = sprintf('%s/analysis/desc_profiling/%s-%s-%s.pdf', dataPath, location, modality, 'profile');
    else
        filename = sprintf('%s/analysis/desc_profiling/%s-%s-%s-%s-%s.pdf', dataPath, location, modality, transformation, level, 'profile');
    end
       
    save_to_file_if_possible(H, filename);
        
    % Find summary statistics / characteristic sensitivty/count [actually
    % can do this during the main loop, probably...]

end