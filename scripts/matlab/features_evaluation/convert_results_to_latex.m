function convert_results_to_latex(S, B, filePath, mode)

    % mode 0 : profile detectors
    % mode 1 : profile descriptors
    % mode 2 : transformation detectors
    % mode 3 : transformation descriptors

    detectorPrefixes = {'SURF'; 'STAR'; 'FAST'; 'ORB'; 'MSER'};
    descriptorPrefixes = {'SURF'; 'SIFT'; 'ORB'; 'BRIEF'};
    
    wantedAlgorithms = 0;

    if ((mode == 0) || (mode == 2))
        wantedAlgorithms = detectorPrefixes;
    else
        wantedAlgorithms = descriptorPrefixes;
    end

    % Environments
    
    allEnvironments = {'Aquavist'; 'Balcony'; 'Pipes'; 'Nitrogen'; 'Driveway'; 'Pavement'; 'Oven'; 'Office'; 'Library'; 'Desk'};
    allTransformations = {'Out-of-focus Blur'; 'Time of day'; 'Non-uniformities'; 'Viewpoint'; 'Rotation'; 'Compression'; 'Quantization'; 'Normalization'; 'Gaussian Noise'; 'Salt and Pepper'};
    
    if ((mode == 0) || (mode == 1))
        wantedCategories = allEnvironments;
    else
        wantedCategories = allTransformations;
    end
    

    fid = fopen(filePath, 'w');
    
    % 2.0
    fprintf(fid, '\\begin{tabular}{l');
    
    for iii = 1:size(wantedAlgorithms)
        fprintf(fid, 'r');
    end
    
    fprintf(fid, '}\n');
    
    fprintf(fid, '\\toprule\n');
    
    if (mode < 2)
        fprintf(fid, 'Environment ');
    else
        fprintf(fid, 'Transformation ');
    end
    
    
    
    for iii = 1:size(wantedAlgorithms)
        fprintf(fid, '& ');
        fprintf(fid, wantedAlgorithms{iii});
        fprintf(fid, ' ');
    end
    
    fprintf(fid, '\\\\\n');
    
    fprintf(fid, '\\midrule\n');
    
    for iii = 1:size(S,1)
        
        fprintf(fid, wantedCategories{iii});
        fprintf(fid, ' ');
        
        for jjj = 1:size(wantedAlgorithms)
            fprintf(fid, '& ');
            
            if (S(iii,jjj) == -1)
                fprintf(fid, '-');
            else
                fprintf(fid, sprintf('%0.2f', S(iii,jjj)));
            end
            
            fprintf(fid, ' ');
        end
        
        fprintf(fid, '\\\\\n');
        
    end
    
    fprintf(fid, '\\midrule\n');
    
    fprintf(fid, '\\textbf{Mean} ');
    
    for iii = 1:size(wantedAlgorithms)
        fprintf(fid, '& \\textbf{');
        fprintf(fid, sprintf('%0.2f', B(iii)));
        fprintf(fid, '} ');
    end
    
    fprintf(fid, '\\\\\n');

    fprintf(fid, '\\bottomrule\n');

    fprintf(fid, '\\end{tabular}\n');
end
