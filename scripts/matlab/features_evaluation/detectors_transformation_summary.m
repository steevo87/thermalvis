function [S B] = detectors_transformation_summary(modality, mode)
    dataPath = get_path_to_data();
    
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
    
    desiredCount = 100;
    maxAllowableDiff = 10;
    
    detectorPrefixes = {'SURF'; 'STAR'; 'FAST'; 'ORB'; 'MSER'}; %; 'RAND'};
    
    transformations = {'ofb'; 'tod'; 'nuc'; 'vpt'; 'rot'; 'cmp'; 'qnt'; 'nrm'; 'gau'; 'sap'};
    
    environments{1} = {'aquavist'; 'nitrogen'}; 
    environments{2} = {'balcony'; 'driveway'};
    environments{3} = {'office'; 'pipes'};
    environments{4} = {'oven'; 'pavement'};
    environments{5} = {'desk'; '-'};
    environments{6} = {'office'; 'aquavist'};
    environments{7} = {'library'; 'outside'};
    environments{8} = {'balcony'; 'oven'};
    environments{9} = {'nitrogen'; 'pavement'};
    environments{10} = {'driveway'; 'pipes'};
    
    
    counts = zeros(size(transformations, 1), size(detectorPrefixes, 1));
    S = zeros(size(counts)); 
    
    for iii = 1:size(transformations, 1)
       for jjj = 1:size(environments{iii}, 1)
           
           subPath = sprintf('%s/%s/%s/%s/%s', dataPath, resultsType, environments{iii}{jjj}, modality, transformations{iii});
           
           % Count folders...
           folders = get_all_folders(subPath);
           
           for kkk = 1:size(folders, 2)
               
               if (strcmp(folders{kkk}(end-1:end), 'BR') ~= 0)
                   for lll = 1:size(detectorPrefixes)
                      resultsName = sprintf('%s/%s/%s.csv', subPath, folders{kkk}, detectorPrefixes{lll});

                      if (exist(resultsName) ~= 0)
                          M = csvread(resultsName);
                          S(iii,lll) = S(iii,lll) + M(3);
                          counts(iii,lll) = counts(iii,lll) + 1;
                          %R = input('continue?')
                      end

                   end
               end
               
           end
           
          %environments{iii}{jjj} 
       end
    end
    
    for iii = 1:size(counts,1)
        for jjj = 1:size(counts,2)
            if (counts(iii,jjj) > 0)
                S(iii,jjj) = S(iii,jjj) / counts(iii,jjj); 
            else
                S(iii,jjj) = -1;
            end
        end
    end
    

    B = zeros(1, size(S,2));
    
    for detector = 1:size(detectorPrefixes, 1)
        
        count = 0;
        
        for ttt = 1:size(transformations, 1)
        
            if (S(ttt, detector) > -1)
               count = count + 1;
               B(detector) = B(detector) + S(ttt, detector);
            end
        end
        
        B(detector) = B(detector) / count;
        
    end
    
end