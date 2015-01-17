function [S B] = descriptors_transformation_summary(modality)
    dataPath = get_path_to_data();
    
    desiredCount = 100;
    maxAllowableDiff = 10;
    
    descriptorPrefixes = {'SURF'; 'SIFT'; 'ORB'; 'BRIEF'};
    
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
    
    
    counts = zeros(size(transformations, 1), size(descriptorPrefixes, 1));
    S = zeros(size(counts)); 
    
    for iii = 1:size(transformations, 1)
       for jjj = 1:size(environments{iii}, 1)
           
           subPath = sprintf('%s/%s/%s/%s/%s', dataPath, 'descresults', environments{iii}{jjj}, modality, transformations{iii});
           
           % Count folders...
           folders = get_all_folders(subPath);
           
           
           
           
           for kkk = 1:size(folders, 2)
              
               if (strcmp(folders{kkk}(end-1:end), 'BR') ~= 0)
                   for lll = 1:size(descriptorPrefixes)
                      resultsName = sprintf('%s/%s/%s.csv', subPath, folders{kkk}, descriptorPrefixes{lll});

                      currScore = -1;

                      if (exist(resultsName) ~= 0)

                        M = csvread(resultsName);

                        [X P A E] = pr_score(M);

                        if (E > currScore)
                            currScore = E;

                            S(iii,lll) = S(iii,lll) + currScore;
                            counts(iii,lll) = counts(iii,lll) + 1;
                        end



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
    
    for descriptor = 1:size(descriptorPrefixes, 1)
        
        count = 0;
        
        for ttt = 1:size(transformations, 1)
        
            if (S(ttt, descriptor) > -1)
               count = count + 1;
               B(descriptor) = B(descriptor) + S(ttt, descriptor);
            end
        end
        
        B(descriptor) = B(descriptor) / count;
        
    end
    
end