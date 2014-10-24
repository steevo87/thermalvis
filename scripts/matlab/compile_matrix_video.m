function compile_matrix_video(folder_addresses, output_address)

    % folder_addresses = '/home/steve/sfm/data/thermal/quad-vid/video_addresses.txt';
    % output_address = '/home/steve/sfm/data/thermal/quad-vid/output';

    padding = 4;
    
    fid = fopen(folder_addresses);
    
    idx = 1;
    
    while (1)
        
        L{idx} = fgets(fid);
        
        if (~ischar(L{idx}))
            break;
        end
        
        size(L{idx})
        L{idx} = L{idx}(1:size(L{idx},2)-1);
        
        if (L{idx}(1) ~= '%')
            idx = idx + 1;
        end
        
    end
    
    L = L(1,1:idx-1);

    fclose(fid);

    rows = ceil(sqrt(size(L,2)));
    cols = ceil(sqrt(size(L,2)));
    

    for iii = 1:size(L,2)
       
       size(L{iii})
       
       %strcmp(L{iii}, '/home/steve/sfm/data/thermal/quad-vid/5')
       
       %pause(1.0)
       %L{iii} = '/home/steve/sfm/data/thermal/quad-vid/5';
       %size(L{iii})
       %pause(1.0)
       
       listOfImages = dir(L{iii})
       imageCount = numel(listOfImages)
       
       J{iii} = listOfImages;
       
    end
    
    % find shortest sequence
    
    shortestSequence = 9e99;
   
    for iii = size(J,2)
       if (size(J{iii},1) < shortestSequence)
           shortestSequence = size(J{iii},1);
       end
    end
    
    shortestSequence = shortestSequence - 2; % to remove non-image files
    
    % Determine image sizes (assume all frames in all videos are the same)
    im_sample = imread(J{1}(3).name);
    %imagesc(im_sample);
    
    im_height = size(im_sample, 1);
    im_width = size(im_sample, 2);
    
    
    
    totalRows = padding;
    totalCols = padding;
    
    for iii = 1:rows
       totalRows = totalRows + padding + im_height; 
    end
    
    for iii = 1:cols
       totalCols = totalCols + padding + im_width; 
    end
    
    [totalRows totalCols]
    
    blankImage = 255*ones(totalRows, totalCols, 3, 'uint8');
    
    for iii = 1:shortestSequence
        for jjj = 1:rows
            for kkk = 1:cols
                
                seq = kkk + cols*(jjj-1);
            
                %x = 1+padding+(jjj-1)*im_height
                %y = padding+jjj*im_height
                
                %x = 1+padding+(kkk-1)*im_width
                %y = padding+kkk*im_width
                
                im_name = strcat(L{seq}, '/', J{seq}(iii+2).name);
                
                im = imread(im_name);
                
                blankImage(1+padding*jjj+(jjj-1)*im_height:padding*jjj+jjj*im_height, 1+padding*kkk+(kkk-1)*im_width:padding*kkk+kkk*im_width,:) = im;

                write_name = strcat(output_address, '/', sprintf('frame%06d.jpg', iii));
                
                imwrite(blankImage, write_name);
                
            end
        end
        
        
    end
    
end