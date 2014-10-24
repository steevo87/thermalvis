function overlay_colorscale_on_video(folder_in, folder_out, overlay_csiro)
%% folder_in = '/home/steve/Desktop/test-x'
%% folder_out = '/home/steve/Desktop/test-x-2'

    csiro_padding = 3;

    if (nargin < 3)
        overlay_csiro = 0;
    end
    
    csiro_logo = [];
    
    if (overlay_csiro)
       csiro_logo = imread('../../../media/csiro.jpg');
       csiro_logo = imresize(csiro_logo, 0.07, 'bicubic');
    end

    listOfImages = dir(folder_in);
    imageCount = numel(listOfImages);

    currImageCount = 0;
    
    scale = imread('../output/scale.png');

    for ggg = 1:imageCount
       if (strcmp(listOfImages(ggg).name, '.') || strcmp(listOfImages(ggg).name, '..'))
            % nothing...
       else
          im_name = strcat(folder_in, '/', listOfImages(ggg).name);
          im = imread( im_name );
          
          im(3:size(scale,1)+2, 3:size(scale,2)+2,:) = scale(:,:,:);
          
          if (overlay_csiro)
              
              for iii = 1:size(csiro_logo,1)
                  %size(im,1)-size(csiro_logo,1)+1-csiro_padding:size(im,1)
                  for jjj = 1:size(csiro_logo,2)
                      %size(im,2)-size(csiro_logo,2)+1-csiro_padding:size(im,2)-csiro_padding
                      if ((csiro_logo(iii,jjj,1) ~= 255) || (csiro_logo(iii,jjj,2) ~= 255) || (csiro_logo(iii,jjj,3) ~= 255))
                          a = size(im,1)-size(csiro_logo,1)+1-csiro_padding+iii;
                          b = size(im,2)-size(csiro_logo,2)+1-csiro_padding+jjj;
                          
                          %[iii jjj a b]
                          im(a, b, :) = csiro_logo(iii,jjj,:);
                      end
                  end
              end
              
              %im(size(im,1)-size(csiro_logo,1)+1-csiro_padding:size(im,1)-csiro_padding, size(im,2)-size(csiro_logo,2)+1-csiro_padding:size(im,2)-csiro_padding,:) = csiro_logo(:,:,:);
          end
          
          imshow(im);
          pause(0.01);
          
          out_name = strcat(folder_out, '/', listOfImages(ggg).name);
          imwrite(im, out_name);
          
       end
    end

end