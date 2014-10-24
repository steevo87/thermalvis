folder_in = '/home/steve/Desktop/nucwarp/A';
folder_out = '/home/steve/Desktop/nucwarp/B';

length = 6;

factor = 25;

for iii = 1:length
    
    im_name = sprintf('%s/frame%04d.jpg', folder_in, iii-1);
    im = imread(im_name);
    
    for jjj = 1:factor
        
        im_name = sprintf('%s/frame%04d.jpg', folder_out, (iii-1)*factor+jjj-1);
        
        imwrite(im, im_name);
        
        
    end
    
    
end
    