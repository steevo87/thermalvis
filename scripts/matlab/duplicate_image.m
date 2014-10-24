function duplicate_image(input_address, output_address, duplications_vector, start)
%% input_address = '/home/steve/Desktop/test-x-2/%04d.png';
%% output_address = '/home/steve/Desktop/test-x-3/%04d.png';
%% start = 0;
%% duplications_vector = [100 100 100 ones(1,247) 100 ones(1,250) 100 100 100];

idx = 0;

for iii = 1:size(duplications_vector,2)
    im = imread(sprintf(input_address, start+iii-1));
    for jjj = 1:duplications_vector(1,iii)
        out_add = sprintf(output_address, idx);
        idx = idx + 1;
        imwrite(im, out_add);
    end
   
   
end

end