% First need to extract data from bagfile:
% rosrun kinfu-listener extractor "/home/steve/radiometry/data/test_2012-11-26-12-08-57.bag" "/home/steve/radiometry/data/test_2012-11-26-12-08-57"

% Address of data extracted from bagfile images in folder
folder_in = '/home/steve/radiometry/data/test_2012-11-26-12-24-39';
thermal_folder = strcat(folder_in, '/thermal');
thermistor_info = strcat(folder_in, '/thermistor.txt');

therm_data = load(thermistor_info);
therm_data(1,1) = therm_data(2,1);
    
% Then go through images and plot the median, against both temperature and
% time (two plots..)

listOfImages = dir(thermal_folder);
imageCount = numel(listOfImages);

currImageCount = 0;

median_vals = [];


for ggg = 1:imageCount
    
    if (mod(ggg,100) == 0)
        disp(sprintf('%d / %d', ggg, imageCount))
    end
    
   if (strcmp(listOfImages(ggg).name, '.') || strcmp(listOfImages(ggg).name, '..'))
        % nothing...
   else
      im_name = strcat(thermal_folder, '/', listOfImages(ggg).name);
      im = imread( im_name );
      
      median_vals = [median_vals; median(im(:))];
      
      %imagesc(im);
      %pause(0.1);
   end
end

figure(1)
plot(median_vals);

figure(2)
plot(therm_data, median_vals);
      