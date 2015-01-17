function save_to_file_if_possible(H, filename)

if (exist('save2pdf') == 0)
    disp('Cannot save to pdf. Please consider aquiring Gabe Hoffmans save2pdf function from http://www.mathworks.nl/matlabcentral/fileexchange/16179-save2pdf');
else
    disp(sprintf('Saving to (%s)...', filename));
    save2pdf(filename, H, 600);
end
    
end