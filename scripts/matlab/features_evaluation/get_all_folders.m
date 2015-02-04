function actualFolders = get_all_folders(path)
    
d = dir(path);
    folderCount = size(d, 1);
    %path
    
    realFolderCount = 0;
    
    if (exist(path) ~= 0)
        for level = 1:folderCount
           if ((strcmp(d(level).name, '.') == 0) && (strcmp(d(level).name, '..')  == 0))
               realFolderCount = realFolderCount + 1;
               actualFolders{realFolderCount} = d(level).name;
           end
        end
    else
       actualFolders = {}; 
    end
    
    
    
end