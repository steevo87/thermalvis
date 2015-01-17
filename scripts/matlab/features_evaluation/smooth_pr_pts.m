function newM = smooth_pr_pts(M)

    newM = [];

    % First get rid of rows with duplicate X values (preserve best)
    iii = 1;
    while (iii < size(M,1))
        if (M(iii,1) == M(iii+1,1))
            
            M(iii,2) = max([M(iii,2) M(iii+1,2)]);
            
            if (iii < size(M,1)-1)               
                M = [M(1:iii,:); M(iii+2:end,:)];
            else
                M = M(1:iii,:);
            end
            
        else
            iii = iii + 1;
        end 
    end
    
    % Then lift rows levels
    for iii = 1:size(M,1)-1
        if (M(iii+1,2) < M(iii,2))
           M(iii+1,2) = M(iii,2); 
        end
    end
    
    % Then zero pad at the start
    if ((M(1,1) == 0.00) && (M(1,2) == 0.00))
        M(1,2) = M(2,2);
    end
    
    % Then go through and "flatten" sections with large gaps
    
    iii = 1;
    
    while (iii < (size(M, 1)-1))
       
        if (abs(M(iii+1,1) - M(iii,1)) > 0.01)
            M = [M(1:iii,:); [0.5*(M(iii,1)+M(iii+1,1)) M(iii,2)]; M(iii+1:end,:)];
        else
            iii = iii + 1;
        end
    end
    
    newM = M;

end