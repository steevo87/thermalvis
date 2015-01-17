function Mss = subsample_descriptor_curve(M, interval)

    Mss = zeros(1 + (1.0 / interval), 2);
    
    currIndex = 1;
    currVal = 0.00;
    
    for iii = 1:size(Mss,1)
        
        aimVal = (iii-1)*interval;
        Mss(iii,1) = aimVal;
        
        for jjj = currIndex:size(M, 1)
            
            if (abs(M(jjj, 1)-aimVal) <= abs(currVal-aimVal))
               currVal = M(jjj, 1);
               currIndex = jjj;
               Mss(iii,2) = M(jjj,2);
            end

        end
        
    end

end