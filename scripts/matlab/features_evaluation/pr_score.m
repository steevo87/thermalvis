function [S P A E] = pr_score(M)
% Calculates the "score" (1 - min distance to optimum performance)
% and the area under the precision-recall curve
% and the maximum product of precision and recall

M = smooth_pr_pts(M);

S = 0.0;
P = 0.0;
A = 0.0;
E = 0.0;

minDiff = 1.00;

for iii = 1:size(M,1)
    
    if (abs((1-M(iii,1)) - M(iii,2)) < minDiff)
        E = mean([(1-M(iii,1)) M(iii,2)]);
        minDiff = abs((1-M(iii,1)) - M(iii,2));
    end
    
    
    currScore = sqrt((1-M(iii,1))^2 + (M(iii,2))^2)/sqrt(2);
    
    if (currScore > S)
        S = currScore;
    end
    
    currProd = (1-M(iii,1))*(M(iii,2));
    
    if (currProd > P)
        P = currProd;
    end
    
    if (iii < size(M,1))
        
        A = A + ((M(iii+1,1)-M(iii,1)) * (M(iii+1,2)+M(iii,2))/2);
        
    end

end