function [a,b,e,ret]=examineExample(i, a, b, C, e, X, Y, tolerance, N, eps, dim)

ret = 0;
if (a(i,1) > 0) && (a(i,1) < C)
    E = e(i,1);
else
    E = cal_learned_func(i, a, b, N, Y, X, dim) - Y(i,1);
end

 r1 = Y(i,1) * E;

 if (r1 < -1*tolerance && a(i,1) < C) || (r1 > tolerance && a(i,1) > 0)
    %case1 argmax E-E2%
    maxEDiff=0;
    j=i;
    for k=1:N
        if a(k,1) > 0 && a(k,1) < C
            temp= abs(E - e(k,1));
            if temp > maxEDiff
                j=k;
            end
        end
    end
    
    if i~=j
        [a,b,e,ret]=takeStep(i, j, a, C, e, Y, X, eps, b, N, dim);
    end

    if(ret ~=1)
        %iterate through the non-bound example%
%        for k=round(randWrapper(1,1)*(N-2))+1:N
        randVal = 1.0;
        for k=(randVal*(N-1)):N
            if (a(k,1) > 0 && a(k,1) < C && ret == 0)
%			    if (i == k)
%				    ret = 0;
%			    else
            	    [a,b,e,ret]=takeStep(i, k, a, C, e, Y, X, eps, b, N, dim);
%			    end
            end
        end
    end

    if(ret ~= 1)
        %iter for entire training set%
        for k=1:N
            if(ret == 0)
%			    if (i == k)
%				    ret = 0;
%			    else
            	    [a,b,e,ret]=takeStep(i, k, a, C, e, Y, X, eps, b, N, dim);
%			    end
            end
        end
    end

    else

end 

