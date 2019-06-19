function [a,b,e,ret]=takeStep(i, j, a1, C, e1, Y, X, eps, b1, N, dim)

a = a1;
e = e1;
b = b1;

ret = 1;
if i == j
    ret =0;
end

if( ret ~= 0)
     %variable initialization%
    a_old = a;

    if a_old(i,1) > 0 && a_old(i,1) < C
        Ei = e(i,1);
    else
        Ei = cal_learned_func(i, a, b, N, Y, X, dim) - Y(i,1);
    end

    if a_old(j,1) > 0 && a_old(j,1) < C
        Ej = e(j,1);
    else
        Ej = cal_learned_func(j, a, b, N, Y, X, dim) - Y(j,1);
    end

    s = Y(i,1) * Y(j,1);

    %Compute L, H%
    if Y(i,1) == Y(j,1)
        gamma = a_old(i,1) + a_old(j,1);
        if gamma > C
            L = gamma-C;
            H = C;
        else
            L = 0;
            H = gamma;
        end
    else
        gamma = a_old(i,1) - a_old(j,1);
        if gamma > 0
            L = 0;
            H = C - gamma;
        else
            L = -gamma;
            H = C;
        end
    end
    

    if L == H
        ret=0;
    end
    
end    

if(ret ~=0)
%     %Compute eta
    k11 = polynomial(3,X(i,:), X(i,:), dim);
    k12 = polynomial(3,X(i,:), X(j,:), dim);
    k22 = polynomial(3,X(j,:), X(j,:), dim);
    eta = 2 * k12 - k11 - k22;
    
    if eta < 0
        a(j,1) = a_old(j,1) + Y(j,1) * (Ej - Ei) / eta;
        if a(j,1) < L
            a(j,1) = L;
        elseif a(j,1) > H
            a(j,1) = H;
        end
    else
        %Compute Lobj, Hobj: objective function at a2=L, a2=H 22di
        c1 = eta/2;
        c2 = Y(j,1) * (Ei-Ej)- eta * a_old(j,1);
        Lobj = c1 * L * L + c2 * L;
        Hobj = c1 * H * H + c2 * H;

        if (Lobj > Hobj+eps)
            a(j,1) = L;
        elseif (Lobj < Hobj-eps)
            a(j,1) = H;
        else
            a(j,1) = a_old(j,1);
        end
    end
    
end


if( ret~= 0)

    if abs(a(j,1)-a_old(j,1)) < eps*(a(j,1)+a_old(j,1)+eps)
        ret=0;
    else
        a(i,1) = a_old(i,1) - s * (a(j,1) - a_old(j,1));
        if a(i,1) < 0
            a(j,1) = a(j,1) + s * a(i,1);
            a(i,1) = 0;
        elseif a(i,1) > C
            t = a(i,1)-C;
            a(j,1) = a(j,1) + s * t;
            a(i,1) = C;
        end

        %Update threshold to reect change in Lagrange multipliers
        if a(i,1) > 0 && a(i,1) < C
            bnew = b + Ei + Y(i,1) * (a(i,1) - a_old(i,1)) * k11 + Y(j,1) * (a(j,1) - a_old(j,1)) * k12;
        else
            if a(j,1) > 0 && a(j,1) < C
                bnew = b + Ej + Y(i,1) * (a(i,1) - a_old(i,1)) * k12 + Y(j,1) * (a(j,1) - a_old(j,1)) * k22;
            else
                b1 = b + Ei + Y(i,1) * (a(i,1) - a_old(i,1)) * k11 + Y(j,1) * (a(j,1) - a_old(j,1)) * k12;
                b2 = b + Ej + Y(i,1) * (a(i,1) - a_old(i,1)) * k12 + Y(j,1) * (a(j,1) - a_old(j,1)) * k22;
                bnew = (b1 + b2) / 2;
            end
        end
        delta_b = bnew - b;
        b = bnew;

        %Update error cache using new Lagrange multipliers 24ai

        t1 = Y(i,1) * (a(i,1)-a_old(i,1));
        t2 = Y(j,1) * (a(j,1)-a_old(j,1));
        for k=1:N
            if 0 < a_old(i,1) && a_old(i,1) < C
                e(k,1) = e(k,1)+t1 * polynomial(3,X(i,:),X(k,:),dim) + t2 * polynomial(3,X(j,:),X(k,:),dim) - delta_b;
                e(i,1) = 0;
                e(j,1) = 0;
            end
        end
        ret = 1;
    end
    
end
    
end
