function ret = selfCheck(in1, in2, tol)

r1 = size(in1, 1);
c1 = size(in1, 2);

r2 = size(in2, 1);
c2 = size(in2, 2);

ret = 1;

if r1~=r2 
    disp(1298);
    ret = 0;
end

if c1 ~= c2
	disp(1297);
	ret = 0;
end

for i=1:r1
    if(ret == 0)
        break;
    end
    for j=1:c1
        if( abs(in1(i,j)-in2(i,j)) > tol)
            ret = 0;
            break;
        end
    end
end


