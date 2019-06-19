function ret=cal_learned_func(k, a, b, N, Y, X,dim)

s=0;
for i=1:N
    if a(i,1) > 0
        s = s +a(i,1)*Y(i,1)*polynomial(3,X(i,:),X(k,:),dim);        
    end
end
s = s - b;
ret=s;

