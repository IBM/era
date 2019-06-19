function out = randWrapper(m,n)

out = zeros(m,n);

seed = 0.9;
for i=1:m
    for j=1:n
        if(i<j)
            out(i,j) = seed * (i/j);
        else
            out(i,j) = seed * (j/i);
        end
    end
end

end

