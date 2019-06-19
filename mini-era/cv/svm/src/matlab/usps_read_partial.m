function ret=usps_read_partial(datacell_1, datacell_2, idx, opt,dim, Iterations)

X=datacell_1(1:dim,:);
if (opt == 1)
    for i=2:Iterations
        temp = datacell_2(1:dim,:);
        X = [X;temp];
    end
else
     for i=1:Iterations
         if idx==0
             ADD=zeros(dim,1)+i;
         else
             if i~=idx
                 ADD=ones(dim,1)*-1;
             else
                 ADD=ones(dim,1);
             end
         end
         if i==1
             X=ADD;
         else
             X=[X; ADD];
         end
     end
end
ret = X;
 
