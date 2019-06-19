function [a,b,C,d,dim,e,eps,a_result,b_result,X,tolerance,Y,ret]=getAlphaFromTrainSet(N, d16trn_1, d16trn_2, Iterations)


tolerance=0.001;
C=0.05;
d = -1;
dim=256;
eps=0.001;
a_result=zeros(Iterations,N);
b_result=zeros(Iterations,1);
ret = 0;
Y = 0;

X=usps_read_partial(d16trn_1, d16trn_2,1,1,N/Iterations, Iterations);

for iter=1:Iterations
    Y=usps_read_partial(d16trn_1,d16trn_2,iter,0,N/Iterations, Iterations);
    if iter==1
    fWriteMatrix(Y, './');
    end

    a=zeros(N,1);
    b=0;
    e=zeros(N,1);
    ExamineAll=1;  
    cnt=0;
    NumChanged=0;
    while (NumChanged>0 || ExamineAll == 1)
        cnt=cnt+1;
        NumChanged=0;
        if ExamineAll==1
            for i=1:N
                [a,b,e,ret] = examineExample(i, a, b, C, e, X, Y, tolerance, N, eps, dim);
                NumChanged=NumChanged+ret;
            end
        else
            for i=1:N
                if a(i,1) > 0 && a(i,1) < C
                    [a,b,e,ret] = examineExample(i, a, b, C, e, X, Y, tolerance, N, eps, dim);
                    NumChanged=NumChanged+ret;
                end
            end
        end
        if ExamineAll==1
            ExamineAll=0;
        elseif NumChanged==0 
            ExamineAll=1;
        end
    end
    
    for r = 1:N
        a_result(iter,r) = a(r,1);
    end
    
%    a_result=transpose(a); %KVS: Problem using transpose function here. So wrote the code above
    b_result(iter,1)=b;
end

