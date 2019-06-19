function script_run_profile(dataDir, resultDir, type, commonDir,toolDir)

path(path, commonDir);
Iterations = 2;

file = [dataDir, '/d16trn_1.txt'];
trn_1 = readFile(file);
file = [dataDir, '/d16trn_2.txt'];
trn_2 = readFile(file);
file = [dataDir, '/d16tst_1.txt'];
tst_1 = readFile(file);
file = [dataDir, '/d16tst_2.txt'];
tst_2 = readFile(file);

trn_1 = d16trn_1;
trn_2 = d16trn_2;
tst_1 = d16tst_1;
tst_2 = d16tst_2;

    N = 100;
    Ntst = 100;
    Iterations = 10;

    if(strcmp(type, 'test'))
    N = 4;
    Ntst = 4;
    Iterations = 2;
    elseif(strcmp(type, 'sim_fast'))
    N = 20;
    Ntst = 20;
    Iterations = 2;
    elseif(strcmp(type, 'sim'))
    N = 16;
    Ntst = 16;
    Iterations = 8;
    elseif(strcmp(type, 'sqcif'))
    N = 60;
    Ntst = 60;
    Iterations = 6;
    elseif(strcmp(type, 'qcif'))
    N = 72;
    Ntst = 72;
    Iterations = 8;
    elseif(strcmp(type, 'vga'))
    N = 450;
    Ntst = 450;
    Iterations = 15;
    elseif(strcmp(type, 'wuxga'))
    N = 1000;
    Ntst = 1000;
    Iterations = 20;
    end
    
fprintf(1,'Input size\t\t- (%dx%dx%d)\n', N, Ntst,Iterations);

elapsed = zeros(1,2);
%% Timing
start = photonStartTiming;

[a,b,C,d,dim,e,eps,a_result,b_result,X,tolerance,Y,ret] = getAlphaFromTrainSet(N, trn_1, trn_2, Iterations);

Yoffset=zeros(Iterations,N);
 
Xtst=usps_read_partial(tst_1, tst_2 ,0,1,Ntst/Iterations, Iterations);
Ytst=usps_read_partial(tst_1, tst_2, 0,0,Ntst/Iterations, Iterations);
 
 for i=1:Iterations
     temp=usps_read_partial(trn_1,trn_2, i,0,N/Iterations, Iterations);
     Yoffset(i,:)=transpose(temp);
 end
 
 
 %error=0;
 result=zeros(Ntst,1);
 for n=1:Ntst
     maxs = 0;
     s=zeros(Iterations,1);
     for i=1:Iterations
         for j=1:N
             if a_result(i,j) > 0
                 s(i,1) = s(i,1) +a_result(i,j)*Yoffset(i,j)*polynomial(3,X(j,:),Xtst(n,:), dim);        
             end
         end
         s(i,1) = s(i,1) - b_result(i,1);
         
         if( s(i,1) > maxs)
             maxs = s(i,1);
         end
     end
     result(n,1) = maxs;
 end
 
 %% Timing
stop = photonEndTiming;

temp = photonReportTiming(start, stop);
elapsed(1) = elapsed(1) + temp(1);
elapsed(2) = elapsed(2) + temp(2);
 
 %% Self check
 fWriteMatrix(result, dataDir);
 tol = 0.001;
 ret = fSelfCheck(result,dataDir, tol);
 if(ret == 0)
     disp('Error in SVM');
 end

%% Timing
photonPrintTiming(elapsed);




