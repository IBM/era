function readMatrix(srcImage, outName)

write = fopen([outName '.m'], 'w');

count = fwrite(write, 'function out = ');
count = fwrite(write, outName);
fprintf(write, '\n');
count = fwrite(write, 'out = [...');
fprintf(write, '\n');    

height = size(srcImage,1);
width = size(srcImage,2);

for nI=1:height
    for nJ=1:width
        fprintf(write, '%f ', srcImage(nI,nJ));
    end
    if(nI < height)
        fprintf(write, ';...\n');
    end
end

fprintf(write, '...\n');
count = fwrite(write, '];');

fclose(write);

end
