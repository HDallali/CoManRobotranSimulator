function writemat(filename,Matrix)   
   
if nargin < 2, error('Requires at least 2 input arguments.'); end   
   
% open the file   
fid = fopen(filename ,'wt');   
if fid == (-1), error(['Could not open file ' filename]); end   
   
% dimensions size of matrix   
[row,col] = size(Matrix);   
   
%Dumping Matrix   
for i = 1:row   
    for j = 1:col   
       str=sprintf('%12.8f ',Matrix(i,j));   
       if(Matrix(i,j)==0)   
           str=sprintf('%12.0f ',Matrix(i,j));   
       end                    
       fwrite(fid, str, 'uchar');   
    end   
    if i<row   
       str=sprintf('\n');   
       fwrite(fid,str);   
    end   
end   
   
fclose(fid)   
end   
