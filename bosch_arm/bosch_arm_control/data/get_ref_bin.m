data=load(fname,'\t');
q4=data(:,5);
tq4=data(:,17);
a=round(q4(1)/0.01)*0.01;
b=round(q4(end)/0.01)*0.01;
res=0.01;
if a<b
    step=res;
else
    step=-res;
end
x=a:step:b;
y=zeros(size(x));
cnt=1;
for i=x
    bin=tq4(q4>=i-res/2&q4<i+res/2);
    y(cnt)=mean(bin);
    cnt=cnt+1;
end
fid=fopen(fout,'w');
fprintf(fid,'%f,%f,%f\n',[a,step,b]);
fprintf(fid,'%f\n',y');
fclose(fid);