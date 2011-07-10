function [xo,yo]=consolidate(x,y)
i=1;
k=1;
while i<length(x)
    xo(k)=x(i);
    yo(k)=y(i);
    cnt=1;
    while i<length(x) && x(i+1)<=x(i) 
        yo(k)=yo(k)+y(i+1);
        xo(k)=xo(k)+x(i+1);
        cnt=cnt+1;
        i=i+1;
    end
    yo(k)=yo(k)/cnt;
    xo(k)=xo(k)/cnt;
    i=i+1;
    k=k+1;
end
if i==length(x)
    xo(k)=x(i);
    yo(k)=y(i);
end
end