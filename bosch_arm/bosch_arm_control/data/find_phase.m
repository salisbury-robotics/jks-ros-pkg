t=0:0.01:0.01*(length(gt)-1);
mi=0;
mc=0;
for i=0:0.01:2*pi
    tmp=sin(t+i);
    c=corr(gt',tmp');
    if mc<c
       mc=c;
       mi=i;
    end
end
mc
mi