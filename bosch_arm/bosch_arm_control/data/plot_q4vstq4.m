function plot_q4vstq4(fname)
a=importdata(fname,'\t');
a(:,1)=a(:,1)-a(1,1);
[b3,a3]=butter(4,0.05);
%TODO:output filted version of q
xflt=filtfilt(b3,a3,a(:,5));
figure;plot(xflt.*(180/pi),a(:,17));
end