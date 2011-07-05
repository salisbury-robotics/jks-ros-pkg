function encoder_filter(fname)
a=importdata(fname,'\t');
a(:,1)=a(:,1)-a(1,1);
figure;plot(a(2:end,5)-a(1:end-1,5));
%fir filter
b2=fir1(8,0.2);
xflt=filtfilt(b2,1,a(:,5));
figure;plot(xflt(2:end)-xflt(1:end-1));
%iir filter
[b3,a3]=butter(2,0.2);
xflt=filtfilt(b3,a3,a(:,5));
figure;plot(xflt(2:end)-xflt(1:end-1));
end