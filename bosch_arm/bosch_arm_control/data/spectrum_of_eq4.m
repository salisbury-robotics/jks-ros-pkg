%spectrum analysis
figure;
data=hkp2stillc300b(:,21);
d=1/size(data,1);
plot(d:d:1,abs(fft(data)));