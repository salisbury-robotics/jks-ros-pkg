function acc_plot(fname)
a=importdata(fname,'\t');
a(:,1)=a(:,1)-a(1,1);
figure;
plot(a(:,1),a(:,4));
figure;
periodogram(a(:,4),[],[],100);
end