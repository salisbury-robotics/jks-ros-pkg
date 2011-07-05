function diagnostic_plot(fname)
a=importdata(fname,'\t');
a(:,1)=a(:,1)-a(1,1);
figure;
plot(a(:,1),a(:,17));
end