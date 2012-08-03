close all
d=load('rpy2.txt');
%trials = [4,5,6,7;
%          5,4,6,7;
% 5746

trials = perms([4,5,6,7]);
%trials = [4,5,6,7];
%trials = [6,5,7,4]; %5,7,4,6];
for t = 1:size(trials,1)
  i = trials(t,:);
  q0 = d(:,i(1));
  q1 = d(:,i(2));
  q2 = d(:,i(3));
  q3 = d(:,i(4));
  r = atan2(2*(q0.*q1+q2.*q3), 1 - 2*(q1.^2 + q2.^2));
  p = asin(2*(q0.*q2 - q3.*q1));
  y = atan2(2*(q0.*q3+q1.*q2), 1 - 2*(q2.^2 + q3.^2));
  if (r(1) > 1.5)
    r = r - pi;
  end
  figure
  plot([r,p,y]);
  s = sprintf('%d,%d,%d,%d', i);
  title(s);
  legend('roll','pitch','yaw');
end
