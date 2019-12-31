%Function that computes measurement noise for each timestamp 
function [v,R] = measurement_noise

%First R
% r1 = [0.02 0.01 0.007 0 0 0 0 0 0];
% r2 = [0 0 0 0.025 0.012 0.009 0 0 0];
% r3 = [0 0 0 0 0 0 0.01 0.005 0.003];
% R = sqrt(r1)'*sqrt(r1) + sqrt(r2)'*sqrt(r2) + sqrt(r3)'*sqrt(r3); 
% sigma = sqrt(r1+r2+r3);

% Second R
T = 30*0.0042;
r1 = [T^3/6 T^2/2 T 0 0 0 0 0 0]; 
r2 = [0 0 0 T^3/6 T^2/2 T 0 0 0];
r3 = [0 0 0 0 0 0 T^3/6 T^2/2 T];
R = (r1'*r1+r2'*r2+r3'*r3);
sigma = r1+r2+r3;


v = sigma'.*randn(9,1);

end