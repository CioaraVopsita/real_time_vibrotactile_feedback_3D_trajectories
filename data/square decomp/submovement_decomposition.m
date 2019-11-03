% %Declare variables
% time = data(:,1)';
% no_submovements = 3;
% Gx = data(:,2)';
% Gy = data(:,3)';
% Gz = data(:,4)';

%bounds for amplitude: if amplitude = displacement; max displacement in
%0.0042s = 7cm => 1680 cm in 1s => ~ 17m in a sec
lb = [time(1) 0.167 -30 -30 -30];
ub = [time(end)-0.167 1 30 30 30];
lb_total = repmat(lb',[1,no_submovements]);
ub_total = repmat(ub',[1,no_submovements]);

%Random parameters
Dx = -30+60*rand(1,no_submovements,10);
Dy = -30+60*rand(1,no_submovements,10);Dz = -15+30*rand(1,no_submovements,10);
D=0.167+0.833*rand(1,no_submovements,10); t0 = (repmat(time(length(time)),[1,no_submovements,10])-D).*rand(1,no_submovements,10);
initparam = [t0; D; Dx; Dy; Dz];

epsilon = [];

options = optimset('MaxFunEvals',10^13,'MaxIter',5000);
%options = optimset('MaxFunEvals',10^13,'MaxIter',5000, 'LargeScale','off');

%fun = @(parameters) 
        
for random_start = 1:10
   optimization(:,:,random_start) = fmincon(@(parameters) submovement(parameters, time, no_submovements, Gx, Gy, Gz), initparam(:,:,random_start), [], [], [], [], lb_total, ub_total, [], options); 
     %optimization(:,:,random_start) = fminsearch(@(parameters) submovement(parameters, time, no_submovements, Gx, Gy, Gz), initparam(:,:,random_start));
end