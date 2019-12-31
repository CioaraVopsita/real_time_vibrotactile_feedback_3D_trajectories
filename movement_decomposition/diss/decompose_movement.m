% %Declare variables - ROW VECTORS!!!!! 1XN
% time = data(:,1)';
% no_submovements = 3;
% Gx = data(:,2)';
% Gy = data(:,3)';
% Gz = data(:,4)';

%bounds for amplitude: if amplitude = displacement; max displacement in
%0.0042s = 7cm => 1680 cm in 1s => ~ 17m in a sec
lb = [0.167         time(1)       -10 -10 -10];
ub = [ 1    time(end)-0.167      10 10 10];
lb_total = repmat(lb',[1,no_submovements]);
ub_total = repmat(ub',[1,no_submovements]);

%Random parameters
Dxr = -10+20*rand(1,no_submovements,10);
Dyr = -10+20*rand(1,no_submovements,10);Dzr = -10+20*rand(1,no_submovements,10);
Dr=0.167+0.833*rand(1,no_submovements,10); t0r = (repmat(time(length(time)),[1,no_submovements,10])-Dr).*rand(1,no_submovements,10);
initparam = [Dr; t0r; Dxr; Dyr; Dzr];

epsilon = []; %global search; more tan 100 initial points; start won't be 0; plot error to see where it goes wrong

%options = optimset('MaxFunEvals',10^13,'MaxIter',5000);

options = optimset('GradObj','on','Hessian', 'on', 'LargeScale','on',...
         'MaxFunEvals',10^13,'MaxIter',5000,...
         'FunValCheck','on','DerivativeCheck','off');
f = @(parameters)submovement(parameters, time, no_submovements, Gx, Gy, Gz); 
for random_start = 1:10
    optimization(:,:,random_start) = fmincon(f, initparam(:,:,random_start), [], [], [], [], lb_total, ub_total, [], options); 
    cost(random_start) = submovement(optimization(:,:,random_start), time, no_submovements, Gx, Gy, Gz);
end