%% 
fun = @(parameters)norm(parameters);
x0 = [1,2,3;4,5,7];
options = optimset("MaxFunEvals",5000, "MaxIter",10^15);
X = fminsearch(fun, x0, options);
%%  
