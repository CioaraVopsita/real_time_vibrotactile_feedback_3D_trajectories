initparameters = rand(2,2);
% fun1 = 30*parameters(1,1).^2 + (parameters(1,2)-parameters(1,3)).^2;
% fun2 = 30*parameters(2,1)^2 + (parameters(2,2)-parameters(2,3))^2;
% funt = @(parameters)fun1+fun2;

g = 2;
t = fmincon (@(param) testf(param(1),param(2), g), initparameters, [], []);

%%%no vector notation