function [cost] = test2fc (rewind, time)
    t0i = [1 2 3 4 5];
    Axi = [2 2 2 2 2];
    t = time+1;
    predictedv = [];
    optionalv = [];
    for n = 1:rewind
        [predicted, optional] = testfunction (t0i(2), Axi(2), time);
        predictedv = [predictedv; predicted];
        optionalv = [optionalv; optional];
    end
    cost = [predictedv; optionalv];
end