function [funt] = testf (d1, d2, d3)
    fun1 = 30*d1.^2 + (d2-d3).^2;
    fun2 = 30*d1^2 + (d2-d3)^2;
    funt = fun1 + fun2;
end
