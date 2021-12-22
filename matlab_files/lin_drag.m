function [A,B,C,D] = lin_drag(par,T,aux)
A = -par(1);
B = -aux;
C = 1;
D = 0;
end
