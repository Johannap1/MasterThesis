function [dx, y] = nl_drag(t,x,u,par,varargin)
dx = -par*x + 9.81*-tan(u);
y = x;
end