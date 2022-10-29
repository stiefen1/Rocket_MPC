function [x_next] = RK4(X,U,h,f)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
% write your function here

k1 = f(X, U);
k2 = f(X+k1*h/2, U);
k3 = f(X+k2*h/2, U);
k4 = f(X+k3*h, U);

x_next = X + h*(k1+2*k2+2*k3+k4)/6;
