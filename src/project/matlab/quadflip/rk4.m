function y = rk4(f, y, dt)
%RK4 Runge-Kutta 4th Order
%   Solves an autonomous (time-invariant) differential equation of the form
%   dy/dt = f(y).

k1 = f(y);
k2 = f(y + dt/2*k1);
k3 = f(y + dt/2*k2);
k4 = f(y + dt  *k3);
y = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

end

