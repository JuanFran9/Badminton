function [znext] = stepRungeKutta(t,z,dt,theta)
% stepRungeKutta Computes one step using the RungeKutta method
% 
% ZNEXT = stepRungeKutta(T,Z,DT,THETA) computes the state vector ZNEXT at the next
% time step T+DT

% State derivatives from the current state
B = dt * stateDeriv(t, z, theta);
C = dt * stateDeriv(t + dt/2, z + B/2, theta);
D = dt * stateDeriv(t + dt/2, z + C/2, theta);
E = dt * stateDeriv(t + dt, z + D,  theta);
% Next state vector from the previous one using RungeKutta's
% update equation
znext = z + 1/6 * (B + 2*C + 2*D + E); % Output
