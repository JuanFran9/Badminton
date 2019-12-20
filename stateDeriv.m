function [dz] = stateDeriv(t,z,theta)
% Calculate the state derivative for a mass-spring-damper system
% 
%     DZ = stateDeriv(T,Z,THETA) computes the derivative DZ = [VX'; AX; VY; AY] of the 
%     state vector Z = [X; VX; Y; VY], where X is the horizontal displacement , Y is the vertical displacement, V is velocity,
%     and A is acceleration. The initial values of Mass, gravity and air
%     density are defined.

M = 0.005; % Mass (kg)
g = 9.81; % Acceleration due to gravity (N/m^2)
rho = 1.225; % Density of air (kg/m^3)

% When the shuttlecock transitions from unstable to stable flight at t = 0.05, the values
% of Cd and A change as follows
 if t < 0.05
        Cd = 0.8;
        A = 0.012;      
 else 
        Cd = 0.6;
        A = 0.009;       
 end
 

v = ((z(2))^2 + (z(4))^2)^0.5; % Calculates resultant velocity (m/s) using Pythagora's theorem

Dragx = (rho * Cd * A * (v^2 * cosd(theta)) / 2 ); % Drag force in the horizontal component
Dragy = (rho * Cd * A * (v^2 * sind(theta)) / 2 ); % Drag force in the vertical component

% State derivatives calculated from free body diagrams
dz1 = z(2);
dz2 = -(Dragx) / M;
dz3 = z(4);
dz4 = -(Dragy + M * g) / M;



dz = [dz1; dz2; dz3; dz4]; % Output