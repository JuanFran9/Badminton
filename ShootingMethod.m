function [CorrectTheta1]   =  ShootingMethod(t0,x0,y0,dt,v0,xend)

% [CORRECTTHETA] = ShootingMethod(T0,X0,Y0,DT,VO,XEND) Where T0 is the initial time, 
% X0 the initial horizontal point, Y0 the initial vertical point, DT the step size, V0 
% the initial velocity and XEND the horizontal point at which the shuttlecock must land. 
% This function finds the best lauch angle for the specified input
% variables and displays it on the graph.


theta(1) = randi([10,40]); % Selects a random angle from 10 to 40 degrees for the first guess
theta(2) = randi([40,90]); % Selects a random angle from 40 to 90 degrees for the second guess

x(1) = ivpSolverRungeKutta(t0,x0,y0,theta(1),dt,v0); % Calculates and plots the trajectory of the first guess

x(2) = ivpSolverRungeKutta(t0,x0,y0,theta(2),dt,v0); % Calculates and plots the trajectory of the first guess


error(1) = x(1) - xend; % Error of first guess
error(2) = x(2) - xend; % Error of second guess

errormax = 0.0001; % Maximum error desired

n = 2;
% While loop to reduce the horizontal distance error until it is 
%smaller than the the errormax
while abs(error(n)) >= errormax
    theta(n+1) = theta(n) - error(n) * ((theta(n) - theta(n-1)) / (error(n) - error(n-1)));
    x(n+1) = ivpSolverRungeKutta(t0,x0,y0,theta(n+1),dt,v0);
    hold on
    error(n+1) = x(n+1)- xend;
    
    n = n + 1;
end

% When the error is less than errormax, the value of theta becomes the
% correct angle for hitting the target to the desired point
CorrectTheta = theta(n); % Output
% The ivpsolverRungeKutta function is executed with the correct angle
ivpSolverRungeKutta(t0,x0,y0,CorrectTheta,dt,v0);


CorrectTheta1 = num2str(theta(n));
% Display textbox with the shot angle
CorrectThetaplot = ['Shot Angle = ', CorrectTheta1, '°'];
CorrectThetaplotdim = [0.16 0.75 0.1 0.1];
annotation('textbox',CorrectThetaplotdim, 'String', CorrectThetaplot, 'Fitboxtotext', 'on', 'fontsize', 9)
end














