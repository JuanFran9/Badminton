function [ztarget] = ivpSolverRungeKutta(t0,x0,y0,theta,dt,v0)
% ivpSolver    Solve an initial value problem (IVP) and plot the result
% 
%     [ZTARGET] = ivpSolver(T0,X0,Y0,THETA,DT,V0) computes the IVP solution using a step 
%     size DT, beginning at time T0 and initial state Z0 and ending at time 
%     TEND. The solution is output as a matrix of state vectors Z. The
%     velocity at impact vend is calculated by using pythagoras with z(2)
%     and z(4). The trajecory of the shuttlecock and the 3D court are plotted
 
close all

% Set initial conditions
t(1) = t0;
z(:,1) = [x0,v0 * cosd(theta), y0, v0 * sind(theta)];
theta(1) = theta;
% Continue stepping until the end time is exceeded
n=1;
while z(3,:) >= 0
    % Increment the time vector by one time step
    t(n+1) = t(n) + dt;
    theta(n) = atand(z(4,n)/z(2,n));
    
    % Apply RungeKutta's method for one time step
    z(:,n+1) = stepRungeKutta(t(n), z(:,n), dt, theta(n));
    
    n = n+1;
end
 
ztarget = z(1,end); %Output

% Compute acceleration from velocity (not needed for solving the
% problem, but nice to have for completeness)
ddzx = diff(z(2,:)) / dt;
ddzy = diff(z(4,:)) / dt;
 
% Final velocity calculated
vend = num2str(( z(2,end)^2 + z(4,end)^2 )^0.5);

 
% Plot the trajectory of the correct flight
figure(2)
plot3(z(1,1:0.05/dt),z(1,1:0.05/dt)*0,z(3,1:0.05/dt),'b','linewidth',1.5) %Plots a red line for the unstable flight
hold on
plot3(z(1,0.05/dt:end),z(1,0.05/dt:end)*0,z(3,0.05/dt:end),'r','linewidth',1.5) %Plots a blue line for the stable flight
 
% Display textbox with final velocity
vendplot = ['Impact velocity = ', vend, 'm/s'];
vendplotdim = [0.16 0.7 0.1 0.1];
annotation('textbox',vendplotdim, 'String', vendplot, 'Fitboxtotext', 'on', 'fontsize', 9)



% Plot a green surface on the as the floor of the court
patch([-3,-3,3,3],[3,-3,-3,3],[0,0,0,0],[0.4660, 0.6740, 0.1880])
 
 
% Plot the white grid lines on the court
plot3([-2.5,-2.5],[-3,3],[0,0],'w','linewidth',1.5)
 
plot3([2.5,2.5],[-3,3],[0,0],'w','linewidth',1.5)
 
plot3([-3,3],[-2.59,-2.59],[0,0],'w','linewidth',1.5)
 
plot3([-3,3],[2.59,2.59],[0,0],'w','linewidth',1.5)
 
plot3([0,0],[-3,3],[0,0],'w','linewidth',2)
 
plot3([0.9,0.9],[-3,3],[0,0],'w','linewidth',2)
 
plot3([-0.9,-0.9],[-3,3],[0,0],'w','linewidth',2)
 
plot3([0.9,3],[0,0],[0,0],'w','linewidth',2)
 
plot3([-0.9,-3],[0,0],[0,0],'w','linewidth',2)
 
% Plot the net of height 1.524m
plot3([0,0],[3,3],[1.55,0],'b','linewidth',2)
 
plot3([0,0],[-3,-3],[1.55,0],'b','linewidth',2)
 
patch([0,0,0,0],[3,3,-3,-3],[0.76,1.524,1.524,0.76],[0.5 0.5 0.5])
alpha(0.5) 
hold off
 
xlabel('Court width (m)')
ylabel('Court Lenght (m)')
zlabel('Height (m)')
legend('Unstable flight','Stable Flight')
end
