% Simulation of two-arm manipulation.
% The cdts (Cooperative Dual Task-Space) class is used to simulate two kuka
% LWR manipulating a broom


%% Clear the system for performing a fresh start
clear all;
clear classes;
close all;
clc;

%% Basic definitions for the two-arm system
kuka1 = DQ_KUKA;
kuka2 = DQ_KUKA;
kuka1.base = 1+DQ.E*0.5*DQ([0,-0.4,0,0]);
kuka2.base =  1+DQ.E*0.5*DQ([0, 0.4,0,0]);
two_arms = DQ_cdts(kuka1, kuka2);


%% Initial conditions
initial_theta1 = [-pi/2    pi/1.5   pi/4   pi/4   0  0  0]';
initial_theta2 = [-pi/2    pi/1.5  -pi/4   pi/4   0  0  0]';
theta=[ initial_theta1; initial_theta2];

%% Task definitions for grabbing the broom.

% The relative configuration between the hands must remain constant in order to minimize internal forces
dqrd = two_arms.xr(theta);

%%Translate the bucket in all directions, but maintain the orientation
dqad_ant =  two_arms.xa(theta);
dqad = (1+DQ.E*0.5*(-0.1*DQ.i-0.1*DQ.j-0.1*DQ.k)) * dqad_ant; 
taskd=[dqad.q;dqrd.q];



%Drawing the arms
opt={'noname','nojaxes'};
plot(kuka1,initial_theta1',opt{:});
hold on;
plot(kuka2,initial_theta2',opt{:});
plot(dqad,'scale',0.5);

grid off;
axis equal;
%axis off;
view (-163,28);
xlabel('x')
ylabel('y')




drawnow;


%% Two-arm control

epsilon = 0.01; %for the stop condition
nerror_ant = epsilon+1;
error = 0;
i=0;
iter=1;
j=1;

%The sweep motion (back and forth) will be performed twice
while norm(nerror_ant - error) > epsilon
    
    %standard control law
    nerror_ant = error;
    jacob = [two_arms.Ja(theta);two_arms.Jr(theta)];
    taskm=  [vec8(two_arms.xa(theta)); vec8(two_arms.xr(theta))];
    
    error = taskd-taskm;
    theta = theta+pinv(jacob)*0.5*error;
    
    % Visualisation
    
    % Plot the arms
    plot(kuka1,theta(1:7)');
    plot(kuka2,theta(8:14)');
    %plot small coordinate systems such that one does not mistake with the desired absolute pose,
    %which is the big frame
    plot(two_arms.xa(theta),'scale',0.1); 
end



