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
initial_theta1 = [-1.6965    2.2620    1.5708    1.4451   -0.4398    0.0628         0]';
initial_theta2 = [1.6336    -0.8168    1.5708    1.5080   -0.2513         0         0]';
theta=[ initial_theta1; initial_theta2];

%% Task definitions for grabbing the broom.

% The relative configuration between the hands must remain constant in order to minimize internal forces
dqrd = two_arms.xr(theta);

% The sweep motion consists of turning the broom around the torso's y axis.
% So let's use the decompositional multiplication! (see )
dqad_ant =  two_arms.xa(theta);
dqad = DQ([cos(pi/16);0;sin(pi/16);0]) .* dqad_ant;

taskd=[dqad.q;dqrd.q];

%% Definition for DRAWING the broom (with respect to the right hand)
d_broom = 0.9;

aux = translation(two_arms.x1(theta))-translation(two_arms.x2(theta));
t_broom = aux*(1/norm(aux.q))*d_broom;

broom_base=translation(two_arms.x2(theta));
broom_tip = broom_base+t_broom;

%Broom's color
my_green = [0,0.7,0];

%% Ploting the system in the initial configuration
%Drawing the arms
%opt={'noname','nojaxes'};
%plot(kuka1,initial_theta1',opt{:});
%plot(kuka2,initial_theta2',opt{:});

figure;
hold on;
grid off;
axis equal;
axis ([-0.6,0.6,-0.2,0.8,-0.1,0.6])
view(-153,24);

% Drawing the arms
opt={'noname','nojaxes','noshadow','nobase'};
plot(kuka1,initial_theta1',opt{:});
plot(kuka2,initial_theta2',opt{:});
% Drawing the broom;
line_handle1=line([broom_base.q(2),broom_tip.q(2)],[broom_base.q(3),broom_tip.q(3)],[broom_base.q(4),broom_tip.q(4)],'color',my_green,'linewidth',3);

drawnow;

pause(1);

%% Two-arm control
epsilon = 0.01; %for the stop condition
error = epsilon+1;
i=0;
iter=1;
j=1;

%The sweep motion (back and forth) will be performed twice
while j <= 4    
    %standard control law
    nerror_ant = error;
    jacob = [two_arms.Ja(theta);two_arms.Jr(theta)];
    taskm =  [vec8(two_arms.xa(theta)); vec8(two_arms.xr(theta))];
    
    error = taskd-taskm;
    theta = theta+pinv(jacob)*0.5*error;
    
    % Visualisation
    
    % Plot the arms
    plot(kuka1,theta(1:7)');
    plot(kuka2,theta(8:14)');
    
    % Plot the broom
    aux = translation(two_arms.x1(theta))-translation(two_arms.x2(theta));
    t_broom = aux*(1/norm(aux.q))*d_broom;

    broom_base=translation(two_arms.x2(theta));
    broom_tip = broom_base+t_broom;

    set(line_handle1, 'Xdata', [broom_base.q(2), broom_tip.q(2)], 'Ydata', [broom_base.q(3), broom_tip.q(3)], 'Zdata', [broom_base.q(4),broom_tip.q(4)]);
    
    % Verify if the sweep direction can be changed
    if(norm(nerror_ant - error) < epsilon)
        %Change the task
        temp = dqad;
        dqad = dqad_ant;
        dqad_ant = temp;
        
        taskd(1:8,1)=dqad.q;
        j = j+1;
    end
    drawnow;
end



