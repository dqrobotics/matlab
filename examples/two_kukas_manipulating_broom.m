%% Clear the system for performing a fresh start
clear all;
clear classes;
close all;
clc;

%% Definitions for the robotics toolbox

% L = LINK([alpha A   theta       D    sigma  offset])

l1_a=link([pi/2       0       0      0.310    ], 'standard'); %This first parameter was modified to take into account the displacement of the base
l2_a=link([-pi/2      0       0      0        ], 'standard');
l3_a=link([-pi/2      0       0      0.4      ], 'standard');
l4_a=link([pi/2       0       0      0        ], 'standard');
l5_a=link([pi/2       0       0      0.39     ], 'standard');
l6_a=link([-pi/2      0       0      0        ], 'standard');
l7_a=link([0          0       0      0        ], 'standard');


l_arm = robot({l1_a l2_a l3_a l4_a l5_a l6_a l7_a});
l_arm.name = 'Left arm';


l1_b=link([pi/2      0     0      0.310    ], 'standard'); %This first parameter was modified to take into account the displacement of the base
l2_b=link([-pi/2     0       0      0        ], 'standard');
l3_b=link([-pi/2     0       0      0.4      ], 'standard');
l4_b=link([pi/2      0       0      0        ], 'standard');
l5_b=link([pi/2      0       0      0.39     ], 'standard');
l6_b=link([-pi/2     0       0      0        ], 'standard');
l7_b=link([0         0       0      0        ], 'standard');

r_arm = robot({l1_b l2_b l3_b l4_b l5_b l6_b l7_b});
r_arm.name = 'Right arm';

%% Definitions for DQ_kinematics
kuka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
kuka_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];


kuka_DH_matrix = [kuka_DH_theta;
    kuka_DH_d;
    kuka_DH_a;
    kuka_DH_alpha];

kuka_kine = DQ_kinematics(kuka_DH_matrix, 'standard');


%% Initial conditions and plot setup
initial_theta1 =[-1.6965    2.2620    1.5708    1.4451   -0.4398    0.0628         0]';
initial_theta2 =[1.6336   -0.8168    1.5708    1.5080   -0.2513         0         0]';

thetastart =  [ initial_theta1; initial_theta2];
theta=thetastart;
epsilon = 0.01;

plotbotopt={'nobase','noname','noshadow','ortho'};


h=figure;

plot(l_arm,initial_theta1',plotbotopt{:});
hold on;
plot(r_arm,initial_theta2',plotbotopt{:});

grid off;
axis equal;
axis off;

view(180,0);

my_green = [0,0.7,0];

%translating the base of the left arm
rh = findobj('Tag', l_arm.name);
rr = get(rh, 'UserData');
rr.base = transl(-0.4,0,0 );
set(rh, 'UserData', rr);

% translating the base of the right arm
rh = findobj('Tag', r_arm.name);
rr = get(rh, 'UserData');
rr.base = transl(0.4, 0, 0);
set(rh, 'UserData', rr);

drawnow;

%% Setting up the two-arm system
% Each arm is translated from the origin
base1 = 1+DQ.E*0.5*DQ([0,-0.4,0,0]);
base2= 1+DQ.E*0.5*DQ([0, 0.4,0,0]);

two_arms = DQ_cdts(kuka_kine, base1, kuka_kine, base2);

%% Task definitions for grabbing the broom.

% The relative configuration between the hands must remain constant in order to minimize internal forces
dqrd = two_arms.xr(theta);

dqad_ant =  two_arms.xa(theta);
% The sweep motion consists of turning the broom around the torso's y axis
dqad =dqad_ant .* DQ([cos(pi/16);0;sin(pi/16);0]);


taskd=[dqad.q;dqrd.q];

%% Definition for the broom (with respect to the right hand)
d_broom = 0.9;
aux = translation(dqrd);
t_broom = aux*(1/norm(aux.q))*d_broom;
dqbroom = 1+DQ.E*0.5*t_broom;

initial_broom = translation(two_arms.x2(theta)* dqbroom);

broom_trans=translation(two_arms.x2(theta));



%% Drawing the broom
line_handle1=line([broom_trans.q(2),initial_broom.q(2)],[broom_trans.q(3),initial_broom.q(3)],[broom_trans.q(4),initial_broom.q(4)],'color',my_green,'linewidth',3);
plot(l_arm,initial_theta1');
hold on;
plot(r_arm,initial_theta2');




%% Two-arm control

epsilon = 0.01; %for the stop condition
error = epsilon+1;
i=0;
iter=1;
j=1;

two_arms.xr(theta)
two_arms.xa(theta)
two_arms.x1(theta)
two_arms.x2(theta)

two_arms.Jr(theta)
two_arms.Ja(theta)
pause()

%The sweep motion (back and forth) will be performed twice
while j <=4
    
    %standard control law
    nerror_ant = error;
    jacob = [two_arms.Ja(theta);two_arms.Jr(theta)];
    taskm=  [vec8(two_arms.xa(theta)); vec8(two_arms.xr(theta))];
    
    error = taskd-taskm;
    theta = theta+pinv(jacob)*0.5*error;
    
    
    
    
    % Visualisation
    plot(l_arm,theta(1:7)');
    plot(r_arm,theta(8:14)');
    aux = translation(two_arms.xr(theta));
    t_broom = aux*(1/norm(aux.q))*d_broom;
    dqbroom = 1+DQ.E*0.5*t_broom;
    broombase = translation(two_arms.x2(theta));
    brooptip = translation(two_arms.x2(theta)*dqbroom);
    set(line_handle1, 'Xdata', [broombase.q(2), brooptip.q(2)], 'Ydata', [broombase.q(3), brooptip.q(3)], 'Zdata', [broombase.q(4),brooptip.q(4)]);
    
   
 
    % Verify if the sweep direction can be changed
    if(norm(nerror_ant - error) < epsilon)
        %Change the task
        temp = dqad;
        dqad = dqad_ant;
        dqad_ant = temp;
        
        taskd(1:8,1)=dqad.q;
        j = j+1;
    end
end



