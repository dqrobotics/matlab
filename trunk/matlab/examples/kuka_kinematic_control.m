close all;
clear all;
clear classes;
clc;

disp('WARNING: this example requires Peter Corke"s Robotics Toolbox');

%DH for the Kuka LWR
%L =LINK([alpha     A   theta   D)
l1_a=link([pi/2      0       0      0.310], 'standard');
l2_a=link([-pi/2    0       0      0        ], 'standard');
l3_a=link([-pi/2    0       0      0.4     ], 'standard');
l4_a=link([pi/2      0       0      0        ], 'standard');
l5_a=link([pi/2      0       0      0.39   ], 'standard');
l6_a=link([-pi/2    0       0      0        ], 'standard');
l7_a=link([0           0       0      0        ], 'standard');

%Definitions of the arm
kuka1 =robot({l1_a l2_a l3_a l4_a l5_a l6_a l7_a});
kuka1.name = 'Kuka 1';

%Standard D-H of KUKA-LWR
kuka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
kuka_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
kuka_DH_matrix = [kuka_DH_theta;
                  kuka_DH_d;
                  kuka_DH_a;
                  kuka_DH_alpha];
                              
kuka_kine = DQ_kinematics(kuka_DH_matrix,'standard');


thetastart =[0    0.3770    0.1257   -0.5655         0         0         0]';
theta = thetastart;

figure;
plotbotopt={'nobase','noname','noshadow','ortho'};
plot(kuka1, theta',plotbotopt{:});
grid off;
%axis off;
view(-0,0)
hold on;

epsilon = 0.001; %The error must be bellow this value in order to stop the robot
thetad = [1.7593    0.8796    0.1257   -1.4451   -1.0053    0.0628         0]';


xd = kuka_kine.fkm(thetad);
xm = kuka_kine.fkm(theta);

plot(kuka1, theta');
    

fprintf('Performing standard kinematic control using dual quaternion coordinates');
gain = 0.1;
theta = thetastart;
error = epsilon+1;

while norm(error) > epsilon
    jacob = kuka_kine.jacobian(theta);
    xm= kuka_kine.fkm(theta);
    error = vec8(xd-xm);
    theta = theta+pinv(jacob)*gain*error;
    plot(kuka1, theta');    
end

fprintf('\nNow let us control only the translation part\n');
%The end-effector will touch the base
pd = [0,0,0,0];

error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka_kine.jacobian(theta);
    xm = kuka_kine.fkm(theta);
    jacobp = kuka_kine.jacobp(jacob,xm);
    pm = translation(xm);
    error = vec4(pd-pm);    
    theta = theta+pinv(jacobp)*gain*error;
    plot(kuka1, theta');       
end

fprintf('\nNow let us control only the orientation\n')

%The end-effector will be aligned with the world frame
rd = DQ(1);

error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka_kine.jacobian(theta);
    xm = kuka_kine.fkm(theta);
    jacobr = jacob(1:4,:);
    rm = xm.P;
    error = vec4(rd-rm);    
    theta = theta+pinv(jacobr)*gain*error;
    plot(kuka1, theta');       
end


fprintf('\nNow let us place the end-effector at a distance of 0.2 m from the base (we are going to perform distance control)\n')

%Technically speaking, we're controlling the square of the distance,
%otherwise the distance Jacobian can have singularities. See discussion on page 76 of 
%ADORNO, B. V., Two-arm manipulation: from manipulators to enhanced human-robot
% collaboration, Université Montpellier 2, Montpellier, France, 2011.
dd=0.2^2;
error = epsilon+1;
while norm(error) > epsilon
    jacob = kuka_kine.jacobian(theta);
    xm = kuka_kine.fkm(theta);
    jacobd = kuka_kine.jacobd(jacob,xm);
    dm = norm(vec4(translation(xm)))^2;
    error = dd-dm;    
    theta = theta+pinv(jacobd)*gain*error;
    plot(kuka1, theta');       
end





