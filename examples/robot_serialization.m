clear all;
close all;
clc;

include_namespace_dq;

base = DQ_HolonomicBase;
arm1 = DQ_KUKA;
arm1.name = 'base';
arm2 = DQ_KUKA;
arm3 = DQ_KUKA;
arm4 = DQ_KUKA;

% Initializes the robot with arm1
robot = DQ_WholeBody(arm1);
%robot.add(arm1)

% Add another independent arm serially coupled to
robot.add(arm2);
robot.add(arm3);
robot.add(arm4)

q = zeros(28,1);
% robot.fkm(q), where q = [q1; q2], is equivalent to arm1.fkm(q1)*arm2.fkm(q2)
x = robot.fkm(q);
% Get the whole-body Jacobian
%J = robot.pose_jacobian(q);

% Plot the whole kinematic chain
plot(DQ(1));
hold on;
plot(arm1.base_frame);
plot(robot,q);
axis([-10,1,0,2,0,5]);
hold on;

for i=0:0.1:pi/2
    q(2) = i;
    plot(robot,q);
  %  pause(0.01);
    drawnow;
end