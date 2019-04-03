clear all;
close all;
clc;

arm1 = DQ_KUKA;
arm2 = DQ_KUKA;

% Initializes the robot with arm1
robot = DQ_WholeBody(arm1);
% Add another independent arm serially coupled to
robot.add(arm2);

q = zeros(14,1);
% robot.fkm(q), where q = [q1; q2], is equivalent to arm1.fkm(q1)*arm2.fkm(q2)
x = robot.fkm(q);
% Get the whole-body Jacobian
%J = robot.pose_jacobian(q);

% Plot the whole kinematic chain
plot(robot,q);
axis([-2,2,-2,2,-2,3]);
hold on;

for i=0:0.1:pi/2
    q(2) = i;
    plot(robot,q);
  %  pause(0.01);
    drawnow;
end