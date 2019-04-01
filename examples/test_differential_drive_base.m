% Simple example to test the DQ_MobileBase class. Still under development.

% (C) Copyright 2019 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
%
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

clear all;
close all;
clc;

param.wheel_radius = 0.1;
param.distance_between_wheels = 2;
base = DQ_DifferentialDriveRobot(param);
base.name = 'Roomba';

% mobile_base.this_robot_type
q = [1,1,0]';
T = 0.001;
gain = 50;
xd = base.fkm([10,10,3*pi/2]);
plot(xd);
hold on;
handle = plot(base.fkm(q));
axis([0,20,-15,15,0,1]);
axis square;
xlabel('X');
ylabel('Y');
x_error = 10;

u = [0;0];
vel = 100;

while true
    figure(gcf);
    q = q + T*base.constraint_jacobian(q(3))*u;
    plot(base,q);
    key = get(gcf,'CurrentCharacter');
  %  key = get(gcf, 'KeyPressFcn')
    
    switch key
        case 'i'
            disp('Forward');
            u = u + vel;
        case 'k'
            disp('Backwards');
            u = u - vel;
        case 'j'
            disp('Left');
            u = u - [-vel;vel];
        case 'l'
            disp('Right');
            u = u + [-vel;vel];
    end
   set(gcf,'CurrentCharacter','@');

    drawnow;
    figure(gcf);
end

while norm(x_error) > 0.001
    if rand(1) > 0.30
        xd = -xd;
    end

    x = base.fkm(q);
    J = base.pose_jacobian(q);
    N = -haminus8(xd)*DQ.C8*J;
    
    x_error_positive = vec8(x'*xd - 1);
    x_error_negative = vec8(x'*xd + 1);
    
    if norm(x_error_positive) <= norm(x_error_negative)
       x_error = x_error_positive;
         disp('Positive');
    else
         x_error = x_error_negative;
        disp('Negative');
    end
    
    u = pinv(N)*gain*x_error;
    q = q + T*base.constraint_jacobian(q(3))*u;
    
    % handle = plot(x, 'erase', handle);
    plot(base,q);
    % pause()
    drawnow;
end


