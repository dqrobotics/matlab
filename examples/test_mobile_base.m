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

robot_struct.robot_type = 'holonomic';

holonomic_base = DQ_HolonomicBase(robot_struct);

% mobile_base.this_robot_type
q = [1,1,0]';


T = 0.001;
gain = 50;
xd = holonomic_base.fkm([10,10,pi]);
%xd = holonomic_base.fkm([1,1,pi]);
plot(xd);
hold on;
handle = plot(holonomic_base.fkm(q));
% axis([-100,100,-100,100,0,1]);
axis square;
xlabel('X');
ylabel('Y');
x_error = vec8(xd - holonomic_base.fkm(q));

while norm(x_error) > 0.001    
    x = holonomic_base.fkm(q);
    J = holonomic_base.pose_jacobian(q);
    N = haminus8(xd)*DQ.C8*J;
    x_error = vec8(1 - x'*xd);
    
    u = pinv(N)*gain*x_error;
    q = q + T*u;
    
  % handle = plot(x, 'erase', handle);
  
   plot(holonomic_base,q);
   pause(0.1);
   drawnow;
    
end


