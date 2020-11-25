% Basic definitions for a free-flying robot with a radius of 0.25 m.

% (C) Copyright 2011-2020 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vilhena Adorno - adorno@ieee.org

classdef FreeFlyingRobot
    methods (Static)
        function robot = kinematics()
            % Create a new Free-Flying Robot           

            robot = DQ_FreeFlyingRobot();
            robot.set_radius(0.25);
        end
    end
end
