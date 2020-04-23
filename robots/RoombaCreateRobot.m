% Basic definitions for the iRoomba Create Robot

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
%     Bruno Vihena Adorno - adorno@ufmg.br

classdef RoombaCreateRobot
    methods (Static)
        function robot = kinematics()
            % Create a new iRoomba Create Robot

            % The parameters below are given in meters
            wheel_radius = 65e-3; 
            distance_between_wheels = 260e-3;

            robot = DQ_DifferentialDriveRobot(wheel_radius,distance_between_wheels);
        end
    end
end
