% (C) Copyright 2011-2019 DQ Robotics Developers
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

classdef LittleJohnRobot
    methods (Static)
        function robot = kinematics()
            % Mobile manipulator composed of a nonholonomic mobile base (Create)
            % serially coupled to a 5-DOF arm (made of AX18 servos)
            arm = Ax18ManipulatorRobot.kinematics();
            base = RoombaCreateRobot.kinematics();

            include_namespace_dq

            % Changing the reference due to the mobile base height
            x = 1 + E_*(1/2)*170e-3*k_;

            base.set_frame_displacement(x);

            robot = DQ_WholeBody(base);
            robot.add(arm);
        end
    end
end
