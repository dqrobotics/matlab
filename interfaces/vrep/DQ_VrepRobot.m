% CLASS DQ_VrepRobot - Abstract class with methods to send and receive 
% robot information to and from VREP.
%
% Usage:
%   Inherit from this class and implement the abstract methods.
%
%   DQ_VrepRobot Methods:
%       send_q_to_vrep - Sends the joint configurations to VREP
%       get_q_from_vrep - Obtains the joint configurations from VREP
%       kinematics - Obtains the DQ_Kinematics implementation of this
%       robot

% (C) Copyright 2020 DQ Robotics Developers
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
%     Murilo Marques Marinho - murilo@nml.t.u-tokyo.ac.jp

classdef (Abstract) DQ_VrepRobot
    
    properties
        robot_name
        vrep_interface
    end
    
    methods (Abstract)
        send_q_to_vrep(obj,q);
        q = get_q_from_vrep(obj);
        kin = kinematics(obj);
    end
end

