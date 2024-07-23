% CLASS DQ_VrepRobot - Abstract class with methods to send and receive 
% robot information to and from VREP.
%
% Usage:
%   Inherit from this class and implement the abstract methods.
%
%   DQ_VrepRobot Methods (Abstract):
%       set_configuration_space_positions - Sends the joint configurations to VREP
%       get_configuration_space_positions - Obtains the joint configurations from VREP

% (C) Copyright 2018-2024 DQ Robotics Developers
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
%     1. Murilo Marques Marinho - murilo@nml.t.u-tokyo.ac.jp
%       - Responsible for the original implementation.
%     2. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%       - Deprecated the following methods to ensure compatibility with the
%       C++ version of the class:
%             - 'send_q_to_vrep'
%             - 'get_q_from_vrep'
%       - Removed the following methods to ensure compatibility with the
%       C++ version of the class:
%             - 'kinematics'

classdef (Abstract) DQ_VrepRobot
    
    properties
        robot_name
        vrep_interface
    end
    
    methods (Abstract)
        set_configuration_space_positions(obj,q);
        q = get_configuration_space_positions(obj);
    end

    methods
        function send_q_to_vrep(obj, q)
            % For backwards compatibility only. Do not use this method.

            warning('Deprecated. Use set_configuration_space_positions() instead.')
            obj.set_configuration_space_positions(q)
        end

        function q = get_q_from_vrep(obj)
            % For backwards compatibility only. Do not use this method.

            warning('Deprecated. Use get_configuration_space_positions() instead.')
            q = obj.get_configuration_space_positions();
        end
    end
end

