% CLASS DQ_VrepRobot - Abstract class with methods to send and receive 
% robot information to and from VREP.
%
% Usage:
%   Inherit from this class and implement the abstract methods.
%
%   DQ_VrepRobot Methods (Abstract):
%       set_configuration - Sends the joint configurations to VREP
%       get_configuration - Obtains the joint configurations from VREP

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
%     3. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%        - The class now inherits from DQ_CoppeliaSimRobot

classdef (Abstract) DQ_VrepRobot < DQ_CoppeliaSimRobot
    methods
        function obj = DQ_VrepRobot()
            warning('Deprecated. Use DQ_CoppeliaSimRobot instead.')
            obj@DQ_CoppeliaSimRobot();
        end
    end
end

