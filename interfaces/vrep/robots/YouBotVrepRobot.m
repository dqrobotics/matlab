% CLASS YouBotVrepRobot - Concrete class to interface with the "KUKA YouBot"
% robot in VREP.
%
% Usage:
%       1) Drag-and-drop a "KUKA YouBot" robot to a VREP scene.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = YouBotVrepRobot("youBot", vi);
%           >> vi.start_simulation();
%           >> robot.get_configuration();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "youBot#0", a third robot, "youBot#1", and so on.
%
%   YouBotVrepRobot Methods:
%       set_configuration - Sends the joint configurations to VREP
%       get_configuration - Obtains the joint configurations from VREP
%       kinematics - Obtains the DQ_Kinematics implementation of this robot

% (C) Copyright 2011-2024 DQ Robotics Developers
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
%        - Responsible for the original implementation
%     2. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%        - Updated for compatibility with the DQ_SerialVrepRobot class.
%     3. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%        - The class now inherits from YouBotCoppeliaSimRobot


classdef YouBotVrepRobot < YouBotCoppeliaSimRobot   
    methods
        function obj = YouBotVrepRobot(robot_name, vrep_interface)
            warning('Deprecated. Use YouBotCoppeliaSimRobot instead.')
            obj@YouBotCoppeliaSimRobot(robot_name, vrep_interface);
        end    
    end
end

