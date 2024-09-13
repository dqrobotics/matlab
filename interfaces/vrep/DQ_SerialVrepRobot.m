% CLASS DQ_SerialVrepRobot - Concrete class to interface with serial robots
% in VREP.
%
% Usage:
%       1) Drag-and-drop a serial robot to a VREP scene. For instance, a
%       "my_robot" robot.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot", vi);
%           >> vi.start_simulation();
%           >> vrep_robot.get_configuration();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "my_robot#0", a third robot, "my_robot#1", and so on.
%
%   DQ_SerialVrepRobot Methods:
%       get_joint_names - Gets the joint names of the robot in the CoppeliaSim scene.
%       set_configuration - Sets the joint configurations of the robot in the CoppeliaSim scene.
%       get_configuration - Gets the joint configurations of the robot in the CoppeliaSim scene.
%       set_target_configuration - Sets the joint configurations of the robot in the CoppeliaSim scene as a target configuration for the joint controllers.
%       get_configuration_velocities - Gets the joint velocities of the robot in the CoppeliaSim scene.
%       set_target_configuration_velocities - Sets the joint velocities of the robot in the CoppeliaSim scene as a target velocity for the joint controllers.
%       set_configuration_space_torques - Sets the joint torques of the robot in the CoppeliaSim scene.
%       get_configuration_space_torques - Gets the joint torques of the robot in the CoppeliaSim scene.

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
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     1. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%        - Responsible for the original implementation, based on the C++ version:
%           - DQ_VrepInterface.h: https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_VrepInterface.h
%           - DQ_SerialVrepRobot.cpp: https://github.com/dqrobotics/cpp-interface-vrep/blob/master/src/dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.cpp

classdef DQ_SerialVrepRobot < DQ_SerialCoppeliaSimRobot
    methods
        function obj = DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface)
            warning('Deprecated. Use DQ_SerialCoppeliaSimRobot instead.')
            obj@DQ_SerialCoppeliaSimRobot(base_robot_name, robot_dof, robot_name, vrep_interface)
        end
    end
end