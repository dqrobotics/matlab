% CLASS LBR4pVrepRobot - Concrete class to interface with the "KUKA LBR4+"
% robot in VREP.
%
% Usage:
%       1) Drag-and-drop a "KUKA LBR4+" robot to a VREP scene.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = LBR4pVrepRobot("LBR4p", vi);
%           >> vi.start_simulation();
%           >> robot.get_configuration_space_positions();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "LBR4p#0", a third robot, "LBR4p#1", and so on.
%
%   LBR4pVrepRobot Methods:
%       kinematics - Obtains the DQ_Kinematics implementation of this robot

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
%        - Responsible for the original implementation
%     2. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%        - Updated for compatibility with the DQ_SerialVrepRobot class.

classdef LBR4pVrepRobot < DQ_SerialVrepRobot
    methods
        function obj = LBR4pVrepRobot(robot_name, vrep_interface)
            obj@DQ_SerialVrepRobot("LBR4p", 7, robot_name, vrep_interface);
        end

        function kin = kinematics(obj)
            %% Obtains the DQ_SerialManipulator instance that represents this LBR4p robot.
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi);
            %  >> robot_kinematics = vrep_robot.kinematics();
            
            LBR4p_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
            LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
            LBR4p_DH_type = double(repmat(DQ_JointType.REVOLUTE,1,7));
            LBR4p_DH_matrix = [LBR4p_DH_theta;
                LBR4p_DH_d;
                LBR4p_DH_a;
                LBR4p_DH_alpha
                LBR4p_DH_type];
            
            kin = DQ_SerialManipulatorDH(LBR4p_DH_matrix);
            
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_effector(1+0.5*DQ.E*DQ.k*0.07);
        end        
    end
end

