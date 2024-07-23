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
%           >> robot.get_configuration_space_positions();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "youBot#0", a third robot, "youBot#1", and so on.
%
%   YouBotVrepRobot Methods:
%       set_configuration_space_positions - Sends the joint configurations to VREP
%       get_configuration_space_positions - Obtains the joint configurations from VREP
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

classdef YouBotVrepRobot < DQ_SerialVrepRobot    
    properties (Constant)
        adjust = ((cos(pi/2) + DQ.i*sin(pi/2)) * (cos(pi/4) + DQ.j*sin(pi/4)))*(1+0.5*DQ.E*-0.1*DQ.k);
    end
    
    methods
        function obj = YouBotVrepRobot(robot_name, vrep_interface)
            obj@DQ_SerialVrepRobot("youBot", 7, robot_name, vrep_interface);
            
            %% youBot don't follow the standard name convention on CoppeliaSim. Also, the use of 'set_names()', as is done in the C++ implementation, is not supported on a constructor in MATLAB
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'youBot')
                error('Expected youBot')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:5
                current_joint_name = {robot_label,'ArmJoint',int2str(i-1),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
        end
        
        function set_configuration_space_positions(obj,q)
            %% Sends the joint configurations to VREP
            %  >> vrep_robot = YouBotVrepRobot("youBot", vi)
            %  >> q = zeros(8,1);
            %  >> vrep_robot.set_configuration_space_positions(q)
            x = q(1);
            y = q(2);
            phi = q(3);
            
            r = cos(phi/2.0)+DQ.k*sin(phi/2.0);
            p = x * DQ.i + y * DQ.j;
            pose = (1 + DQ.E*0.5*p)*r;
            
            obj.vrep_interface.set_joint_positions(obj.joint_names,q(4:8));
            obj.vrep_interface.set_object_pose(obj.base_frame_name, pose * obj.adjust');
        end
        
        function q = get_configuration_space_positions(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = YouBotVrepRobot("youBot", vi)
            %  >> q = vrep_robot.get_configuration_space_positions(q)
            base_x = obj.vrep_interface.get_object_pose(obj.base_frame_name) * obj.adjust;
            base_t = vec3(translation(base_x));
            base_phi = rotation_angle(rotation(base_x));
            base_arm_q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            
            q = [base_t(1); base_t(2); base_phi; base_arm_q];
        end
        
        function kin = kinematics(obj)
            %% Obtains the DQ_WholeBody instance that represents this youBot robot.
            %  >> vrep_robot = YouBotVrepRobot("youBot", vi)
            %  >> robot_kinematics = vrep_robot.kinematics()
            
            include_namespace_dq
            % The DH parameters and other geometric parameters are based on
            % Kuka's documentation:
            % http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications
            % https://www.generationrobots.com/img/Kuka-YouBot-Technical-Specs.pdf
            pi2 = pi/2;
            arm_DH_theta = [    0,    pi2,       0,      pi2,        0];
            arm_DH_d =   [  0.147,      0,       0,        0,    0.218];
            arm_DH_a =   [  0.033,  0.155,   0.135,        0,        0];
            arm_DH_alpha =   [pi2,      0,       0,      pi2,        0];
            arm_DH_type = double(repmat(DQ_JointType.REVOLUTE,1,5));
            arm_DH_matrix = [arm_DH_theta;
                arm_DH_d;
                arm_DH_a;
                arm_DH_alpha
                arm_DH_type];
            
            arm =  DQ_SerialManipulatorDH(arm_DH_matrix);
            base = DQ_HolonomicBase();
            
            x_bm = 1 + E_*0.5*(0.165*i_ + 0.11*k_);
            
            base.set_frame_displacement(x_bm);
            
            kin = DQ_WholeBody(base);
            kin.add(arm);
            
            effector = 1 + E_*0.5*0.3*k_;
            kin.set_effector(effector);
        end        
    end
end

