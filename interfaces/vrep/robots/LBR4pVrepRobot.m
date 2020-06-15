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
%           >> robot.get_q_from_vrep();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "LBR4p#0", a third robot, "LBR4p#1", and so on.
%
%   LBR4pVrepRobot Methods:
%       send_q_to_vrep - Sends the joint configurations to VREP
%       get_q_from_vrep - Obtains the joint configurations from VREP
%       kinematics - Obtains the DQ_Kinematics implementation of this robot

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

classdef LBR4pVrepRobot < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
    end
    
    methods
        function obj = LBR4pVrepRobot(robot_name,vrep_interface)
            %% Constructs an instance of a LBR4pVrepRobot
            %  >> vi = VrepInterface()
            %  >> vi.connect('127.0.0.1',19997);
            %  >> robot = LBR4pVrepRobot("LBR4p", vi)
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'LBR4p')
                error('Expected LBR4p')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:7
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end
        
        function send_q_to_vrep(obj,q)
            %% Sends the joint configurations to VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> q = zeros(7,1);
            %  >> vrep_robot.send_q_to_vrep(q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> q = vrep_robot.get_q_from_vrep(q)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end
        
        function kin = kinematics(obj)
            %% Obtains the DQ_SerialManipulator instance that represents this LBR4p robot.
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> robot_kinematics = vrep_robot.kinematics()
            
            LBR4p_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
            LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
            LBR4p_DH_matrix = [LBR4p_DH_theta;
                LBR4p_DH_d;
                LBR4p_DH_a;
                LBR4p_DH_alpha];
            
            kin = DQ_SerialManipulator(LBR4p_DH_matrix,'standard');
            
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_effector(1+0.5*DQ.E*DQ.k*0.07);
        end
        
    end
end

