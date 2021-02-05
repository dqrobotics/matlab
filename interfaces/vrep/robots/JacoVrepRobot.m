% CLASS JacoVrepRobot - Concrete class to interface with the "Jaco arm"
% robot in VREP.
%
% Usage:
%       1) Drag-and-drop a "Jaco arm" robot to a VREP scene.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = JacoVrepRobot("Jaco", vi);
%           >> vi.start_simulation();
%           >> robot.get_q_from_vrep();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "Jaco#0", a third robot, "Jaco#1", and so on.
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
%     Frederico Fernandes Afonso Silva - fredf.afonso@gmail.com

classdef JacoVrepRobot < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
    end
    
    methods
        function obj = JacoVrepRobot(robot_name,vrep_interface)
            %% Constructs an instance of a LBR4pVrepRobot
            %  >> vi = VrepInterface()
            %  >> vi.connect('127.0.0.1',19997);
            %  >> robot = JacoVrepRobot("Jaco", vi)
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'Jaco')
                error('Expected Jaco')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:6
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end
        
        function send_q_to_vrep(obj,q)
            %% Sends the joint configurations to VREP
            %  >> vrep_robot = JacoVrepRobot("Jaco", vi)
            %  >> q = zeros(6,1);
            %  >> vrep_robot.send_q_to_vrep(q)
            obj.vrep_interface.set_joint_target_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = JacoVrepRobot("Jaco", vi)
            %  >> q = vrep_robot.get_q_from_vrep()
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end
        
        function qd = get_qd_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = JacoVrepRobot("Jaco", vi)
            %  >> qd = vrep_robot.get_qd_from_vrep()
            qd = obj.vrep_interface.get_joint_velocities(obj.joint_names);
        end
        
        function send_tau_to_vrep(obj,tau)
            %% Sends the joint configurations to VREP
            %  >> vrep_robot = JacoVrepRobot("Jaco", vi)
            %  >> tau = zeros(6,1);
            %  >> vrep_robot.send_tau_to_vrep(tau)
            obj.vrep_interface.set_joint_torques(obj.joint_names,tau)
        end
        
        function tau = get_tau_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = JacoVrepRobot("Jaco", vi)
            %  >> tau = vrep_robot.get_tau_from_vrep()
            tau = obj.vrep_interface.get_joint_torques(obj.joint_names);
        end
        
        function kin = kinematics(obj)
            %% Obtains the DQ_SerialManipulator instance that represents this Jaco robot.
            %  >> vrep_robot = JacoVrepRobot("Jaco", vi)
            %  >> robot_kinematics = vrep_robot.kinematics()
            
            % Standard D-H of JACO j2n6s300
            jaco_DH_theta = [0, pi/2, pi/2, 0, 0, 0];
            jaco_DH_d = [-0.1188, 0, 0, -0.252, -0.079, -0.04];
            jaco_DH_a = [0, -0.41, 0, 0, 0, 0];
            jaco_DH_alpha = [-pi/2, -pi, -pi/2, -0.3056*pi, 0.3056*pi, pi];
            jaco_DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,6);
            jaco_DH_matrix = [jaco_DH_theta;
                jaco_DH_d;
                jaco_DH_a;
                jaco_DH_alpha;
                jaco_DH_type];
            
            kin = DQ_SerialManipulator(jaco_DH_matrix,'standard');
            
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
        end
        
    end
end