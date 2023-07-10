% CLASS FrankaEmikaPandaVrepRobot - Concrete class to interface with the
% "FrankaEmikaPanda" robot in VREP.
%
% Usage:
%       1) Drag-and-drop a "FrankaEmikaPanda" robot to a VREP scene.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi);
%           >> vi.start_simulation();
%           >> robot.get_q_from_vrep();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "FrankaEmikaPanda#0", a third robot,
%       "FrankaEmikaPanda#1", and so on.
%
%   LBR4pVrepRobot Methods:
%       send_q_to_vrep - Sends the joint configurations to VREP
%       get_q_from_vrep - Obtains the joint configurations from VREP
%       get_q_dot_from_vrep - Obtains the robot generalized velocities from V-REP.
%       send_tau_to_vrep - Sends the joint torques to VREP
%       get_tau_from_vrep - Obtains the joint torques from VREP
%       kinematics - Obtains the DQ_Kinematics implementation of this robot

% (C) Copyright 2011-2023 DQ Robotics Developers
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
%     Frederico Fernandes Afonso Silva - frederico.silva@ieee.org

classdef FrankaEmikaPandaVrepRobot < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
        force_sensor_names;
        link_names;
        lua_script_name;
    end
    
    methods
        function obj = FrankaEmikaPandaVrepRobot(robot_name,vrep_interface)
            %% Constructs an instance of a FrankaEmikaPandaVrepRobot
            %  >> vi = VrepInterface()
            %  >> vi.connect('127.0.0.1',19997);
            %  >> robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'Franka')
                error('Expected Franka')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names, link names, and base frame name
            obj.joint_names = {};
            for i=1:7
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
                
                current_link_name = {robot_label,'_link',int2str(i+1),robot_index};
                obj.link_names{i} = strjoin(current_link_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end
        
        function send_q_to_vrep(obj,q)
            %% Sends the joint configurations to VREP
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> q = zeros(7,1);
            %  >> vrep_robot.send_q_to_vrep(q)
            obj.vrep_interface.set_joint_target_positions(obj.joint_names,q);
        end
        
        function q = get_q_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> q = vrep_robot.get_q_from_vrep()
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end
        
        function send_q_dot_to_vrep(obj,q_dot)
            %% Sends the joint velocities to VREP
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> q_dot = zeros(7,1);
            %  >> vrep_robot.send_q_dot_to_vrep(q_dot)
            obj.vrep_interface.set_joint_target_velocities(obj.joint_names,q_dot);
        end
        
        function qd = get_q_dot_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> qd = vrep_robot.get_q_dot_from_vrep()
            qd = obj.vrep_interface.get_joint_velocities(obj.joint_names);
        end
        
        function send_tau_to_vrep(obj,tau)
            %% Sends the joint torques to VREP
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> tau = zeros(7,1);
            %  >> vrep_robot.send_tau_to_vrep(tau)
            obj.vrep_interface.set_joint_torques(obj.joint_names,tau)
        end
        
        function tau = get_tau_from_vrep(obj)
            %% Obtains the joint torques from VREP
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> tau = vrep_robot.get_tau_from_vrep()
            tau = obj.vrep_interface.get_joint_torques(obj.joint_names);
        end
        
        function kin = kinematics(obj)
            %% Obtains the DQ_SerialManipulator instance that represents this Franka Emika Panda robot.
            %  >> vrep_robot = FrankaEmikaPandaVrepRobot('Franka', vi)
            %  >> robot_kinematics = vrep_robot.kinematics()
            
            % Franka Emika Panda
            % Create a DQ_SerialManipulator object
            kin = FrankaEmikaPandaRobot.kinematics;
            
            % Update base and reference frame with V-REP values
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));

            % Set the end-effector
            kin.set_effector(1+0.5*DQ.E*DQ.k*0.1820);
        end        
    end
end