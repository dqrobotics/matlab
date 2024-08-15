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

classdef DQ_SerialVrepRobot < DQ_VrepRobot
    properties
        joint_names;
        base_frame_name;
    end

    methods
        function obj = DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface)
            % This method constructs an instance of a DQ_SerialVrepRobot.
            % Usage:
            %     DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface);  
            %          base_robot_name: The base name of the robot in the CoppeliaSim scene.
            %          robot_dof: The number of DoF of the robot in the CoppeliaSim scene.
            %          robot_name: The name of the robot in the CoppeliaSim scene.
            %          vrep_interface: The DQ_VrepInterface object connected to the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     
            %     Note that the name of the robot should be EXACTLY the same as in the CoppeliaSim
            %     scene. For instance, if you drag-and-drop a second robot, its name will become 
            %     "my_robot#0", a third robot, "my_robot#1", and so on.

            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;

            %% The use of 'initialize_joint_names_from_vrep()', as is done in the C++ implementation, is not supported on a constructor in MATLAB
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(obj.robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label, base_robot_name)
                error('Expected %s', base_robot_name)
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names, link names, and base frame name
            obj.joint_names = {};
            for i=1:robot_dof
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end

        function joint_names = get_joint_names(obj)
            % This method gets the joint names of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_joint_names;
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     joint_names = vrep_robot.get_joint_names;

            joint_names = obj.joint_names;
        end

        function set_configuration_space_positions(obj, q)
            % This method sets the joint configurations of the robot in the CoppeliaSim scene.
            % Usage:
            %     set_configuration_space_positions(q);  
            %          q: The joint configurations of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     q = zeros(7,1);
            %     vrep_robot.set_configuration_space_positions(q);
            %     
            %     Note that this calls "set_joint_positions" in DQ_VrepInterface, meaning that it
            %     is only suitable for joints in kinematic mode.

            obj.vrep_interface.set_joint_positions(obj.joint_names, q)
        end
        
        function q = get_configuration_space_positions(obj)
            % This method gets the joint configurations of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_space_positions;  
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     q = vrep_robot.get_configuration_space_positions;

            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end

        function set_target_configuration_space_positions(obj, q_target)
            % This method sets the joint configurations of the robot in the CoppeliaSim scene as a target configuration for the joint controllers.
            % Usage:
            %     set_target_configuration_space_positions(q_target);  
            %          q_target: The target joint configurations of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     q_target = zeros(7,1);
            %     vrep_robot.set_target_configuration_space_positions(q_target);
            %     
            %     Note that this calls "set_joint_target_positions" in DQ_VrepInterface, meaning that it
            %     is only suitable for joints in dynamic mode with position control.

            obj.vrep_interface.set_joint_target_positions(obj.joint_names, q_target)
        end
        
        function qd = get_configuration_space_velocities(obj)
            % This method gets the joint velocities of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_space_velocities;  
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     qd = vrep_robot.get_configuration_space_velocities;
            
            qd = obj.vrep_interface.get_joint_velocities(obj.joint_names);
        end

        function set_target_configuration_space_velocities(obj, v_target)
            % This method sets the joint velocities of the robot in the CoppeliaSim scene as a target velocity for the joint controllers.
            % Usage:
            %     set_target_configuration_space_velocities(v_target);  
            %          v_target: The target joint velocities of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     v_target = zeros(7,1);
            %     vrep_robot.set_target_configuration_space_velocities(v_target);
            %     
            %     Note that this calls "set_joint_target_velocities" in DQ_VrepInterface, meaning that it
            %     is only suitable for joints in dynamic mode with velocity control.

            obj.vrep_interface.set_joint_target_velocities(obj.joint_names, v_target);
        end

        function set_configuration_space_torques(obj,tau)
            % This method sets the joint torques of the robot in the CoppeliaSim scene.
            % Usage:
            %     set_configuration_space_torques(tau);  
            %          tau: The joint torques of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     tau = zeros(7,1);
            %     vrep_robot.set_configuration_space_torques(tau);
            %     
            %     Note that this calls "set_joint_torques" in DQ_VrepInterface, meaning that it
            %     is only suitable for joints in dynamic mode with force/torque control.

            obj.vrep_interface.set_joint_torques(obj.joint_names,tau)
        end

        function tau = get_configuration_space_torques(obj)
            % This method gets the joint torques of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_space_torques;  
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     tau = vrep_robot.get_configuration_space_torques;

            tau = obj.vrep_interface.get_joint_torques(obj.joint_names);
        end
    end
end