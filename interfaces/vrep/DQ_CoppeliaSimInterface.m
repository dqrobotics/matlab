% (C) Copyright 2024 DQ Robotics Developers
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
%
%     1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%        - Responsible for the original implementation.

classdef (Abstract)  DQ_CoppeliaSimInterface < handle
    
    properties (Access = protected) 
    end
    
    methods (Access = protected) 
        % function obj = DQ_CoppeliaSim(inputArg1,inputArg2)
        % end
    end
        
    methods(Abstract)
        % This method connects to CoppeliaSim.
        % Calling this function is required before anything else can happen.
        connect(obj, host, port, TIMEOUT_IN_MILISECONDS);

        % This method enables or disables the stepped (synchronous) mode
        % for the remote API server service that the client is connected to.
        % Example:
        %       set_stepping_mode(true)    % stepping mode enabled
        %       set_stepping_mode(false)   % stepping mode disabled
        set_stepping_mode(obj, flag);

        % This method sends trigger signal to the CoppeliaSim scene, 
        % which performs a simulation step when the stepping mode is used.
        trigger_next_simulation_step(obj);

        % This method starts the CoppeliaSim simulation.
        start_simulation(obj);

        % This method stops the CoppeliaSim simulation.
        stop_simulation(obj);

        % This method gets the handles for a cell array of 
        % object names in the the CoppeliaSim scene.
        handles = get_handles(obj,names);

        % This method gets the handle for a given object in the CoppeliaSim scene. 
        handle = get_handle(obj,name);

        % This method gets the translation of an object in the CoppeliaSim scene.
        t = get_object_translation(obj,objectname);

        % This method sets the translation of an object in the CoppeliaSim scene.
        set_object_translation(obj,objectname,translation);

        % This method gets the rotation of an object in the CoppeliaSim scene.
        r = get_object_rotation(obj, objectname);

        % This method sets the rotation of an object in the CoppeliaSim scene.
        set_object_rotation(obj,objectname,rotation);

        % This method gets the pose of an object in the CoppeliaSim scene. 
        x = get_object_pose(obj,objectname);

        % This method sets the pose of an object in the CoppeliaSim scene. 
        set_object_pose(obj,objectname,pose);

        % This method sets the joint positions of a robot in the CoppeliaSim scene.        
        set_joint_positions(obj,jointnames,joint_positions);

        % This method sets the joint positions of a robot in the CoppeliaSim scene. 
        joint_positions = get_joint_positions(obj,jointnames);

        % This method sets the joint target positions of a robot in the CoppeliaSim scene. 
        % It is required a dynamics enabled scene, and joints in dynamic mode 
        % with position control mode. 
        % information about joint modes:
        % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
        set_joint_target_positions(obj,jointnames,joint_target_positions);

        % This method gets the joint velocities of a robot in the CoppeliaSim scene. 
        joint_velocities = get_joint_velocities(obj,jointnames);

        % This method sets the joint target velocities of a robot in the CoppeliaSim scene.
        % It is required a dynamics enabled scene, and joints in dynamic mode 
        % with velocity control mode. Check this link for more
        % information about joint modes:
        % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
        set_joint_target_velocities(obj,jointnames,joint_target_velocities);

        % This method sets the joint torques of a robot in the CoppeliaSim scene.
        % It is required a dynamics enabled scene, and joints in dynamic mode 
        % with velocity or force/torque control mode. Check this link for more
        % information about joint modes:
        % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
        set_joint_torques(obj,jointnames,torques);  

        % This method gets the joint torques of a robot in the CoppeliaSim scene.
        joint_torques = get_joint_torques(obj,jointnames);
    end
end


