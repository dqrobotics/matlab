% CLASS DQ_VrepInterface - Communicate with V-REP using dual quaternions.
%
% Installation:
%   1) Enable V-REP's remote API on the Server Side:
%   http://www.coppeliarobotics.com/helpFiles/en/remoteApiServerSide.htm
%       - Port 19997 is enabled by default, please refer to the V-REP
%       documentation if you need more ports.
%   2) Enable V-REP's remote API on the Client Side:
%   http://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm
%       You have to add two folders to your MATLAB path. For example, on
%       64bit Windows:
%           - YOUR_VREP_FOLDER\programming\remoteApiBindings\matlab\matlab
%           - YOUR_VREP_FOLDER\programming\remoteApiBindings\lib\lib\Windows\64Bit
% For more information refer to the remote API documentation.
%
% Usage:
%   If your installation is done correctly, the following minimal example
%   will start the V-REP simulation, sleep for one second, and stop the
%   simulation:
%       1) Open V-REP with the default scene
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vi.start_simulation();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%
% Familiarizing yourself with V-REP's remote API terminology might be
% helpful to fully understand the documentation.
% http://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm
%
%   DQ_VrepInterface Methods:
%       connect - Connects to a V-REP Remote API Server
%       disconnect - Disconnects from currently connected server
%       disconnect_all - Flushes all Remote API connections
%       start_simulation - Start V-REP simulation
%       stop_simulation - Stop V-REP simulation
%       get_object_translation - Get object translation as a pure
%       quaternion
%       set_object_translation - Set object translation with a pure
%       quaternion
%       get_object_rotation - Get object rotation as a unit quaternion
%       set_object_rotation - Set object rotation with a unit quaternion
%       get_object_pose - Get object pose as a unit dual quaternion
%       set_object_pose - Set object pose with a unit dual quaternion
%       set_joint_positions - Set the joint positions of a robot
%       set_joint_target_positions - Set the joint target positions of a
%       robot
%       get_joint_positions - Get the joint positions of a robot
%
%   DQ_VrepInterface Methods (For advanced users)
%       get_handle - Get the handle of a V-REP object
%       get_handles - Get the handles for multiple V-REP objects
%

% (C) Copyright 2018-2019 DQ Robotics Developers
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

classdef DQ_VrepInterface < handle
    
    properties (Access = private)
        % the V-REP remote API instance used by this interface
        vrep;
        % the client ID of this remote API connection
        clientID;
        % a map between V-REP object names and DQ_VrepInterfaceMapElements
        handles_map;
    end
    
    properties (Constant)
        % Constant that denotes the V-VREP's remote API blocking operation mode
        OP_BLOCKING  = remApi('remoteApi').simx_opmode_blocking;
        % Constant that denotes the V-VREP's remote API streaming operation mode
        OP_STREAMING = remApi('remoteApi').simx_opmode_streaming;
        % Constant that denotes the V-VREP's remote API oneshot operation mode
        OP_ONESHOT   = remApi('remoteApi').simx_opmode_oneshot;
        % Constant that denotes the V-VREP's remote API buffer operation mode
        OP_BUFFER    = remApi('remoteApi').simx_opmode_buffer;
    end
    
    methods (Access = private)
        
        function handle = handle_from_string_or_handle(obj,name_or_handle)
            if(ischar(name_or_handle))
                name = name_or_handle;
                if(obj.handles_map.isKey(name))
                    handle = obj.handles_map(name).handle;
                else
                    handle = obj.get_handle(name);
                    obj.handles_map(name) = DQ_VrepInterfaceMapElement(handle);
                end
                
            else
                handle=name_or_handle;
            end
        end
        
        function element = element_from_string(obj,name)
            obj.handle_from_string_or_handle(name); %Add new handle if needed
            element = obj.handles_map(name);
        end
    end
    
    methods
        
        function obj = DQ_VrepInterface()
            obj.vrep=remApi('remoteApi');
            obj.handles_map = containers.Map;
            obj.clientID = -1;
            disp(['This version of DQ Robotics DQ_VrepInterface is compatible'...
                ' with VREP 3.5.0']);
        end
        
        function connect(obj,ip,port)
            %% Connects to a V-REP remote API server on a given ip and port
            obj.clientID = obj.vrep.simxStart(ip,port,true,true,5000,5);
            if (obj.clientID>-1)
                disp('Connected to the remote API server');
            else
                error('Failed to connect to remote API server');
            end
        end
        
        %% Close
        function disconnect(obj)
            %% Disconnects from the V-REP remote API server
            obj.vrep.simxFinish(obj.clientID);
        end
        
        %% Close all
        function disconnect_all(obj)
            %% Flushes all V-REP remote API connections from the server
            obj.vrep.simxFinish(-1);
        end
        
        %% Start Simulation
        function start_simulation(obj)
            %% Starts the V-REP simulation
            obj.vrep.simxStartSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot);
        end
        
        %% Stop Simulation
        function stop_simulation(obj)
            %% Stops the V-REP simulation
            obj.vrep.simxStopSimulation(obj.clientID,obj.vrep.simx_opmode_blocking);
        end
        
        %% Get Handles
        function handles = get_handles(obj,names)
            %% Get the V-REP handles for a cell array of object names
            handles = [];
            if(iscell(names))
                for i=1:length(names)
                    [~,handles(i)] = obj.vrep.simxGetObjectHandle(...
                        obj.clientID,...
                        char(names(i)),...
                        obj.vrep.simx_opmode_blocking);
                end
            else
                error('Error in get_handles: argument names must be of type cell, e.g. names = [{joint1,joint2}];');
            end
        end
        
        %% Get Handle
        function handle = get_handle(obj,name)
            %% Get the V-REP handle for a given object
            [~,handle] = obj.vrep.simxGetObjectHandle(...
                obj.clientID,...
                name,...
                obj.vrep.simx_opmode_blocking);
        end
        
        %% Get Object Translation
        function t = get_object_translation(obj,handle,relative_to_handle,opmode)
            %% Get the translation of an object in V-REP
            %%  >> t = vi.get_object_translation('DefaultCamera');
            
            % First approach to the auto-management using
            % DQ_VrepInterfaceMapElements. If the user does not specify the
            % opmode, it is chosen first as STREAMING and then as BUFFER,
            % as specified by the remote API documentation
            if nargin <= 2
                element = obj.element_from_string(handle);
                if(~element.state_from_function_signature('get_object_translation'))
                    [~,object_position]  = obj.vrep.simxGetObjectPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handle),...
                        -1,...
                        obj.OP_STREAMING);
                    % We need to check the buffer until it is not empty,
                    % TODO add a timeout.
                    retval = 1;
                    while retval==1
                        [retval,object_position] = obj.vrep.simxGetObjectPosition(...
                            obj.clientID,...
                            obj.handle_from_string_or_handle(handle),...
                            -1,...
                            obj.OP_BUFFER);
                    end
                else
                    [~,object_position]  = obj.vrep.simxGetObjectPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handle),...
                        -1,...
                        obj.OP_BUFFER);
                end
            else
                [~,object_position]  = obj.vrep.simxGetObjectPosition(...
                    obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    opmode);
            end
            t = DQ([0,double(object_position)]);
        end
        
        %% Set Object Translation
        function set_object_translation(obj,handle,t,relative_to_handle,opmode)
            %% Set the translation of an object in V-REP
            %%  >> t = DQ.i*0.01;
            %%  >> vi.set_object_translation('DefaultCamera',t);
            
            if nargin == 3
                obj.vrep.simxSetObjectPosition(obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    -1,...
                    t.q(2:4),...
                    obj.OP_ONESHOT);
            else
                obj.vrep.simxSetObjectPosition(obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    t.q(2:4),...
                    opmode);
            end
        end
        
        %% Get Object Rotation
        function r = get_object_rotation(obj, handle, relative_to_handle, opmode)
            %% Get the rotation of an object in V-REP
            %%  >> r = vi.get_object_rotation('DefaultCamera');
            
            % Create some aliases
            id = obj.clientID;
            handle1 = obj.handle_from_string_or_handle(handle);
            
            
            % First approach to the auto-management using
            % DQ_VrepInterfaceMapElements. If the user does not specify the
            % opmode, it is chosen first as STREAMING and then as BUFFER,
            % as specified by the remote API documentation
            if nargin <= 2
                element = obj.element_from_string(handle);
                if(~element.state_from_function_signature('get_object_rotation'))
                    [~,obj_rot] = obj.vrep.simxGetObjectQuaternion(id, ...
                        handle1,...
                        -1,...
                        obj.OP_STREAMING);
                    % We need to check the buffer until it is not empty,
                    % TODO add a timeout.
                    retval = 1;
                    while retval==1
                        [retval,obj_rot] = obj.vrep.simxGetObjectQuaternion(id,...
                            handle1,...
                            -1,...
                            obj.OP_BUFFER);
                    end
                else
                    [~,obj_rot] = obj.vrep.simxGetObjectQuaternion(id,handle1,...
                        -1,...
                        obj.OP_BUFFER);
                end
            else
                handle2 = obj.handle_from_string_or_handle(relative_to_handle);
                [~,obj_rot] = obj.vrep.simxGetObjectQuaternion(id,...
                    handle1,...
                    handle2,...
                    opmode);
            end
            object_rotation_double = double(obj_rot);
            
            %V-Rep's quaternion representation is [x y z w] so we have to
            %take that into account
            r = normalize(DQ([object_rotation_double(4),...
                object_rotation_double(1),...
                object_rotation_double(2),...
                object_rotation_double(3)]));
        end
        
        %% Set Object Rotation
        function set_object_rotation(obj,handle,r,relative_to_handle,opmode)
            %% Set the rotation of an object in V-REP
            %%  >> r = DQ.i;
            %%  >> vi.set_object_rotation('DefaultCamera',r);
            
            if nargin == 3
                obj.vrep.simxSetObjectQuaternion(...
                    obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    -1,...
                    [r.q(2:4); r.q(1)],...
                    obj.OP_ONESHOT); %V-Rep's quaternion representation is [x y z w] so we have to take that into account
            else
                obj.vrep.simxSetObjectQuaternion(...
                    obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    [r.q(2:4); r.q(1)],...
                    opmode); %V-Rep's quaternion representation is [x y z w] so we have to take that into account
            end
        end
        
        %% Get Object Pose
        function x = get_object_pose(obj,handle,relative_to_handle,opmode)
            %% Get the pose of an object in V-REP
            %%  >> x = vi.get_object_pose('DefaultCamera');
            
            if nargin <= 2
                t = obj.get_object_translation(handle);
                r = obj.get_object_rotation(handle);
            else
                t = obj.get_object_translation(...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    opmode);
                r = obj.get_object_rotation(...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    opmode);
            end
            x = r + 0.5*DQ.E*t*r;
        end
        
        %% Set Object Pose
        function set_object_pose(obj,handle,x,relative_to_handle,opmode)
            %% Set the pose of an object in V-REP
            %%  >> t = DQ.i*0.01;
            %%  >> r = DQ.i;
            %%  >> x = r+0.5*DQ.E*t*r;
            
            if nargin == 3
                t = translation(x);
                r = rotation(x);
                obj.set_object_translation(...
                    obj.handle_from_string_or_handle(handle),...
                    t,...
                    -1,...
                    obj.OP_ONESHOT);
                obj.set_object_rotation(...
                    obj.handle_from_string_or_handle(handle),...
                    r,...
                    -1,...
                    obj.OP_ONESHOT);
            else
                t = translation(x);
                r = rotation(x);
                obj.set_object_translation(...
                    obj.handle_from_string_or_handle(handle),...
                    t,...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    opmode);
                obj.set_object_rotation(...
                    obj.handle_from_string_or_handle(handle),...
                    r,...    
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    opmode);
            end
        end
        
        %% Set Joint Positions
        function set_joint_positions(obj,handles,thetas,opmode)
            %% Set the joint positions of a robot in V-REP. For joints that are in 'Passive Mode' in V-REP
            %%  >> joint_names = {'redundantRob_joint1','redundantRob_joint2','redundantRob_joint3','redundantRob_joint4','redundantRob_joint5','redundantRob_joint6','redundantRob_joint7'};
            %%  >> vi.set_joint_positions(joint_names,[0 pi/2 0 pi/2 0 pi/2 0]);
            
            if nargin == 3
                for joint_index=1:length(handles)
                    obj.vrep.simxSetJointPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles{joint_index}),...
                        thetas(joint_index),...
                        obj.OP_ONESHOT);
                end
            else
                for joint_index=1:length(handles)
                    obj.vrep.simxSetJointPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles{joint_index}),...
                        thetas(joint_index),...
                        opmode);
                end
            end
        end
        
        %% Set Joint Target Positions
        function set_joint_target_positions(obj,handles,thetas,opmode)
            %% Set the joint target positions of a robot in V-REP. For joints that are in 'Force/Torque Mode' in V-REP
            %%  >> joint_names = {'redundantRob_joint1','redundantRob_joint2','redundantRob_joint3','redundantRob_joint4','redundantRob_joint5','redundantRob_joint6','redundantRob_joint7'};
            %%  >> vi.set_joint_target_positions(joint_names,[0 pi/2 0 pi/2 0 pi/2 0]);
            
            if nargin == 3
                for joint_index=1:length(handles)
                    if isa(handles,'cell')
                        obj.vrep.simxSetJointTargetPosition(...
                            obj.clientID,...
                            obj.handle_from_string_or_handle(handles{joint_index}),...
                            thetas(joint_index),...
                            obj.OP_ONESHOT);
                    else
                        obj.vrep.simxSetJointTargetPosition(...
                            obj.clientID,...
                            obj.handle_from_string_or_handle(handles),...
                            thetas(joint_index),...
                            obj.OP_ONESHOT);
                    end
                    
                end
            else
                for joint_index=1:length(handles)
                    obj.vrep.simxSetJointTargetPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles{joint_index}),...
                        thetas(joint_index),...
                        opmode);
                end
            end
        end
        
        %% Get Joint Positions
        function [thetas,retval]=get_joint_positions(obj,handles,opmode)
            %% Get joint positions
            %%  >> joint_names = {'redundantRob_joint1','redundantRob_joint2','redundantRob_joint3','redundantRob_joint4','redundantRob_joint5','redundantRob_joint6','redundantRob_joint7'};
            %%  >> vi.get_joint_positions(joint_names)
            
            thetas = zeros(length(handles),1);
            for joint_index=1:length(handles)
                % First approach to the auto-management using
                % DQ_VrepInterfaceMapElements. If the user does not specify the
                % opmode, it is chosen first as STREAMING and then as BUFFER,
                % as specified by the remote API documentation
                if nargin <= 2
                    if isa(handles,'cell')
                        element = obj.element_from_string(handles{joint_index});
                    else
                        element = obj.element_from_string(handles);
                    end
                    if(~element.state_from_function_signature('get_joint_positions'))
                        [~,tmp] = obj.vrep.simxGetJointPosition(...
                            obj.clientID,...
                            element.handle,...
                            obj.OP_STREAMING);
                        retval=1;
                        while retval==1
                            [retval,tmp] = obj.vrep.simxGetJointPosition(...
                                obj.clientID,...
                                element.handle,...
                                obj.OP_BUFFER);
                        end
                    else
                        [retval,tmp] = obj.vrep.simxGetJointPosition(...
                            obj.clientID,...
                            element.handle,...
                            obj.OP_BUFFER);
                    end
                else
                    [retval,tmp] = obj.vrep.simxGetJointPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles{joint_index}),...
                        opmode);
                end
                thetas(joint_index) = double(tmp);
            end
        end
        
    end
    
end



