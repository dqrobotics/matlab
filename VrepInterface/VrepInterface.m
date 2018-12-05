% (C) Copyright 2018 DQ Robotics Developers
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

classdef VrepInterface < handle
    % VREP 3.5.0 remote API and DQ Robotics interface
    %   Connects to VREP remote API and retrieves/sends data compatible
    %   with DQ Robotics.
    % VREP remote API's performance is quite dependent on the correct usage of the
    % opmodes. Refer to http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
    
    properties
        vrep;
        clientID;
        handles_map;
    end
    properties (Constant)
        OP_BLOCKING  = remApi('remoteApi').simx_opmode_blocking;
        OP_STREAMING = remApi('remoteApi').simx_opmode_streaming;
        OP_ONESHOT   = remApi('remoteApi').simx_opmode_oneshot;
        OP_BUFFER    = remApi('remoteApi').simx_opmode_buffer;
    end
    
    methods
        %% Internal functions
        function handle = handle_from_string_or_handle(obj,name_or_handle)
            if(ischar(name_or_handle))
                name = name_or_handle;
                if(obj.handles_map.isKey(name))
                    handle = obj.handles_map(name);
                else
                    handle = obj.getHandle(name);
                    obj.handles_map(name) = handle;
                end
                
            else
                handle=name_or_handle;
            end
        end
        
        %% Constructor
        function obj = VrepInterface()
            obj.vrep=remApi('remoteApi');
            obj.handles_map = containers.Map;
            obj.clientID = -1;
            disp('Note[2]: This version of DQ Robotics VrepInterface is compatible with VREP 3.5.0')
        end
        
        function connect(obj,ip,port)
            obj.clientID = obj.vrep.simxStart(ip,port,true,true,5000,5);
            if (obj.clientID>-1)
                disp('Connected to the remote API server');
            else
                error('Failed to connect to remote API server');
            end
        end
        
        %% Close
        function disconnect(obj)
            obj.vrep.simxFinish(obj.clientID);
        end
        
        %% Close all
        function disconnectAll(obj)
            obj.vrep.simxFinish(-1);
        end
        
        %% Start Simulation
        function startSimulation(obj)
            obj.vrep.simxStartSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot);
        end
        
        %% Stop Simulation
        function stopSimulation(obj)
            obj.vrep.simxStopSimulation(obj.clientID,obj.vrep.simx_opmode_blocking);
        end
        
        %% Get Handles
        function handles = getHandles(obj,names)
            handles = [];
            if(iscell(names))
                for i=1:length(names)
                    [~,handles(i)] = obj.vrep.simxGetObjectHandle(obj.clientID,char(names(i)),obj.vrep.simx_opmode_blocking);
                end
            else
                error('Error in getHandles: argument names must be of type cell, e.g. names = [{joint1,joint2}];');
            end
        end
        
        %% Get Handle
        function handle = getHandle(obj,name)
            [~,handle] = obj.vrep.simxGetObjectHandle(obj.clientID,name,obj.vrep.simx_opmode_blocking);
        end
        
        %% Get Object Translation
        function t = getObjectTranslation(obj,handle,relative_to_handle,opmode)
            [~,object_position]  = obj.vrep.simxGetObjectPosition(obj.clientID,obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),opmode);
            t = DQ([0,double(object_position)]);
        end
        
        %% Set Object Translation
        function setObjectTranslation(obj,handle,relative_to_handle,t,opmode)
            obj.vrep.simxSetObjectPosition(obj.clientID,obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),t.q(2:4),opmode);
        end
        
        %% Get Object Rotation
        function r = getObjectRotation(obj,handle,relative_to_handle,opmode)
            [~,object_rotation] = obj.vrep.simxGetObjectQuaternion(obj.clientID,obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),opmode);
            r = normalize(DQ(double(object_rotation)));
        end
        
        %% Set Object Rotation
        function setObjectRotation(obj,handle,relative_to_handle,r,opmode)
            obj.vrep.simxSetObjectOrientation(obj.clientID,obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),r.q(1:4),opmode);
        end
        
        %% Get Object Pose
        function x = getObjectPose(obj,handle,relative_to_handle,opmode)
            t = obj.getObjectTranslation(obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),opmode);
            r = obj.getObjectRotation(obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),opmode);
            x = r + 0.5*DQ.E*t*r;
        end
        
        %% Set Object Pose
        function setObjectPose(obj,handle,relative_to_handle,x,opmode)
            t = translation(x);
            r = rotation(x);
            obj.setObjectTranslation(obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),t,opmode);
            obj.setObjectRotation(obj.handle_from_string_or_handle(handle),obj.handle_from_string_or_handle(relative_to_handle),r,opmode);
        end
        
        %% Set Joint Positions
        function setJointPositions(obj,handles,thetas,opmode)
            for joint_index=1:length(handles)
                obj.vrep.simxSetJointPosition(obj.clientID,handles(joint_index),thetas(joint_index),opmode);
            end
        end
        
        %% Get Joint Positions
        function thetas=getJointPositions(obj,handles)
            thetas = zeros(length(handles),1);
            for joint_index=1:length(handles)
                [~,tmp] = obj.vrep.simxGetJointPosition(obj.clientID,handles(joint_index),obj.vrep.simx_opmode_blocking);
                thetas(joint_index) = double(tmp);
            end
        end
        
        %% Get image
        function img=getVisionSensorImageBlocking(obj,handle)
            [~,~,img]=obj.vrep.simxGetVisionSensorImage2(obj.clientID,obj.handle_from_string_or_handle(handle),0,obj.vrep.simx_opmode_blocking);
        end
    end
    
end



