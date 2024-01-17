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
%       set_synchronous - Set the stepped (synchronous) mode for the remote
%       API server service that the client is connected to. 
%       trigger_next_simulation_step - Send a synchronization trigger 
%       signal to the CoppeliaSim scene
%       wait_for_simulation_step_to_end - Return the time needed for a 
%       command to be sent to the CoppeliaSim scene, executed, and sent back.
%       set_joint_target_velocities -  Set the joint target velocities of a
%       robot
%       get_joint_velocities - Get the joint velocities of a robot
%       get_joint_torques - Get the joint torques of a robot
%       set_joint_torques - Set the joint torques of a robot
%
%   DQ_VrepInterface Methods (For advanced users)
%       get_handle - Get the handle of a V-REP object
%       get_handles - Get the handles for multiple V-REP objects
%

% (C) Copyright 2018-2023 DQ Robotics Developers
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
%     1. Murilo Marques Marinho (murilo@nml.t.u-tokyo.ac.jp)
%        - Responsible for the original implementation.
%
%     2. Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
%        - Added the following methods:
%             - set_synchronous()
%             - trigger_next_simulation_step()
%             - wait_for_simulation_step_to_end()
%             - set_joint_target_velocities()
%             - get_joint_velocities()
%        - Improved the documentation of the class
%
%     3. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%        - Added the following methods:
%             - get_joint_torques() (see https://github.com/dqrobotics/matlab/pull/104)
%             - set_joint_torques() (see https://github.com/dqrobotics/matlab/pull/104)
%       - Altered the following properties from 'private' to 'protected'
%       (see discussions in https://github.com/dqrobotics/matlab/pull/101
%       to further details):
%             - vrep
%             - clientID


classdef DQ_VrepInterface < handle
    
    properties (Access = private)
        % a map between V-REP object names and DQ_VrepInterfaceMapElements
        handles_map;
    end

    properties (Access = protected)
        % the V-REP remote API instance used by this interface
        vrep;
        % the client ID of this remote API connection
        clientID;
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
        % Constant that denotes the V-VREP's remote API joint velocity ID
        JOINT_VELOCITY_PARAMETER_ID = remApi('remoteApi').sim_jointfloatparam_velocity;
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
            % This method connects to the remote api server (i.e. CoppeliaSim).
            % Calling this function is required before anything else can happen.
            % Usage:
            %     connect(ip, port);  
            %          ip:  The ip address where the CoppeliaSim is located.
            %          port: The port number where to connect.
            %
            % Example:
            %      connect('127.0.0.1', 19997);

            obj.clientID = obj.vrep.simxStart(ip,port,true,true,5000,5);
            if (obj.clientID>-1)
                disp('Connected to the remote API server');
            else
                error('Failed to connect to remote API server');
            end
        end
      
        function disconnect(obj)
            % This method ends the communication between the client and
            % the CoppeliaSim scene. This should be the very last method called.
            obj.vrep.simxFinish(obj.clientID);
        end
        
        function disconnect_all(obj)
            % This method ends all running communication threads with the 
            % CoppeliaSim scene. This should be the very last method called.
            obj.vrep.simxFinish(-1);
        end

        function set_synchronous(obj,flag)
            % This method enables or disables the stepped (synchronous) mode
            % for the remote API server service that the client is connected to.
            % Example:
            %       set_synchronous(true)    % synchronous mode enabled
            %       set_synchronous(false)   % synchronous mode disabled
            if (~islogical(flag))
                error('Error in set_synchronous: argument must be logical, not a %s. \nExample:\n set_synchronous(true)', class(flag));
            end
            obj.vrep.simxSynchronous(obj.clientID,flag);
        end

        function trigger_next_simulation_step(obj)
            % This method sends a synchronization trigger signal to the CoppeliaSim scene, 
            % which performs a simulation step when the synchronous mode is used.
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end

        function ping_time = wait_for_simulation_step_to_end(obj)
            % This method returns the time needed for a command to be sent 
            % to the CoppeliaSim scene, executed, and sent back.
            [~, ping_time] =  obj.vrep.simxGetPingTime(obj.clientID);
        end
        
        function start_simulation(obj)
            % This method starts the CoppeliaSim simulation.
            obj.vrep.simxStartSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot);
        end
        
        function stop_simulation(obj)
            % This method stops the CoppeliaSim simulation.
            obj.vrep.simxStopSimulation(obj.clientID,obj.vrep.simx_opmode_blocking);
        end
        
        function handles = get_handles(obj,names)
            % This method gets the handles for a cell array of 
            % object names in the the CoppeliaSim scene.
            % Usage:
            %     get_handles(names);  
            %          names: The cell array of object names.
            %
            % Example: 
            %     handle = get_handles({'ReferenceFrame_1', 'ReferenceFrame_2'});

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
        
        function handle = get_handle(obj,name)
            % This method gets the handle for a given object in the CoppeliaSim scene. 
            % 
            % Usage:
            %     get_handles(name);  
            %          names: The object name.
            % Example: 
            %     handle = get_handle('ReferenceFrame');

            [~,handle] = obj.vrep.simxGetObjectHandle(...
                obj.clientID,...
                name,...
                obj.vrep.simx_opmode_blocking);
        end
        
        function t = get_object_translation(obj,objectname,reference_frame,opmode)
            % This method gets the translation of an object in the CoppeliaSim scene.
            %
            % Usage:
            %     Recommended:
            %     t = get_object_translation(objectname) 
            %
            %     Advanced:
            %     t = get_object_translation(objectname,reference_frame,opmode);  
            %
            %          objectname:  The object name
            %          (optional) reference_frame:  Indicates the name of 
            %                       the relative reference frame in which you 
            %                       want the translation. If not specified, 
            %                       the absolute frame is used.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      % Recommended:
            %      t = get_object_translation('DefaultCamera');  
            %
            %      % For advanced usage:
            %      t = get_object_translation('DefaultCamera', 'Frame_b', OP_ONESHOT); 
            
            % First approach to the auto-management using
            % DQ_VrepInterfaceMapElements. If the user does not specify the
            % opmode, it is chosen first as STREAMING and then as BUFFER,
            % as specified by the remote API documentation
            
            handle = objectname; % alias
            
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
                relative_to_handle = reference_frame; % alias
                [~,object_position]  = obj.vrep.simxGetObjectPosition(...
                    obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    opmode);
            end
            t = DQ([0,double(object_position)]);
        end
        
        function set_object_translation(obj,objectname,translation,reference_frame,opmode)
            % This method sets the translation of an object in the CoppeliaSim scene.
            % Usage:
            %     Recommended:
            %     set_object_translation(objectname,translation);
            %
            %     Advanced:
            %     set_object_translation(objectname,translation,reference_frame,opmode);  
            %
            %          objectname:  The object name.
            %          translation: The desired translation. 
            %          (optional) reference_frame:  Indicates the name of 
            %                       the relative reference frame in which 
            %                       the desired translation is expressed. 
            %                       If not specified, the absolute frame is used.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      % Recommended:
            %      set_object_translation('DefaultCamera', t);  
            %
            %      % For advanced usage:
            %      set_object_translation('DefaultCamera', t, 'Frame_b', OP_ONESHOT); 

            % create some aliases
            handle = objectname;
            t = translation;
  
            if nargin == 3
                obj.vrep.simxSetObjectPosition(obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    -1,...
                    t.q(2:4),...
                    obj.OP_ONESHOT);
            else
                relative_to_handle = reference_frame; % alias
                obj.vrep.simxSetObjectPosition(obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    t.q(2:4),...
                    opmode);
            end
        end
        
        function r = get_object_rotation(obj, objectname, reference_frame, opmode)
            % This method gets the rotation of an object in the CoppeliaSim scene.
            %
            % Usage:
            %     Recommended:
            %     t = get_object_rotation(objectname);
            %
            %     Advanced:
            %     t = get_object_rotation(objectname,reference_frame,opmode);  
            %
            %          objectname: The object name
            %          (optional) reference_frame:  Indicates the name of 
            %                       the relative reference frame in which you 
            %                       want the rotation. If not specified, 
            %                       the absolute frame is used.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      % Recommended:
            %      r = get_object_rotation('DefaultCamera');  
            %
            %      % For advanced usage:
            %      r = get_object_rotation('DefaultCamera', 'Frame_b', OP_ONESHOT);             
       
            
            % create some aliases
            handle = objectname;
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
                relative_to_handle = reference_frame; % alias
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
        
        function set_object_rotation(obj,objectname,rotation,reference_frame,opmode)
            % This method sets the rotation of an object in the CoppeliaSim scene.
            % Usage:
            %     Recommended:
            %     set_object_rotation(objectname,rotation);
            %
            %     Advanced:
            %     set_object_rotation(objectname,rotation,reference_frame,opmode);  
            %
            %          objectname: The object name.
            %          rotation: The desired rotation. 
            %          (optional) reference_frame:  Indicates the name of 
            %                       the relative reference frame in which 
            %                       the desired rotation is expressed. 
            %                       If not specified, the absolute frame is used.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      % Recommended:
            %      set_object_rotation('DefaultCamera', r); 
            %
            %      % For advanced usage:
            %      set_object_rotation('DefaultCamera', r, 'Frame_b', OP_ONESHOT); 
 
            % create some aliases
            handle = objectname;
            r = rotation;
            
            if nargin == 3
                obj.vrep.simxSetObjectQuaternion(...
                    obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    -1,...
                    [r.q(2:4); r.q(1)],...
                    obj.OP_ONESHOT); %V-Rep's quaternion representation is [x y z w] so we have to take that into account
            else
                relative_to_handle = reference_frame; % alias
                obj.vrep.simxSetObjectQuaternion(...
                    obj.clientID,...
                    obj.handle_from_string_or_handle(handle),...
                    obj.handle_from_string_or_handle(relative_to_handle),...
                    [r.q(2:4); r.q(1)],...
                    opmode); %V-Rep's quaternion representation is [x y z w] so we have to take that into account
            end
        end
        
        function x = get_object_pose(obj,objectname,reference_frame,opmode)
            % This method gets the pose of an object in the CoppeliaSim scene.
            %
            % Usage:
            %     Recommended:
            %      x = get_object_pose(objectname);
            %
            %     Advanced:
            %      x = get_object_pose(objectname,reference_frame,opmode);  
            %
            %          objectname: The object name
            %          (optional) reference_frame:  Indicates the name of 
            %                       the relative reference frame in which you 
            %                       want the pose. If not specified, 
            %                       the absolute frame is used.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      % Recommended:
            %      x = get_object_pose('DefaultCamera'); 
            %
            %      % For advanced usage:
            %      x = get_object_pose('DefaultCamera', 'Frame_b', OP_ONESHOT);              


            handle = objectname; % alias
            

            if nargin <= 2
                t = obj.get_object_translation(handle);
                r = obj.get_object_rotation(handle);
            else
                relative_to_handle = reference_frame; % alias
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
        
        function set_object_pose(obj,objectname,pose,reference_frame,opmode)
            % This method sets the pose of an object in the CoppeliaSim scene.
            % Usage:
            %     Recommended:
            %      set_object_pose(objectname,pose);
            %     
            %     Advanced:
            %     set_object_pose(objectname,pose,reference_frame,opmode);  
            %
            %          objectname: The object name.
            %          pose: The desired pose. 
            %          (optional) reference_frame:  Indicates the name of 
            %                       the relative reference frame in which 
            %                       the desired pose is expressed. 
            %                       If not specified, the absolute frame is used.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      
            %      t = DQ.i*0.01;
            %      r = DQ.i;
            %      x = r+0.5*DQ.E*t*r;
            %
            %      % Recommended:
            %      set_object_pose('DefaultCamera', x);  
            %
            %      % For advanced usage:
            %      set_object_pose('DefaultCamera', x, 'Frame_b', OP_ONESHOT);  
 
            % create some aliases
            handle = objectname;
            x = pose;            

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
                relative_to_handle = reference_frame;
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
        
        function set_joint_positions(obj,jointnames,joint_positions,opmode)
            % This method sets the joint positions of a robot in the CoppeliaSim scene.
            % It is required a dynamics disabled scene. 
            %
            % Usage:
            %      Recommended:
            %      set_joint_positions(jointnames, joint_positions);
            %
            %      Advanced:
            %      set_joint_positions(jointnames, joint_positions, opmode);
            %
            %          jointnames: The joint names.
            %          joint_positions: The joint positions.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       % Recommended:
            %       set_joint_positions(jointnames, u);
            %
            %       % Advanced usage:
            %       set_joint_positions(jointnames, u, OP_ONESHOT);
            
            % create some aliases
            handles = jointnames;
            thetas = joint_positions;

            if nargin == 3
                % The recommended mode is OP_ONESHOT
                opmode = obj.OP_ONESHOT;
            end
            
            for joint_index=1:length(handles)
                if isa(handles,'cell')
                    obj.vrep.simxSetJointPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles{joint_index}),...
                        thetas(joint_index),...
                        opmode);
                else
                    obj.vrep.simxSetJointPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles),...
                        thetas(joint_index),...
                        opmode);
                end
            end
        end
            
        function set_joint_target_positions(obj,jointnames,joint_target_positions,opmode)
            % This method sets the joint target positions of a robot in the CoppeliaSim scene. 
            % It is required a dynamics enabled scene, and joints in dynamic mode 
            % with position control mode.
            %
            % Usage:
            %      Recommended:
            %      set_joint_target_positions(jointnames,joint_target_positions);
            %      
            %      Advanced:
            %      set_joint_target_positions(jointnames, joint_target_positions, opmode);   
            %
            %          jointnames: The joint names.
            %          joint_target_positions: The joint target positions.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %       jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       % Recommended:
            %       set_joint_target_positions(jointnames, u);     
            %
            %       % Advanced usage:
            %       set_joint_target_positions(jointnames, u, OP_ONESHOT);
            
            % create some aliases
            handles = jointnames;
            thetas = joint_target_positions;

            if nargin == 3
                % The recommended mode is OP_ONESHOT
                opmode = obj.OP_ONESHOT;
            end            
            
            for joint_index=1:length(handles)
                if isa(handles,'cell')
                    obj.vrep.simxSetJointTargetPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles{joint_index}),...
                        thetas(joint_index),...
                        opmode);
                else
                    obj.vrep.simxSetJointTargetPosition(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(handles),...
                        thetas(joint_index),...
                        opmode);
                end                
            end            
        end
        
        function [joint_positions,retval]=get_joint_positions(obj,jointnames,opmode)
            % This method gets the joint positions of a robot in the CoppeliaSim scene.
            % Usage:
            %      Recommended:
            %      joint_positions = get_joint_positions(jointnames);
            %      [joint_positions, retval] = get_joint_positions(jointnames);
            %
            %      Advanced:
            %      joint_positions] = get_joint_positions(jointnames,opmode);
            %      [joint_positions, retval] = get_joint_positions(jointnames, opmode);  
            %
            %          -Parameters:
            %
            %            jointnames: The joint names.
            %            (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            %          - Outputs:
            %
            %            joint_positions: The joints positions
            %            retval: The return code of the Remote API function, 
            %                   which is defined in https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
            %       
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %      
            %      % Recommended:
            %      joint_positions = get_joint_positions(jointnames);
            %      [joint_positions, rtn] = get_joint_positions(jointnames);
            %      
            %     % Advanced usage:
            %      joint_positions = get_joint_positions(jointnames, OP_ONESHOT);
            %      [joint_positions, rtn] = get_joint_positions(jointnames, OP_ONESHOT);


            % create some aliases
            handles = jointnames;    
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
            joint_positions = thetas;
        end

        function joint_velocities = get_joint_velocities(obj,jointnames,opmode)
            % This method gets the joint velocities of a robot in the CoppeliaSim scene.
            % Usage:
            %      Recommended:
            %      joint_velocities = get_joint_velocities(jointnames);
            %
            %      Advanced:
            %      joint_velocities = get_joint_velocities(jointnames, opmode)   
            %
            %          jointnames: The joint names.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      % Recommended:
            %      joint_velocities = get_joint_velocities(jointnames);
            %
            %      % Advanced usage:
            %      joint_velocities = get_joint_velocities(jointnames, OP_ONESHOT);
        
            joint_velocities = zeros(length(jointnames),1);
            for joint_index=1:length(jointnames)
                % First approach to the auto-management using
                % DQ_VrepInterfaceMapElements. If the user does not specify the
                % opmode, it is chosen first as STREAMING and then as BUFFER,
                % as specified by the remote API documentation
                if nargin <= 2
                    if isa(jointnames,'cell')
                        element = obj.element_from_string(jointnames{joint_index});
                    else
                        element = obj.element_from_string(jointnames);
                    end
                    if(~element.state_from_function_signature('get_joint_velocities'))
                        [~,tmp] =  obj.vrep.simxGetObjectFloatParameter(...
                            obj.clientID,...
                            element.handle,...
                            obj.JOINT_VELOCITY_PARAMETER_ID,...
                            obj.OP_STREAMING);
                        retval=1;
                        while retval==1
                            [retval,tmp] = obj.vrep.simxGetObjectFloatParameter(...
                                obj.clientID,...
                                element.handle,...
                                obj.JOINT_VELOCITY_PARAMETER_ID,...
                                obj.OP_BUFFER);
                        end
                    else
                        [~,tmp] = obj.vrep.simxGetObjectFloatParameter(...
                            obj.clientID,...
                            element.handle,...
                            obj.JOINT_VELOCITY_PARAMETER_ID,...
                            obj.OP_BUFFER);
                    end
                else
                    [~,tmp] = obj.vrep.simxGetObjectFloatParameter(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(jointnames{joint_index}),...
                        obj.JOINT_VELOCITY_PARAMETER_ID,...
                        opmode);
                end
                joint_velocities(joint_index) = double(tmp); 
            end
        end 

        function set_joint_target_velocities(obj,jointnames,joint_target_velocities,opmode)
            % This method sets the joint velocities of a robot in the CoppeliaSim scene.
            % It is required a dynamics enabled scene, and joints in dynamic mode 
            % with velocity control mode. Check this link for more
            % information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            %
            % Usage:
            %      Recommended:
            %      set_joint_target_velocities(jointnames, joint_target_velocities);
            %
            %      Advanced:
            %      set_joint_target_velocities(jointnames, joint_target_velocities, opmode);   
            %
            %          jointnames: The joint names.
            %          joint_target_velocities: The joint target velocities.
            %          (optional) opmode: The operation mode. If not specified, 
            %                       the opmode will be set automatically. 
            %                          
            %                        You can use the following modes:
            %                           OP_BLOCKING 
            %                           OP_STREAMING 
            %                           OP_ONESHOT 
            %                           OP_BUFFER;
            %
            %                       Check this link for more details:
            %                       https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %      u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %      % Recommended:
            %      set_joint_target_velocities(jointnames, u);
            %
            %      % Advanced usage:
            %      set_joint_target_velocities(jointnames, u, OP_ONESHOT);
            
            if nargin == 3
                % The recommended mode is OP_ONESHOT
                opmode = obj.OP_ONESHOT;
            end            
            
            for joint_index=1:length(jointnames)
                if isa(jointnames,'cell')
                    obj.vrep.simxSetJointTargetVelocity(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(jointnames{joint_index}),...
                        joint_target_velocities(joint_index),...
                        opmode);
                else
                    obj.vrep.simxSetJointTargetVelocity(...
                        obj.clientID,...
                        obj.handle_from_string_or_handle(jointnames),...
                        joint_target_velocities(joint_index),...
                        opmode);
                end                
            end            
        end

        function joint_torques = get_joint_torques(obj,handles,opmode)
            % This method gets the joint torques of a robot in the CoppeliaSim scene.
            % Usage:
            %      Recommended:
            %      joint_torques = get_joint_torques(jointnames);
            %
            %      Advanced:
            %      joint_torques = get_joint_torques(jointnames, opmode)
            %
            %          jointnames: The joint names.
            %          (optional) opmode: The operation mode. If not
            %            specified, the opmode will be set automatically. 
            %                          
            %            You can use the following modes:
            %               OP_BLOCKING 
            %               OP_STREAMING 
            %               OP_ONESHOT 
            %               OP_BUFFER;
            %
            %      Check this link for more details: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxGetJointForce
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      % Recommended:
            %      joint_torques = get_joint_torques(jointnames);
            %
            %      % Advanced usage:
            %      joint_torques = get_joint_torques(jointnames, OP_STREAMING);
            
            joint_torques = zeros(length(handles),1);

            % If the user does not specify the opmode, it is chosen first
            % as STREAMING and then as BUFFER, as specified by the remote
            % API documentation.
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
                    if(~element.state_from_function_signature('get_joint_torques'))
                        [~,tmp] = obj.vrep.simxGetJointForce(obj.clientID, element.handle, obj.OP_STREAMING);
                        return_code = 1;
                        while(return_code == 1)
                            [return_code,tmp] = obj.vrep.simxGetJointForce(obj.clientID, element.handle, obj.OP_BUFFER);
                        end
                    else
                        [~,tmp] = obj.vrep.simxGetJointForce(obj.clientID, element.handle, obj.OP_BUFFER);
                    end
                else
                    [~,tmp] = obj.vrep.simxGetJointForce(obj.clientID, ...
                        obj.handle_from_string_or_handle(handles{joint_index}), opmode);
                end
                joint_torques(joint_index) = double(-tmp); % V-REP returns joint torques with an inverse sign
            end
        end
        
        function set_joint_torques(obj,joint_names,torques,opmode)
            % This method sets the joint torques of a robot in the CoppeliaSim scene.
            % Usage:
            %      Recommended:
            %      set_joint_torques(jointnames, torques);
            %
            %      Advanced:
            %      set_joint_torques(jointnames, torques, opmode);
            %
            %          jointnames: The joint names.
            %          torques: The joint torques.
            %          (optional) opmode: The operation mode. If not
            %            specified, the opmode will be set automatically. 
            %                          
            %            You can use the following modes:
            %               OP_BLOCKING 
            %               OP_STREAMING 
            %               OP_ONESHOT 
            %               OP_BUFFER;
            %
            %      Check this link for more details: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxSetJointTargetVelocity
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %      u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %      % Recommended:
            %      set_joint_torques(jointnames, u);
            %
            %      % Advanced usage:
            %      set_joint_torques(jointnames, u, opmode);
            
            % If the user does not specify the opmode, it is chosen first
            % as STREAMING and then as OP_ONESHOT, as specified by the
            % remote API documentation.
            if nargin == 3
                % The recommended mode is OP_ONESHOT
                for joint_index=1:length(joint_names)
                    obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.handle_from_string_or_handle(joint_names{joint_index}), ...
                        sign(torques(joint_index))*10e10, obj.OP_ONESHOT);
                    obj.vrep.simxSetJointForce(obj.clientID, obj.handle_from_string_or_handle(joint_names{joint_index}), ...
                        abs(torques(joint_index)), obj.OP_ONESHOT);
                end
            else
                for joint_index=1:length(joint_names)
                    obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.handle_from_string_or_handle(joint_names{joint_index}),...
                        sign(torques(joint_index))*10e10, opmode);
                    obj.vrep.simxSetJointForce(obj.clientID,obj.handle_from_string_or_handle(joint_names{joint_index}),abs(torques(joint_index)),...
                        opmode);
                end
            end
        end
        
    end
    
end



