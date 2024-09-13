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
%        - Renamed the class from DQ_VrepInterface to DQ_CoppeliaSimLegacyInterface
%           - The class now inherits from DQ_CoppeliaSimLegacyInterface
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


classdef DQ_VrepInterface < DQ_CoppeliaSimLegacyInterface
    methods
        function obj = DQ_VrepInterface()
            warning('Deprecated. Use DQ_CoppeliaSimLegacyInterface instead.')
            obj@DQ_CoppeliaSimLegacyInterface();
        end
    end
end



