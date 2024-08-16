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
    %DQ_COPPELIASIM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods (Access = protected) 
        function obj = DQ_CoppeliaSim(inputArg1,inputArg2)
            %DQ_COPPELIASIM Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
    end
        
    methods(Abstract)
        % This method connects to CoppeliaSim.
        % Calling this function is required before anything else can happen.
        connect(obj, host, port);

        % This method enables or disables the stepped (synchronous) mode
        % for the remote API server service that the client is connected to.
        % Example:
        %       set_stepping_mode(true)    % stepping mode enabled
        %       set_stepping_mode(false)   % stepping mode disabled
        set_stepping_mode(true);

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

        get_object_translation(obj,objectname,reference_frame);
            





       

    end
end


