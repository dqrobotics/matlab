% (C) Copyright 2019 DQ Robotics Developers
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

classdef DQ_VrepInterfaceMapElement
    properties
        % For VREP's remote API, the first call to any "get" function should be OP_STREAMING and the following calls should be
        % OP_BUFFER, so we need to track the states of each of them using
        % the following map
        set_states_map
        handle
    end
    methods
        %% Constructor
        function obj = DQ_VrepInterfaceMapElement(handle)
            obj.set_states_map = containers.Map;
            obj.handle = handle;
        end
                
        function state = state_from_function_signature(obj,function_signature)
            if(obj.set_states_map.isKey(function_signature))
                obj.set_states_map(function_signature) = true;
            else
                obj.set_states_map(function_signature) = false;
            end
            state = obj.set_states_map(function_signature);
        end
    end
end
