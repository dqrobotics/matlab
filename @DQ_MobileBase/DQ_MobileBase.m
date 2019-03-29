% CLASS DQ_MobileBase
% Usage: robot_kine = DQ_MobileBase(robot_parameters)
% - 'robot_parameters' is a struct containing all parameters needed to
% obtain the mobile base kinematic model. It must contain at least the
% field 'robot.type' to correctly identify the robot type.
%
% METHODS:
%       raw_fkm
%       fkm
%       raw_pose_jacobian
%       pose_jacobian
%
% See also DQ_kinematics

% (C) Copyright 2015 DQ Robotics Developers
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
%     Bruno Vihena Adorno - adorno@ufmg.br

classdef DQ_MobileBase < handle
    % DQ_MobileBase inherits the HANDLE superclass to avoid unnecessary copies
    % when passing DQ_kinematics objects as arguments to methods.
    
    properties (Constant)
        robot_types = {'holonomic', 'differential_drive'};
    end
    
    properties
        this_robot_type;
    end
    
    methods
        function obj = DQ_MobileBase(robot_parameters)
            % The default robot is a holonomic mobile base
            if nargin == 0
                obj.this_robot_type = 'holonomic';
            elseif ~isstruct(robot_parameters) 
                error(['DQ_MobileBase needs a struct parameter. Type '...
                       '<DQ_MobileBase.robot_types> to see the robot types '...
                       'currently supported and the corresponding structure '...
                       'array.']);
            elseif ~isfield(robot_parameters, 'robot_type')
                error('The robot_type field is missing');
            else %The robot_parameters.robot_type field is available                 
                robot_type = lower(robot_parameters.robot_type);
                obj.this_robot_type = robot_type;
                
                switch robot_type
                    case 'holonomic'
                        disp('Holonomic base');
                    case 'differential_drive'
                        disp('Differential drive base');
                    otherwise 
                        error('The current robot types are "%s"',char(DQ_MobileBase.robot_types)'); 
                end
            end
        end
    end
    
    methods
        
    end
        
    methods(Static)
        % Return the different robot types currently supported by this
        % class
        function get_robot_types()
            robot_types = DQ_MobileBase.robot_types;
            for i = robot_types
                disp(robot_types{i});
            end
        end
        
        function pose = fkm(q)
            x = q(1);
            y = q(2);
            phi = q(3);
            
            include_namespace_dq
            
            real_part = cos(phi/2) + k_*sin(phi/2);
            dual_part = (1/2)*i_*(x*cos(phi/2) + y*sin(phi/2)) + ...
                        (1/2)*j_*(-x*sin(phi/2) + y*cos(phi/2));            
            pose = real_part + E_*dual_part;
        end
    end
end
