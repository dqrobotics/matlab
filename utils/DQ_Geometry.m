
% (C) Copyright 2011-2019 DQ Robotics Developers
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

classdef DQ_Geometry
    
    methods (Static)
        
        function ret = point_to_point_squared_distance(point1, point2)
            % POINT_TO_POINT_SQUARED_DISTANCE(point1, point2) returns the
            % square of the Euclidean distance between point1 and point2, where
            % point1 and point2 are pure quaternions
            
            if ~is_pure_quaternion(point1)
                error('Input point1 is not a pure quaternion.');
            elseif ~is_pure_quaternion(point2)
                error('Input point2 is not a pure quaternion.');
            end
            
            point = vec3(point1-point2);
            
            ret = point'*point;
        end
        
        
        function ret = point_to_line_squared_distance(point, line)
            % POINT_TO_LINE_SQUARED_DISTANCE(point, line) returns the square of the
            % Euclidean distance between 'point' and 'line', where point is a pure
            % quaternion and 'line' is a unit-norm pure dual quaternion.
            % See ?T. Brox, B. Rosenhahn, J. Gall, and D. Cremers, ?Combined region and
            % motion-based 3d tracking of rigid and articulated objects,? IEEE Transactions
            % on Pattern Analysis and Machine Intelligence, vol. 32, no. 3, pp. 402?415,
            % 2010.
            
            if ~is_pure_quaternion(point)
                error('Input point is not a pure quaternion.');
            elseif ~is_line(line)
                error('Input line is not a line.');
            end
            
            l = P(line);
            m = D(line);
            
            new_point = vec4(cross(point,l)) - m;
            ret = vec4(new_point)'*vec4(new_point);
            
        end
        
        function ret = point_to_plane_distance(point, plane)
            % POINT_TO_PLANE_DISTANCE(point, plane) returns the Euclidean distance
            % between a given point and a given plane, where 'point' is a pure
            % quaternion and 'plane' is a unit norm dual quaternion with pure primary
            % part and real dual part.
            %
            % ?M. M. Marinho, B. V. Adorno, K. Harada, and M. Mitsuishi, ?Active
            % Constraints Using Vector Field Inequalities for Surgical Robots,? in
            % 2018 IEEE International Conference on Robotics and Automation (ICRA), 2018,
            % pp. 5364?5371.
            if ~is_pure_quaternion(point)
                error('Input point is not a pure quaternion.');
            elseif ~is_plane(plane)
                error('Input plane is not a plane.');
            end
            
            plane_n = P(plane);
            plane_d = D(plane);
            
            ret = double(dot(point,plane_n) - plane_d);
        end
        
        
        function ret = line_to_line_squared_distance(line1, line2)
            % LINE_TO_LINE_SQUARED_DISTANCE(line1,line2) returns the square of the
            % Euclidean distance between two lines, where 'line1' and 'line2' are
            % unit-norm pure dual quaternions.
            if ~is_line(line1)
                error('Input line1 is not a line.');
            elseif ~is_line(line2)
                error('Input line2 is not a line.');
            end
            
            l1_cross_l2 = cross(line1, line2);
            l1_dot_l2   = dot(line1, line2);
            
            % If the lines are not parallel
            if P(l1_cross_l2) ~= 0
                norm_p_cross_product = norm(P(l1_cross_l2));
                norm_d_dot_product = norm(D(l1_dot_l2));
                ret = double(norm_d_dot_product/norm_p_cross_product)^2;
            else
                dual_l1_cross_l2 = D(l1_cross_l2);
                ret = vec4(dual_l1_cross_l2)'*vec4(dual_l1_cross_l2);
            end                
        end
        
        function ret = line_to_line_angle(line1, line2)
            % LINE_TO_LINE_ANGLE(line1,line2) returns the angle between two lines,
            % where 'line1' and 'line2' are unit-norm pure dual quaternions.
            
            if ~is_line(line1)
                error('Input line1 is not a line.');
            elseif ~is_line(line2)
                error('Input line2 is not a line.');
            end            
            
            l1_dot_l2   = dot(line1, line2);
            % Retrieve the angle between the lines---Eq. 35 of Marinho et
            % al. (2019)
            ret = acos(double(P(l1_dot_l2)));    
        end
    end
end