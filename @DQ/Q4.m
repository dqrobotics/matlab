% Q4(x) Given the unit quaternion r, return the partial derivative of vec4(r) 
% with respect to vec3(log(r)).
%       See Eq. (22) of Savino et al (2020). 
%       Pose consensus based on dual quaternion algebra with application to 
%       decentralized formation control of mobile manipulators.
%       https://doi.org/10.1016/j.jfranklin.2019.09.045

% (C) Copyright 2011-2023 DQ Robotics Developers
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
% 1. Bruno Vilhena Adorno (adorno@ieee.org)
%        - Responsible for the original implementation.
%
% 2. Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
%      Implemented the modifications requested in #90
%          (https://github.com/dqrobotics/matlab/issues/90)
%        - Removed Q4() from Q8.m, exposed the method to the user, and
%        - added the check for unit norm.        

function ret = Q4(x)
% Return the partial derivative of the unit quaternion r with respect to
% log(r).
    if ~is_unit(x) || ~is_quaternion(x)
        error('Q4 function is defined only for unit quaternions');
    end

    r = vec4(x);
    phi = double(rotation_angle(x));
    n = vec3(rotation_axis(x));
    nx = n(1); ny = n(2); nz = n(3);

    if phi == 0
        theta = 1;
    else
        theta = sin(phi/2)/(phi/2);
    end
 
    gamma = r(1) - theta; 

    ret = [           -r(2),            -r(3),            -r(4);
           gamma*nx^2+theta,      gamma*nx*ny,      gamma*nx*nz; 
                gamma*nx*ny, gamma*ny^2+theta,      gamma*ny*nz;
                gamma*nz*nx,      gamma*nz*ny, gamma*nz^2+theta];
end
