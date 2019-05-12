% Return the partial derivative of the unit dual quaternion x with respect to log(x)

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

function ret = Q8(x)

    if ~is_unit(x)
        error('Q8 function is defined only for unit dual quaternions');
    end

    r = rotation(x);
    p = translation(x);

    Q = Q4(r);
    Qp = [zeros(1,3);
          eye(3)];

    ret = [Q, zeros(4,3);
           0.5*hamiplus4(p)*Q, haminus4(r)*Qp];

end

function ret = Q4(rot)
% Return the partial derivative of the unit quaternion r with respect to
% log(r).

    r = vec4(rot);
    phi = double(rotation_angle(rot));
    n = vec3(rotation_axis(rot));
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