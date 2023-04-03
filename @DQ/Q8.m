% Q8(x) Given the unit dual quaternion x, Q8(x) returns the partial derivative of vec8(x) 
% with respect to vec6(log(x)).
%       See theorem 4 of Savino et al (2020). 
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
%
% 1. Bruno Vilhena Adorno (adorno@ieee.org)
%        - Responsible for the original implementation.
%
% 2. Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
%        - Removed Q4() as requested in #90.
%          https://github.com/dqrobotics/matlab/issues/90


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
