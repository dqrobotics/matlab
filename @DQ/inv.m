% inv(h) returns the inverse of the dual quaternion h, which is given by
% h'/(norm(h)^2)
% See also dinv

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
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

function ret = inv(h)

    % Calculates norm(h)^2 = a + DQ.E*b, where a is a positive real number
    % and b is a real number
    x = h*h';
    primary = x.P(1);
    dual = x.D(1);
    % Recall that the inverse of a dual number is found by using Taylor
    % expansion. See Fact 2.2.1 of ?Adorno, B. V. (2017). Robot Kinematic 
    % Modeling and Control Based on Dual Quaternion Algebra -- Part I:
    % Fundamentals. 
    % https://hal.archives-ouvertes.fr/hal-01478225v1
    ret = h'*(1/primary - DQ.E*(dual/(primary^2)));
end