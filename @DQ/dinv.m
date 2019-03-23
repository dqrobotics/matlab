% dinv(a) returns the inverse of a under the decompositional
% multiplication (i.e., the group operation of CMI(3))
%
% See Lemma 5 of ?Adorno, B. V. (2017). Robot Kinematic Modeling and Control 
% Based on Dual Quaternion Algebra -- Part I: Fundamentals. 
% https://hal.archives-ouvertes.fr/hal-01478225v1
%

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

function ret = dinv(x)
    if ~isa(x,'DQ')
        x = DQ(x);
    end
    
    ret = T(x)'*P(x)';
end