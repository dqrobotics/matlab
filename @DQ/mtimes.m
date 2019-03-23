% a * b returns the standard dual quaternion multiplication between a
% and b
% Scalars can be used as well; for example: 2 * a.

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

function res = mtimes(a,b)
    if ~isa(a,'DQ');
        a = DQ(a);
    end
    if ~isa(b,'DQ');
        b = DQ(b);
    end
    primary = QuaternionMultiplication(a.q(1:4),b.q(1:4));
    dual = QuaternionMultiplication(a.q(1:4),b.q(5:8)) + ...
        QuaternionMultiplication(a.q(5:8),b.q(1:4));
    res = DQ([primary; dual]);
end

function r = QuaternionMultiplication(p,q)
    P=[p(1,1),-p(2,1),-p(3,1),-p(4,1);
        p(2,1),p(1,1),-p(4,1),p(3,1);
        p(3,1),p(4,1),p(1,1),-p(2,1);
        p(4,1),-p(3,1),p(2,1),p(1,1)];

    r = P*q;
end