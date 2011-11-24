%p = translation(dq) returns the translation of the unit dual quaternion dq,
%assuming  dq=r + DQ.E * p * r * (0.5), that is, the translation followed
%by rotation movement.
function ret = translation(dq)
    if norm(dq) ~= 1
        error('Not a unit dual quaternion')
    end
    
    ret =  2*dq.D*dq.P';
end