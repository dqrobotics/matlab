%PINV(dq) returns the inverse of dq under the decompositional
%multiplication (i.e., the group operation of CMI(3))
function ret = pinv(dq)
    dq = DQ(dq);
    temp = tplus(dq)*tplus(dq');    
    ret = temp'*dq';
end