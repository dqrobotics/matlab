% The function log(dq) returns the logarithm of the dual quaternion dq
function ret = log(dq)
dq = DQ(dq);

primary = mod(dq.theta/2,pi)*dq.rotation_axis;
dual = translation(dq)*0.5;

ret = primary + DQ.E* dual;

end