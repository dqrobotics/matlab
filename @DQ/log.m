% The function log(dq) returns the logarithm of the dual quaternion dq
function ret = log(dq)
    dq = DQ(dq);
    
    primary = (theta(dq)/2)*dq.rotation_axis;
    dual = translation(dq)*0.5;
    
    ret = primary + DQ.E* dual;
    
   