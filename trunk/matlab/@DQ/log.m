function ret = log(dq)
    non_dual = mod(dq.theta/2,pi)*dq.rotation_axis;
    dual = dq.t/2;    
    
    ret = DQ([0;non_dual;0;dual(2:4)]);