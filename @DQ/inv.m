% inv(dq) returns the inverse of the dual quaternion dq.
function ret = inv(dq)
    q = dq*dq'; %(norm(dq)^2)
    
    q_inv = DQ([1/q.q(1),0,0,0,-q.q(5)/(q.q(1)^2),0,0,0]);
    ret = dq'*q_inv;
end