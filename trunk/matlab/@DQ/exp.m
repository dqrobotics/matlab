% The function exp(dq) returns the exponential of the dual quaternion dq
function res=exp(dq)
dq=DQ(dq);

theta=norm(dq.P.q);



if(theta ~= 0)
    prim = DQ(cos(theta)) + (sin(theta)/theta)*dq.P;
else
    prim = DQ(0);
end

res=prim+DQ.E*dq.D*prim;