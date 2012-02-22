% The function exp(dq) returns the exponential of the dual quaternion dq
function res=exp(dq)
dq=DQ(dq);

phi=norm(dq.P.q);

if(phi ~= 0)
    prim = cos(phi) + (sin(phi)/phi)*dq.P;
else
    prim = DQ(1);
end

if(prim.q(1) < 0)
   
    res = -1*(prim+DQ.E*dq.D*prim);
else
    res = prim+DQ.E*dq.D*prim;
end

end
