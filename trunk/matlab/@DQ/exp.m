% The function exp(g) returns the exponential of the pure dual quaternion g
function res=exp(g)
g=DQ(g);

if Re(g) ~= 0
   error('The exponential operation is defined only for pure dual quaternions');
end

phi=norm(g.P.q);

if(phi ~= 0)
    prim = cos(phi) + (sin(phi)/phi)*g.P;
else
    prim = DQ(1);
end

res = prim+DQ.E*g.D*prim;

end
