%T(x) returns the unit dual quaternion corresponding to the
%translation part of x. More specifically, if x = r+DQ.E*(1/2)*p*r,
%T(x) returns 1+DQ.E*(0.5)*p
function ret = T(x)
    ret = x*x.P';
end