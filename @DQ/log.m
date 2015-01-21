% LOG(x) returns the logarithm of the dual quaternion x
function ret = log(x)
x = DQ(x);

if norm(x) ~= 1
    error('The log function is currently defined only for unit dual quaternions.');
end

primary = (x.rotation_angle*0.5)*x.rotation_axis;
dual = translation(x)*0.5;

ret = primary + DQ.E* dual;

end