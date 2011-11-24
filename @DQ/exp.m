function dq=exp(v)
theta=norm(v.n);

if(theta ~= 0)
    n = v.n/theta;
else
    n=zeros(4,1);
end

dq=compose_tq(2*v.d,DQ([cos(theta);sin(theta)*n(2:end)]));