function dq = compose_tq(t,q)
t = diag(diag(t));
dq = DQ([1;0;0;0;0.5*t])*q;

