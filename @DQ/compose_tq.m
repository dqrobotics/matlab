function dq = compose_tq(t,q)
disp('Deprecated. Please use the dual quaternion operations to obtain what you want. If you are wondering how to do this, see the example file.')
t = diag(diag(t));
dq = DQ([1;0;0;0;0.5*t])*q;

