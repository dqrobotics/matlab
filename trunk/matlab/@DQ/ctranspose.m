% dq' returns the conjugate of dq

function res = ctranspose(a)
    a = DQ(a);
    res = DQ([conjugate(a.q(1:4,1));conjugate(a.q(5:8,1))]);
end

function q = conjugate(p)

q=[p(1,1);-p(2,1);-p(3,1);-p(4,1)];
end