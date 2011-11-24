% dq1 + dq2 returns the sum between dq1 and dq2
% Scalars can be used as well; for example, the operation dq + 1 is valid.

function res = plus(a,b)
    a = DQ(a);
    b = DQ(b);
    res = DQ(a.q + b.q);
end