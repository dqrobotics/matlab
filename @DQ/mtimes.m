% dq1 * dq2 returns the standard dual quaternion multiplication between dq1
% and dq2
% Scalars can be used as well; for example: 2 * dq1.
function res = mtimes(a,b)
    a = DQ(a);
    b = DQ(b);
    non_dual = quaternionMultiplication(a.q(1:4),b.q(1:4));
    dual = quaternionMultiplication(a.q(1:4),b.q(5:8))+quaternionMultiplication(a.q(5:8),b.q(1:4));
    res = DQ([non_dual; dual]);

end

function r = quaternionMultiplication(p,q)

P=[p(1,1),-p(2,1),-p(3,1),-p(4,1);
   p(2,1),p(1,1),-p(4,1),p(3,1);
   p(3,1),p(4,1),p(1,1),-p(2,1);
   p(4,1),-p(3,1),p(2,1),p(1,1)];

r = P*q;

end