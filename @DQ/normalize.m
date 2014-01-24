%C = NORMALIZE(a) normalizes the dual quaternion a
function c = normalize(a)
a = DQ(a);


c = a*inv(norm(a));