%C = DOT(a,b) returns the dot product between two dual quaternions
function c = dot(a,b)
a = DQ(a);
b = DQ(b);

c = -(a*b+b*a)*0.5;