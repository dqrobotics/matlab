%C = CROSS(a,b) returns the cross product between two dual quaternions
function c = cross(a,b)
a = DQ(a);
b = DQ(b);

c = (a*b-b*a)*0.5;