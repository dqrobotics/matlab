%Decompositional multiplication 
function res = times(a,b)

    a = DualQuaternion(a);    
    b = DualQuaternion(b);
    
    res = b.tplus*a.tplus*DQ(b.n)*(norm(b.n)^-1)*DQ(a.n)*(norm(a.n)^-1);
end