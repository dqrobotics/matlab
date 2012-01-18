%Decompositional multiplication 
function res = times(a,b)

    a = DQ(a);    
    b = DQ(b);
    
    res = tplus(b)*tplus(a)*b.P*a.P;
end