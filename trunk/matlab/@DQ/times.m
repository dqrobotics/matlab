%Decompositional multiplication 
function res = times(a,b)

    a = DQ(a);    
    b = DQ(b);
    
    res = tplus(a)*tplus(b)*a.P*b.P;
end