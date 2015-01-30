%Decompositional multiplication 
function res = times(a,b)
    a = DQ(a);    
    b = DQ(b);
    
    res = T(a)*T(b)*P(a)*P(b);
end