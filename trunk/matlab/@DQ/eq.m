%dq1 == dq2 returns 1 if dq1 equals dq2, returns 0
%otherwise
function ret = eq(a,b)
    a=DQ(a);
    b=DQ(b);
    
    ret = 1;
    for i=1:8
        if abs(a.q(i) - b.q(i)) > DQ.threshold
            ret = 0;
            break
        end
    end    
end