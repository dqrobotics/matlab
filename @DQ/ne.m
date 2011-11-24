%dq1 == dq2 returns 1 if dq1 is different from dq2, returns 0
%otherwise
function ret = ne(a,b)
    a=DQ(a);
    b=DQ(b);
    
    ret = 0;
    for i=1:8
        if abs(a.q(i) - b.q(i)) > DQ.threshold
            ret = 1;
            break
        end
    end    
end