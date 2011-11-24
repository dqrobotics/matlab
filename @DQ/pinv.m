function ret = pinv(dq)
    temp = dq .* dq';    
    ret = temp'*dq';
end