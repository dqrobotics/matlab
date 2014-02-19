%ACOS(x) returns the acos of scalar dual quaternion x. If x is not scalar,
%the function returns an error
function ret = acos(x)
    if(Im(x) == 0) && (D(x) == 0)
        ret = acos(x.q(1));
    else
        error('The dual quaternion is not a scalar');
    end    
end