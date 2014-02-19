%ROTATION_ANGLE(X) returns the rotation angle of X or an error otherwise
function res = rotation_angle(x)
    if is_unit(x)
        res = 2*acos(Re(P(x)));
    else
        error('The dual quaternion does not have unit norm.')
    end
end