% IS_UNIT(x) returns 1 if x is a unit norm dual quaternion, 0 otherwise
function ret = is_unit(x)
    if norm(x) == 1
        ret = 1;
    else
        ret = 0;
    end
end
