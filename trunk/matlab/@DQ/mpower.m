function x = mpower(dq,m)
% x = dq ^ m returns the dual quaternion corresponding to the operation
% exp(m*log(dq)), where dq is a dual quaternion and m is a real number. For
% the moment, this operation is defined only for unit dual quaternions.
if ~isa(m,'double')
    error('The second parameter must be a double');
end
dq = DQ(dq);

x = exp(m*log(dq));
end