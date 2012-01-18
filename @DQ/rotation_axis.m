%rotation_axis(dq) returns the rotation axis (nx*i + ny*j + nz*k) of the
%unit dual quaternion dq
function ret = rotation_axis(dq)
    dq = DQ(dq);
    
    if norm(dq) ~= 1
        error('The dual quaternion is non-unit');
    end
      
    phi = acos(dq.q(1));
        
    if(phi == 0)
        ret= DQ([0,0,0,1]); %This is just a convention. It could be any rotation axis.
    else        
        ret = dq.P.Im*(sin(phi)^(-1));
    end

end