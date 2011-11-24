%norm(dq) returns the dual scalar corresponding to the norm of dq
function ret = norm(dq)
    aux = dq'*dq;
    %Taking the square root of aux (to be compatible with the definition of quaternion norm)
    %This is performed based on the Taylor expansion.
    if(aux.P == 0)
        ret = DQ(0);
    else
        aux.q(1)=sqrt(aux.q(1));
        aux.q(5)=aux.q(5)/(2*aux.q(1));
        
        %Applying the threshold to the norm
        for i=1:8
            if (abs(aux.q(i)) < DQ.threshold)
                aux.q(i) = 0;
            end
        end
        ret = aux;
    end    
end