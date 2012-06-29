function display(obj)

    row = size(obj,1);
    column = size(obj,2);
    
    disp([inputname(1),' = '])
    
    
    
    for i = 1:row   
        row_string= [];
        
        for j = 1:column
            [has_primary_element, s] = build_string(obj(i,j),0);
            [has_dual_element, sd] = build_string(obj(i,j),4);

            if has_dual_element
                sd=['E*(',sd,')'];
                if has_primary_element
                    s=['(',s,')'];
                    sd=[' + ',sd];
                end
            end

            if (has_primary_element + has_dual_element) == 0
                s = '0';
            end
            
            row_string = [row_string,'         ', [s,sd]];
            
        end
        disp(row_string);
    end
    
end

function [has_element, s] = build_string(obj, shift)
    s = '';
    aux = [' ', 'i', 'j', 'k'];
        
    has_element = 0;    
    
    for i=1:4
        if abs(obj.q(i+shift)) > DQ.threshold %To avoid printing values very close to zero
            
            
            if obj.q(i+shift) < 0           
                 s = [s,' - '];
            elseif has_element ~= 0
                s = [s,' + '];
            end
            
            if i == 1                
                s = [s,num2str(abs(obj.q(i+shift)))];
            else                   
                s = [s,num2str(abs(obj.q(i+shift))),aux(i)];   
            end
        
            has_element = 1;
        end
    end
end