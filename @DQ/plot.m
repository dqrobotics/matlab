%plot(dq,OPTIONS) plots the dual quaternion dq. The following options are
%available:
%   'scale'
%   'erase'
% Ex.: plot(dq,'scale',5) will plot dq with the axis scaled by a factor of 5
%      
% plot(dq,'erase') will plot dq, but erasing previous values of dq. This is
% useful for plotting frames in motion (if erase is not used, it will leave a trail)
function dq=plot(dq,varargin)

if norm(dq) ~= 1
    error('Only unit dual quaternions can be plotted');
end


optargin = size(varargin,2);

scale = 1;

if optargin > 0
    for j =1:optargin
        my_string = num2str(cell2mat(varargin(1,j)));
        if findstr(my_string,'era')
           
            if(dq.isplot)
                dq.isplot = 0;
                for i = 1:3                
                    delete(dq.handle_axis{i});
                    delete(dq.handle_text{i});
                end 
            end
        elseif findstr(my_string,'sca')
                scale = cell2mat(varargin(1,j+1));
        end
    end
end

    dq.isplot = 1;
    % create unit vectors and rotate them by the quaternion part of dq.
    t1 = 1+DQ.E*[0, scale*0.5, 0, 0];
    t2 = 1+DQ.E*[0, 0, scale*0.5, 0];
    t3 = 1+DQ.E*[0, 0, 0, scale*0.5];
    
    %Do a vector rotation
	x = dq.P*t1*dq.P';
	y = dq.P*t2*dq.P';
	z = dq.P*t3*dq.P';
    
    old_ishold = ishold;
    
    dq_t = translation(dq);
    x_t = translation(x);
    y_t = translation(y);
    z_t = translation(z);
    
    dq.handle_axis{1} = plot3([0;x_t.q(2)]+dq_t.q(2), [0; x_t.q(3)]+dq_t.q(3), [0; x_t.q(4)]+dq_t.q(4), 'r','Linewidth',1,'erasemode', 'xor');
    if(~ishold)
        hold on;
    end
    
	dq.handle_text{1} = text(dq_t.q(2)+x_t.q(2), dq_t.q(3)+x_t.q(3), dq_t.q(4)+x_t.q(4), 'x');
	set(dq.handle_text{1} , 'Color', 'k');

	dq.handle_axis{2} =plot3([0;y_t.q(2)]+dq_t.q(2), [0; y_t.q(3)]+dq_t.q(3), [0; y_t.q(4)]+dq_t.q(4), 'g','Linewidth',1,'erasemode', 'xor');
	dq.handle_text{2}  = text(dq_t.q(2)+y_t.q(2), dq_t.q(3)+y_t.q(3), dq_t.q(4)+y_t.q(4), 'y');
	set(dq.handle_text{2} , 'Color', 'k');

	dq.handle_axis{3} = plot3([0;z_t.q(2)]+dq_t.q(2), [0; z_t.q(3)]+dq_t.q(3), [0; z_t.q(4)]+dq_t.q(4), 'b','Linewidth',1,'erasemode', 'xor');
	dq.handle_text{3}  = text(dq_t.q(2)+z_t.q(2), dq_t.q(3)+z_t.q(3), dq_t.q(4)+z_t.q(4), 'z');
	set(dq.handle_text{3} , 'Color', 'k');
    if(~old_ishold)
        hold off;
    end
    
end