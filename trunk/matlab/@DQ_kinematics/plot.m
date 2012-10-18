function plot(obj,joint_angles,varargin)
    
    robot = obj.robot_RT;
    has_options = 0;
    if nargin < 2
        error('Usage: plot(robot,theta,[options])');
    elseif nargin >= 3
        has_options = 1;
    end
    
    if size(joint_angles,2) == 1
        joint_angles = joint_angles';
    end
    
    % If there are dummy joints, we must plot them
    if obj.n_dummy
        theta = zeros(1,length(obj.theta));
        counter = 0;
        for i = 1:length(theta)
            if obj.dummy(i) == 1
                theta(i) = obj.theta(i);
                counter = counter + 1;
            else
                theta(i)=joint_angles(i-counter)+obj.theta(i);
            end
            
        end
    else        
        theta = joint_angles;
    end
    
    
   % if has_options
   %     plot(robot,theta+obj.theta, varargin{:});
   % else
   %     plot(robot,theta+obj.theta);
   % end
    %translating the base if necessary
    rh = findobj('Tag', robot.name);
    rr = get(rh, 'UserData');
    base = translation(obj.base);
    rr.base = transl(base.q(2:4));
    set(rh, 'UserData', rr);
    
   if has_options
        plot(robot,theta, varargin{:});
    else
        plot(robot,theta);
    end
end